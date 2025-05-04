/*──────────────────────────────────────────────────────────
 * IndyDualArm.hpp   —   Indy7 Dual-Arm Eigen Wrapper
 *──────────────────────────────────────────────────────────*/
#pragma once

#include <Eigen/Core>
#include <cstddef>

/* ─── Eigen 타입 별칭 ─── */
using Vector6d = Eigen::Matrix<double, 6, 1>;
constexpr std::size_t DOF = 12; // 6 DoF × 2 arms
/**
 * @brief  12-DoF DualArm 동역학/기구학
 *
 * 주요 기능
 * - Forward / Inverse Dynamics
 * - Forward Kinematics (POE)
 * - Joint/Task-space trajectory generator
 * - H∞ / QP 기반 컨트롤러
 * - slx폴더의 simulink 파일들을 기반으로 c code generation.
 * 사용 예시는 @ref examples/test_dualarm.cpp 참고.
 */
class IndyDualArm
{
public:
IndyDualArm();
~IndyDualArm();

    /**
 * @brief   단일 팔(6-DoF) 동역학/기구학 결과 구조체
 *
 * FK/ID 계산 후 **왼팔(l)**·**오른팔(r)** 모두가 이 형식을 사용합니다.
 */
    struct Arm
    {
        Eigen::Matrix<double,6,6> M;   //!< 관절질량행렬 @f$M(q)@f$
        Eigen::Matrix<double,6,1> c;   //!< 코리올리/원심 항 @f$c(q,\dot q)@f$
        Eigen::Matrix<double,6,1> g;   //!< 중력 토크 @f$g(q)@f$
        Eigen::Matrix<double,6,6> C;   //!< 코리올리/원심 항 @f$c(q,\dot q)@f$

        Eigen::Matrix4d           T;   //!< TCP 좌표계 - base 변환 (SE3)
        Vector6d                  V;   //!< TCP twist @f$[v;\,\omega]@f$

        Eigen::Matrix<double,6,6> J;   //!< body Jacobian
        Eigen::Matrix<double,6,6> Jdot; //!< @f$\dot J_b@f$
    };

/**
 * @brief  목표 궤적(desired) 묶음 (Joint- / Task-space 공용)
 *
 * <b>필드 의미</b>  
 * • **Joint-space** : 6-DoF(팔 하나) 기준 관절 궤적  
 * • **Task-space**  : TCP 기준 SE(3) 자세 · twist · twist-dot  
 *
 * 모든 필드는 “목표치”이므로, 실제값과 비교하여 오차를 만들거나  
 * 컨트롤러에 바로 넘겨 사용할 수 있습니다.
 */
struct Des
{
    /* ────────── Task-space ────────── */

    Eigen::Matrix4d T;        //!< @brief @f$T_\text{des}(t)@f$ (4×4 HT)
    Eigen::Matrix4d T_start;  //!< @brief 궤적 시작 @f$T_\text{start}@f$
    Eigen::Matrix4d T_end;    //!< @brief 궤적 종료 @f$T_\text{end}@f$

    Vector6d V;       //!< @brief 목표 twist @f$V_\text{des}(t)@f$
    Vector6d Vdot;    //!< @brief 목표 twist-dot @f$\dot V_\text{des}(t)@f$

    Vector6d V_start;     //!< twist 시작치
    Vector6d V_end;       //!< twist 종료치
    Vector6d Vdot_start;  //!< twist-dot 시작치
    Vector6d Vdot_end;    //!< twist-dot 종료치

    /* ────────── Joint-space ────────── */

    Vector6d q;     //!< @brief @f$q_\text{des}(t)@f$
    Vector6d qdot;  //!< @brief @f$\dot q_\text{des}(t)@f$
    Vector6d qddot; //!< @brief @f$\ddot q_\text{des}(t)@f$

    Vector6d q_start,  qdot_start,  qddot_start; //!< 시작점
    Vector6d q_end,    qdot_end,    qddot_end;   //!< 종료점

    /* ────────── 위치·오리엔테이션 보조 ────────── */

    Eigen::Vector3d p;      //!< 위치 @f$\mathbf{p}(t)@f$
    Eigen::Vector3d pdot;   //!< 속도 @f$\dot{\mathbf{p}}(t)@f$
    Eigen::Vector3d pddot;  //!< 가속도 @f$\ddot{\mathbf{p}}(t)@f$

    Eigen::Vector3d xi;     //!< @f$\xi = \log R(t)@f$
    Eigen::Vector3d xidot;  //!< @f$\dot\xi@f$
    Eigen::Vector3d xiddot; //!< @f$\ddot\xi@f$
};

/**
 * @brief  두 팔(12 DoF) 기준 <em>궤적(Desired)</em> 묶음
 *
 * ‣ 모든 항목은 “좌(0 – 5) ∥ 우(6 – 11)” 관절을 하나로 묶어 12 × 1 / 12 × 12
 *   형식을 따릅니다.  
 * ‣ Task-space 항목은 <em>좌·우 TCP 사이의 상대 자세 · twist</em>를 의미합니다.
 */
struct RelDes
{
    /* ─── Task-space ─── */
    Eigen::Matrix4d  T;      //!< @brief 좌-우 TCP 상대 HT @f$T_{\mathrm{rel}}(t)@f$
    Vector6d         V;      //!< @brief 상대 twist            @f$V_{\mathrm{rel}}(t)@f$
    Vector6d         Vdot;   //!< @brief 상대 twist-dot        @f$\dot V_{\mathrm{rel}}(t)@f$

    /* ─── Joint-space (12 DoF) ─── */
    Eigen::Matrix<double,12,1> q;     //!< @f$q_\text{des}(t)@f$
    Eigen::Matrix<double,12,1> qdot;  //!< @f$\dot q_\text{des}(t)@f$
    Eigen::Matrix<double,12,1> qddot; //!< @f$\ddot q_\text{des}(t)@f$
};

/**
 * @brief  두 팔(12 DoF) 기준 <em>동역학 / 기구학</em> 상태
 *
 * * `M`, `c`, `g` : 12 DoF 전관절 관성·코리올리·중력  
 * * 나머지 필드 : 좌·우 TCP 사이 **상대** 자세·twist·야코비안
 */
struct RelArm
{
    /* ─── 동역학 (12 DoF) ─── */
    Eigen::Matrix<double,12,12> M;   //!< @brief 관성행렬 @f$M(q)@f$
    Eigen::Matrix<double,12,1>  c;   //!< @brief 코리올리/원심 @f$c(q,\dot q)@f$
    Eigen::Matrix<double,12,1>  g;   //!< @brief 중력항     @f$g(q)@f$
    Eigen::Matrix<double,12,12> C;   //!< 코리올리/원심 항 @f$c(q,\dot q)@f$

    /* ─── Task-space 상대 값 ─── */
    Eigen::Matrix4d  T;               //!< @brief 상대 HT @f$T_{\mathrm{rel}}@f$
    Vector6d         V;               //!< @brief 상대 twist

    Eigen::Matrix<double,6,12> J;     //!< @brief 상대 Jacobian
    Eigen::Matrix<double,6,12> Jdot;  //!< @brief @f$\dot J@f$
};


Eigen::Matrix<double, 6, 7> lambda_l;   //!< LocalPOE screw axis (left)
Eigen::Matrix<double, 6, 7> lambda_r;   //!< LocalPOE screw axis (right)
Eigen::Matrix<double, 6, 13> lambda_lr; //!< LocalPOE screw axis (full)

Eigen::Matrix<double, 12, 1> q_init;     //!< 초기 관절각 [rad]
Eigen::Matrix<double, 12, 1> q_min;      //!< 관절 하한 [rad]
Eigen::Matrix<double, 12, 1> q_max;      //!< 관절 상한 [rad]
Eigen::Matrix<double, 12, 1> qdot_min;   //!< 속도 하한 [rad/s]
Eigen::Matrix<double, 12, 1> qdot_max;   //!< 속도 상한 [rad/s]

  
    // Arm l;       //!< 왼팔 동역학/기구학 결과
    // Arm r;       //!< 오른팔 동역학/기구학 결과
    // RelArm lr;   //!< 전체 12-DoF 동역학/기구학 결과
    // Arm nom_l;       //!< 왼팔 동역학/기구학 결과
    // Arm nom_r;       //!< 오른팔 동역학/기구학 결과
    // RelArm nom_lr;   //!< 전체 12-DoF 동역학/기구학 결과
    
    // Des des_l;   //!< 왼팔 목표 궤적
    // Des des_r;   //!< 오른팔 목표 궤적
    // RelDes des_lr; //!< 통합 12-DoF 목표 궤적
    /* Life-cycle */
    void initialize();                              ///< FD/FK/ID/JSTraj/TSTraj 모두 *_initialize()
    void initialize(const Eigen::VectorXd &q_init); ///< FD/FK/ID/JSTraj/TSTraj 모두 *_initialize()
    void initialize(const std::string& urdf_path,const Eigen::VectorXd &q_init);//pinocchio
    
    void initialize(const Eigen::VectorXd &q0, const Eigen::Matrix<double, 6, 7> &lambda_l, const Eigen::Matrix<double, 6, 7> &lambda_r, const Eigen::Matrix<double, 6, 13> &lambda_lr);
    void initialize(const Eigen::VectorXd &q0, const Eigen::Matrix<double, 12, 1> &q_min, const Eigen::Matrix<double, 12, 1> &q_max, const Eigen::Matrix<double, 12, 1> &qdot_min, const Eigen::Matrix<double, 12, 1> &qdot_max);

    void terminate(); ///< 위 모듈 전부 *_terminate()
    /* 1) Forward Dynamics : τ → (q, q̇) */
    void forwardDynamics(const Eigen::VectorXd &tau,
                         Eigen::VectorXd &q_next,
                         Eigen::VectorXd &qd_next,
                         const Vector6d &Fext_r = Vector6d::Zero(),
                         const Vector6d &Fext_l = Vector6d::Zero());
    void forwardDynamicsNom(const Eigen::VectorXd &tau,
                            Eigen::VectorXd &q_next,
                            Eigen::VectorXd &qd_next,
                            const Vector6d &Fext_r = Vector6d::Zero(),
                            const Vector6d &Fext_l = Vector6d::Zero());
    /* 2) Forward Kinematics : q → Tₗ, Tᵣ */
    void updateFK(const Eigen::VectorXd&q,
        const Eigen::VectorXd& qdot, Arm &arm_l,Arm &arm_r,RelArm &arm_lr);
        void updateFK(const Eigen::VectorXd&q,
            const Eigen::VectorXd& qdot, Arm &arm_l,Arm &arm_r,RelArm &arm_lr,const Eigen::Matrix<double, 6, 7> &lambda_l, const Eigen::Matrix<double, 6, 7> &lambda_r, const Eigen::Matrix<double, 6, 13> &lambda_lr);
    /* 3) Inverse Dynamics : (q,q̇,q̈) → (M,c,g) */
    void updateID(const Eigen::VectorXd&q,
        const Eigen::VectorXd& qdot, Arm &arm_l,Arm &arm_r,RelArm &arm_lr);

    /* … 앞부분 동일 … */
    Eigen::VectorXd NRIC(const Eigen::VectorXd &HinfK, const Eigen::VectorXd &q_r, const Eigen::VectorXd &qdot_r,const Eigen::VectorXd&q_n,const Eigen::VectorXd &qdot_n, Eigen::VectorXd &eint_nr,double dt){
        Eigen::VectorXd e_nr,edot_nr,tau_a;
        e_nr = q_n-q_r;
        edot_nr= qdot_n-qdot_r;
        eint_nr +=dt*e_nr;
        tau_a = HinfK.asDiagonal()*(edot_nr+20.0*e_nr+100.0*eint_nr);
        return tau_a;
    }
    // void inverseDynamicsNom(const Eigen::VectorXd &q_lr_nom,
    //     const Eigen::VectorXd &qdot_lr_nom);
    /* 4) Joint-Space Trajectory : (t, qₛ, qₑ, T0, Tf) → (q_d, q̇_d, q̈_d) */
    Des jointSpaceTrajectory(double t_sec,
        const Vector6d &q_start,
        const Vector6d &q_end,
        double T0,
        double Tf );

  /**
     * @brief des_l, des_r 궤적을 통합하여 des_lr (12-DoF)로 설정합니다.
     * 
     * 이 함수는 H∞, TaskSpaceController 등의 joint-level 입력을 준비할 때 사용됩니다.
     * 
     * @param des_l 왼팔 궤적 (q, qdot, qddot)
     * @param des_r 오른팔 궤적 (q, qdot, qddot)
     */
    RelDes setTraj(const Des &des_l, const Des &des_r)
    {
        RelDes des_lr;
        des_lr.q.segment(0, 6) = des_l.q;
        des_lr.qdot.segment(0, 6) = des_l.qdot;
        des_lr.qddot.segment(0, 6) = des_l.qddot;
        des_lr.q.segment(6, 6) = des_r.q;
        des_lr.qdot.segment(6, 6) = des_r.qdot;
        des_lr.qddot.segment(6, 6) = des_r.qddot;
        return des_lr;
    }

    /* 5) Task-Space Trajectory : (t, Tₛ, Tₑ, T0, Tf) → (T, V, V̇, p, ξ, …) */
    Des taskSpaceTrajectory(double               t_sec,
        const Eigen::Matrix4d& T_start,
        const Eigen::Matrix4d& T_end,
        double               T0,
        double               Tf);
 
    /**
     * @brief Task-space controller (SPD + QP) 기반 토크 계산
     * 
     * `TaskSpaceController_step()` 호출. FK, Trajectory가 선행되어야 함.
     * 내부적으로 l, r, lr, des_l, des_r, des_lr 등의 필드를 사용합니다.
     * 
     * @param q        현재 관절각 (12×1)
     * @param qdot     현재 관절속도 (12×1)
     * @param dt       제어 루프 주기 [sec]
     * @param TaskKp   태스크 공간 P 게인 (12×1)
     * @param TaskKv   태스크 공간 D 게인 (12×1)
     * @param b0       Damping coeff
     * @param a        Damping coeff
     * @param HinfK    H∞ Gain
     * @return 토크 출력 τ (12×1)
     */
    Eigen::VectorXd taskSpaceController(
        const Arm l,
        const Arm r,
        const RelArm lr,
        const Eigen::VectorXd&               q,
        const Eigen::VectorXd&               qdot,
        const  Des &des_l,
        const  Des &des_r,
        double                               dt,
        const Eigen::VectorXd& q_init,
        const Eigen::VectorXd& q_max,
        const Eigen::VectorXd& q_min,
        const Eigen::VectorXd& qdot_max,
        const Eigen::VectorXd& qdot_min,
        const Eigen::VectorXd&               TaskKp    ,
        const Eigen::VectorXd&               TaskKv    ,
        const Eigen::Vector2d&               b0         ,
        const Eigen::Vector2d&               a      ,
        const Eigen::VectorXd&               HinfK );

        Eigen::MatrixXd MassMatrix(const Eigen::VectorXd &q);
        Eigen::VectorXd GravityForces(const Eigen::VectorXd& q);
        Eigen::MatrixXd CoriolisMatrix(const Eigen::VectorXd& q,const Eigen::VectorXd& qdot);
        Eigen::MatrixXd MassMatrixInverse(const Eigen::VectorXd& q);
        Eigen::VectorXd HinfControl(const RelArm &lr,const Eigen::VectorXd q, const Eigen::VectorXd q_dot, const  RelDes &des, Eigen::VectorXd& e_int, double dt, const Eigen::VectorXd Hinf_K, const Eigen::VectorXd gamma);
        Eigen::VectorXd HinfControl(const Arm &arm, const Eigen::VectorXd q, const Eigen::VectorXd q_dot, const  Des &des, Eigen::VectorXd& e_int, double dt, const Eigen::VectorXd Hinf_K, const Eigen::VectorXd gamma);
private:
    /**
     * @brief Simulink 모듈을 전부 초기화합니다.
     * 
     * 내부적으로 FD/FK/ID/JSTraj/TSTraj/HinfController/TaskSpaceController를 초기화합니다.
     */
    void initModules_(); ///< @internal 직접 호출하지 마세요
    /**
     * @brief setLambda()을 통해 기본 Local POE screw parameter 설정
     * 
     * lambda_l : 왼팔  λ ∈ ℝ⁶ˣ⁷  
     * lambda_r : 오른팔 λ ∈ ℝ⁶ˣ⁷  
     * lambda_lr: 전체 λ ∈ ℝ⁶ˣ¹³ (좌+우)
     */
    void setLambda();   ///< @internal 사용자 초기화 override 시 자동 호출됨
    /**
     * @brief Joint position/velocity 상한/하한을 기본값(±π rad)으로 설정합니다.
     */
    void setMinMax();   ///< @internal 사용자 지정 initialize(q, qmin, qmax, …) 전용
    void pinocchio_initialize(const std::string&  urdf_path);
    struct Impl;
    Impl* pimpl; 
};
