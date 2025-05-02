/*──────────────────────────────────────────────────────────
 * IndyDualArm.hpp   —   Indy7 Dual-Arm Eigen Wrapper
 *──────────────────────────────────────────────────────────*/
#pragma once
#include <Eigen/Core>
#include <cstddef>

/* ─── Eigen 타입 별칭 ─── */
using  Vector6d = Eigen::Matrix<double,6,1>;
constexpr std::size_t DOF = 12;                 // 6 DoF × 2 arms

/* ─── code-gen C 헤더 전부 포함 ─── */


/* ─────────── 래퍼 클래스 ─────────── */
class IndyDualArm
{
public:
    typedef struct {
        Eigen::Matrix<double,6,6> M;
        Eigen::Matrix<double,6,1> c;
        Eigen::Matrix<double,6,1> g;
        Eigen::Matrix4d T;
        Vector6d V;
        Eigen::Matrix<double,6,6> J;
        Eigen::Matrix<double,6,6> Jdot;
        
    } Arm;

    typedef struct {
 
        Eigen::Matrix4d T;
        Eigen::Matrix4d T_start;
        Eigen::Matrix4d T_end;
        Vector6d V;
        Vector6d Vdot;
        Vector6d V_start;
        Vector6d V_end;
        Vector6d Vdot_start;
        Vector6d Vdot_end;
        

        Eigen::Matrix<double,6,1> q;
        Eigen::Matrix<double,6,1> qdot;
        Eigen::Matrix<double,6,1> qddot;

        Eigen::Matrix<double,6,1> q_start;
        Eigen::Matrix<double,6,1> qdot_start;
        Eigen::Matrix<double,6,1> qddot_start;
        Eigen::Matrix<double,6,1> q_end;
        Eigen::Matrix<double,6,1> qdot_end;
        Eigen::Matrix<double,6,1> qddot_end;  
        Eigen::Matrix<double,3,1> p;
        Eigen::Matrix<double,3,1> pdot;
        Eigen::Matrix<double,3,1> pddot;
        Eigen::Matrix<double,3,1> xi;
        Eigen::Matrix<double,3,1> xidot;
        Eigen::Matrix<double,3,1> xiddot;
        


    } Des; 
    typedef struct {
 
        Eigen::Matrix4d T;
        Vector6d V;
        Vector6d Vdot;
        Eigen::Matrix<double,12,1> q;
        Eigen::Matrix<double,12,1> qdot;
        Eigen::Matrix<double,12,1> qddot;
        
    } RelDes;   
    typedef struct {
        Eigen::Matrix<double,12,12> M;
        Eigen::Matrix<double,12,1> c;
        Eigen::Matrix<double,12,1> g;
        Eigen::Matrix4d T;
        Vector6d V;
        Eigen::Matrix<double,6,12> J;
        Eigen::Matrix<double,6,12> Jdot;
    } RelArm;

    Eigen::Matrix<double, 6, 7> lambda_l; // LocalPOE Parameters
    Eigen::Matrix<double, 6, 7> lambda_r;
    Eigen::Matrix<double, 6, 13> lambda_lr;
    Eigen::Matrix<double, 12, 1> q_init; 
    Eigen::Matrix<double, 12, 1> q_min;
    Eigen::Matrix<double, 12, 1> q_max;
    Eigen::Matrix<double, 12, 1> qdot_min;
    Eigen::Matrix<double, 12, 1> qdot_max;
    
    Arm l;
    Arm r;
    RelArm lr;
    Des des_l;
    Des des_r;
    RelDes des_lr;
    /* Life-cycle */
    void initialize();      ///< FD/FK/ID/JSTraj/TSTraj 모두 *_initialize()
    void initialize(const Eigen::VectorXd& q_init);      ///< FD/FK/ID/JSTraj/TSTraj 모두 *_initialize()
    void initialize(const Eigen::VectorXd &q0,const Eigen::Matrix<double,6,7> &lambda_l,const Eigen::Matrix<double,6,7> &lambda_r,const Eigen::Matrix<double,6,13> &lambda_lr );
    void initialize(const Eigen::VectorXd &q0,const Eigen::Matrix<double,12,1> &q_min,const Eigen::Matrix<double,12,1> &q_max,const Eigen::Matrix<double,12,1> &qdot_min,const Eigen::Matrix<double,12,1> &qdot_max);

    void terminate();       ///< 위 모듈 전부 *_terminate()
    /* 1) Forward Dynamics : τ → (q, q̇) */
    void forwardDynamics(const Eigen::VectorXd& tau,
                         Eigen::VectorXd&       q_next,
                         Eigen::VectorXd&       qd_next,
                         const Vector6d& Fext_r = Vector6d::Zero(),
                         const Vector6d& Fext_l = Vector6d::Zero());

    /* 2) Forward Kinematics : q → Tₗ, Tᵣ */
    void forwardKinematics(const Eigen::VectorXd&  q,const Eigen::VectorXd&  qdot);

    /* 3) Inverse Dynamics : (q,q̇,q̈) → (M,c,g) */
    void inverseDynamics(const Eigen::VectorXd& q,
                         const Eigen::VectorXd& qdot);

/* … 앞부분 동일 … */

/* 4) Joint-Space Trajectory : (t, qₛ, qₑ, T0, Tf) → (q_d, q̇_d, q̈_d) */
void jointSpaceTrajectory(double t_sec,
    const Vector6d &q_start,
    const Vector6d &q_end,
    double T0,
    double Tf,
    Des& des);

/* … 나머지 동일 … */
void setTraj(const Des &des_l,const Des &des_r ){
    des_lr.q.segment(0,6) = des_l.q;
    des_lr.qdot.segment(0,6) = des_l.qdot;
    des_lr.qddot.segment(0,6) = des_l.qddot;
    des_lr.q.segment(6,6) = des_r.q;
    des_lr.qdot.segment(6,6) = des_r.qdot;
    des_lr.qddot.segment(6,6) = des_r.qddot;
}

/* 5) Task-Space Trajectory : (t, Tₛ, Tₑ, T0, Tf) → (T, V, V̇, p, ξ, …) */
void taskSpaceTrajectory(double               t_sec,
    const Eigen::Matrix4d& T_start,
    const Eigen::Matrix4d& T_end,
    double               T0,
    double               Tf,
    Des &des);

    Eigen::VectorXd hinfController(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& qdot,
        const Eigen::VectorXd& K);
    Eigen::VectorXd   taskSpaceController(
        const Eigen::VectorXd&               q,
        const Eigen::VectorXd&               qdot,
        double                               dt,
        const Eigen::VectorXd&               TaskKp    ,
        const Eigen::VectorXd&               TaskKv    ,
        const Eigen::Vector2d&               b0         ,
        const Eigen::Vector2d&               a      ,
        const Eigen::VectorXd&               HinfK      );                


    private:
    void initModules_();  // FD / FK / ID / JSTraj / TSTraj *_initialize()
    void setLambda();
    void setMinMax();


};
