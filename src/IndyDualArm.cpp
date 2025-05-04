/*──────────────── IndyDualArm.cpp ─────────────────────────*/
#include "IndyDualArm.hpp"
#include <cstring> // std::memcpy
#include <stdexcept>
#ifndef PINOCCHIO_HEADER_H
#define PINOCCHIO_HEADER_H
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/aba.hpp"
#endif // pinocchio_header
extern "C"
{
#include "FD.h"     /* Forward Dynamics */
#include "FK.h"     /* Forward Kinematics */
#include "ID.h"     /* Inverse Dynamics  */
#include "JSTraj.h" /* Joint-Space Traj  */
#include "TSTraj.h" /* Task-Space  Traj  */
#include "HinfController.h"
#include "TaskSpaceController.h"
#include "FD_nom.h"
#include "ID_nom.h"

}

#define COPY_TO(dst, src, n) std::memcpy((dst), (src), (n) * sizeof(double))
#define COPY_FROM(src, dst, n) std::memcpy((dst), (src), (n) * sizeof(double))
struct IndyDualArm::Impl {
    pinocchio::Model model;
    pinocchio::Data  data;
    // … 내부 멤버 …
};
void IndyDualArm::initModules_()
{
    FD_initialize();
    FD_nom_initialize();
    
    FK_initialize();
    ID_initialize();
    ID_nom_initialize();
    JSTraj_initialize();
    TSTraj_initialize();
    HinfController_initialize();
    TaskSpaceController_initialize();
}
void IndyDualArm::setMinMax(){
     q_max = Eigen::VectorXd::Ones(12)*3.141592;
     q_min = Eigen::VectorXd::Ones(12)*(-3.141592);

     qdot_max = Eigen::VectorXd::Ones(12)*3.141592;
     qdot_min = Eigen::VectorXd::Ones(12)*(-3.141592);
}

void IndyDualArm::setLambda(){
        lambda_l<<  0,-0.111945470245711,-0.450000000000000,-0.232084291010446,-0.00722667581655122,-0.0911029490978083,6.11257062427590e-36,
        -1.17357539658372,0.0246912114191473,0,0.181466898367024,-0.0447110760186115,0.0286221059600170,-2.25514051876985e-16,
        0.993815552956183,0.243747835971880,-0.0305000000000000,0.141365128073726,0.145068226710239,0.174516399919260,0.0599999999999999,
        -2.09100000000000,1.20917417943659,0,-1.20917417943659,1.20917417943659,-1.20917417943659,0,
        0,1.20917417943659,0,-1.20905770917293,1.20917417943659,-1.20905770917293,0,
        0,-1.20905770917293,0,1.20917417943659,-1.20905770917293,1.20917417943659,0;
        lambda_r<<            0   ,  -0.11195  ,      -0.45  ,   -0.23208 ,  -0.0072267    ,-0.091103,  -2.5885e-20,
                        1.1762    , 0.024691  , 1.7347e-18   ,   0.18147  ,  -0.044711     ,0.028622 ,  2.2551e-16,
                        0.9923    ,  0.24375  ,    -0.0305   ,   0.14137  ,    0.14507     , 0.17452 ,        0.06,
                        2.091     ,  1.2092   ,1.0434e-16    ,  -1.2092   ,    1.2092      ,-1.2092  ,-1.8334e-16,
                            0     ,  1.2092  ,-5.5908e-17    ,  -1.2091   ,    1.2092      ,-1.2091  ,-4.0658e-20,
                            0     , -1.2091  , 1.3175e-17    ,   1.2092   ,   -1.2091      , 1.2092  ,          0;
        lambda_lr<<-6.11257062427590e-36,0.0911029490978083,0.00722667581655122,0.232084291010446,0.450000000000000,0.111945470245711,0,-0.111945470245711,-0.450000000000000,-0.232084291010446,-0.00722667581655134,-0.0911029490978080,-2.58853268680914e-20,
        2.25514051876985e-16,-0.0286221059600170,0.0447110760186115,-0.181466898367024,0,-0.0246912114191473,0.541044112920571,0.0246912114191473,1.73472347597681e-18,0.181466898367023,-0.0447110760186111,0.0286221059600170,2.25514051876985e-16,
        -0.0599999999999999,-0.174516399919260,-0.145068226710239,-0.141365128073726,0.0305000000000000,-0.243747835971880,-0.00150453352734745,0.243747835971880,-0.0305000000000000,0.141365128073727,0.145068226710239,0.174516399919260,0.0600000000000000,
        0,1.20917417943659,-1.20917417943659,1.20917417943659,0,-1.20917417943659,-2.10118530717959,1.20917417943659,-1.04335921901920e-16,-1.20917417943659,1.20917417943659,-1.20917417943659,1.83339582048191e-16,
        0,1.20905770917293,-1.20917417943659,1.20905770917293,0,-1.20917417943659,0,1.20917417943659,5.59082217244013e-17,-1.20905770917293,1.20917417943659,-1.20905770917293,4.06575814682064e-20,
        0,-1.20917417943659,1.20905770917293,-1.20917417943659,0,1.20905770917293,0,-1.20905770917293,-1.31751732895636e-17,1.20917417943659,-1.20905770917293,1.20917417943659,0;
}
IndyDualArm::IndyDualArm() {
    pimpl = new Impl();
    // Pinocchio 모델 초기화 등…
  }
  
  IndyDualArm::~IndyDualArm() {
    delete pimpl;
  }
  
/* ───────── Life-cycle ───────── */
void IndyDualArm::initialize()
{
    initModules_(); // 단순 모듈 초기화
    setLambda();
    setMinMax();
}
void IndyDualArm::pinocchio_initialize(const std::string&  urdf_path){
    pinocchio::urdf::buildModel(urdf_path, pimpl->model);
    Eigen::Vector3d g;
    g<<0,0,-9.80665;
    pimpl->model.gravity.linear(g);
}
void IndyDualArm::terminate()
{
    TSTraj_terminate();
    JSTraj_terminate();
    ID_terminate();
    ID_nom_terminate();
    
    FK_terminate();
    FD_terminate();
    FD_nom_terminate();
    HinfController_terminate();
    TaskSpaceController_terminate();
}
void IndyDualArm::initialize(const Eigen::VectorXd &q0)
{
    if (q0.size() != DOF)
        throw std::invalid_argument("q_init must be 12×1 vector");

    /* L-Arm (index 0-5) */
    q_init_l1 = q0[0];
    q_init_l2 = q0[1];
    q_init_l3 = q0[2];
    q_init_l4 = q0[3];
    q_init_l5 = q0[4];
    q_init_l6 = q0[5];

    /* R-Arm (index 6-11) */
    q_init_r1 = q0[6];
    q_init_r2 = q0[7];
    q_init_r3 = q0[8];
    q_init_r4 = q0[9];
    q_init_r5 = q0[10];
    q_init_r6 = q0[11];
    q_init = q0;

    initModules_(); 
    setLambda();
    setMinMax();
}

void IndyDualArm::initialize(const std::string& urdf_path,const Eigen::VectorXd &q0)
{

    if (q0.size() != DOF)
        throw std::invalid_argument("q_init must be 12×1 vector");

    /* L-Arm (index 0-5) */
    q_init_l1 = q0[0];
    q_init_l2 = q0[1];
    q_init_l3 = q0[2];
    q_init_l4 = q0[3];
    q_init_l5 = q0[4];
    q_init_l6 = q0[5];

    /* R-Arm (index 6-11) */
    q_init_r1 = q0[6];
    q_init_r2 = q0[7];
    q_init_r3 = q0[8];
    q_init_r4 = q0[9];
    q_init_r5 = q0[10];
    q_init_r6 = q0[11];
    q_init = q0;
    pinocchio_initialize(urdf_path);
    initModules_(); 
    setLambda();
    setMinMax();
}
void IndyDualArm::initialize(const Eigen::VectorXd &q0,const Eigen::Matrix<double,6,7> &lambda_l,const Eigen::Matrix<double,6,7> &lambda_r,const Eigen::Matrix<double,6,13> &lambda_lr )
{
    if (q0.size() != DOF)
        throw std::invalid_argument("q_init must be 12×1 vector");

    /* L-Arm (index 0-5) */
    q_init_l1 = q0[0];
    q_init_l2 = q0[1];
    q_init_l3 = q0[2];
    q_init_l4 = q0[3];
    q_init_l5 = q0[4];
    q_init_l6 = q0[5];

    /* R-Arm (index 6-11) */
    q_init_r1 = q0[6];
    q_init_r2 = q0[7];
    q_init_r3 = q0[8];
    q_init_r4 = q0[9];
    q_init_r5 = q0[10];
    q_init_r6 = q0[11];

    initModules_(); // 설정값 반영하여 초기화
    this->lambda_l  =lambda_l;
    this->lambda_r  =lambda_r;
    this->lambda_lr  =lambda_lr;
    this->q_init = q0;

}
void IndyDualArm::initialize(const Eigen::VectorXd &q0,const Eigen::Matrix<double,12,1> &q_min,const Eigen::Matrix<double,12,1> &q_max,const Eigen::Matrix<double,12,1> &qdot_min,const Eigen::Matrix<double,12,1> &qdot_max){
    if (q0.size() != DOF)
    throw std::invalid_argument("q_init must be 12×1 vector");

    /* L-Arm (index 0-5) */
    q_init_l1 = q0[0];
    q_init_l2 = q0[1];
    q_init_l3 = q0[2];
    q_init_l4 = q0[3];
    q_init_l5 = q0[4];
    q_init_l6 = q0[5];

    /* R-Arm (index 6-11) */
    q_init_r1 = q0[6];
    q_init_r2 = q0[7];
    q_init_r3 = q0[8];
    q_init_r4 = q0[9];
    q_init_r5 = q0[10];
    q_init_r6 = q0[11];

    
    this->q_min  =q_min;
    this->q_max  =q_max;
    this->qdot_min  =qdot_min;
    this->qdot_max  =qdot_max;
    this->q_init = q0;
    initModules_(); // 설정값 반영하여 초기화
    

}
/*───────── 1) Forward Dynamics ───────────────────────*/
void IndyDualArm::forwardDynamics(const Eigen::VectorXd &tau,
                                  Eigen::VectorXd &q_next,
                                  Eigen::VectorXd &qd_next,
                                  const Vector6d &Fext_r,
                                  const Vector6d &Fext_l)
{
    if (tau.size() != DOF)
        throw std::invalid_argument("tau size ≠ 12");

    COPY_FROM(tau.data(), FD_U.tau, DOF);
    COPY_FROM(Fext_r.data(), FD_U.Fext_r, 6);
    COPY_FROM(Fext_l.data(), FD_U.Fext_l, 6);

    FD_step();

    q_next.resize(DOF);
    qd_next.resize(DOF);
    COPY_TO(q_next.data(), FD_Y.q, DOF);
    COPY_TO(qd_next.data(), FD_Y.qdot, DOF);
}
void IndyDualArm::forwardDynamicsNom(const Eigen::VectorXd &tau,
    Eigen::VectorXd &q_next,
    Eigen::VectorXd &qd_next,
    const Vector6d &Fext_r,
    const Vector6d &Fext_l)
{
if (tau.size() != DOF)
throw std::invalid_argument("tau size ≠ 12");

COPY_FROM(tau.data(), FD_nom_U.tau, DOF);
COPY_FROM(Fext_r.data(), FD_nom_U.Fext_r, 6);
COPY_FROM(Fext_l.data(), FD_nom_U.Fext_l, 6);

FD_nom_step();

q_next.resize(DOF);
qd_next.resize(DOF);
COPY_TO(q_next.data(), FD_nom_Y.q, DOF);
COPY_TO(qd_next.data(), FD_nom_Y.qdot, DOF);
}

// void updateFK(const Eigen::VectorXd&q,
//     const Eigen::VectorXd& qdot, Arm &arm_l,Arm &arm_r,RelArm &arm_lr);


/*───────── 2) Forward Kinematics ─────────────────────*/
void IndyDualArm::updateFK(const Eigen::VectorXd&q,
    const Eigen::VectorXd& qdot, IndyDualArm::Arm &arm_l,IndyDualArm::Arm &arm_r,IndyDualArm::RelArm &arm_lr)
{
/* ─── 입력 유효성 검사 ─── */
if(q.size()!=DOF || qdot.size()!=DOF)
throw std::invalid_argument("q / qdot size must be 12");

/* ─── ExtU 에 복사 ─── */
std::memcpy(FK_U.q     , q.data()    , DOF*sizeof(double));
std::memcpy(FK_U.qdot  , qdot.data() , DOF*sizeof(double));
std::memcpy(FK_U.lambda_l , this->lambda_l.data() , 42 * sizeof(double));
std::memcpy(FK_U.lambda_r , this->lambda_r.data() , 42 * sizeof(double));
std::memcpy(FK_U.lambda_lr, this->lambda_lr.data(), 78 * sizeof(double));

/* 1-step 수행 */
FK_step();

/* ─── 필수 출력 ─── */
std::memcpy(arm_l.T .data(), FK_Y.T_l , 16*sizeof(double));
std::memcpy(arm_r.T.data(), FK_Y.T_r , 16*sizeof(double));
std::memcpy(arm_lr.T.data(), FK_Y.T_lr , 16*sizeof(double));
std::memcpy(arm_l.J.data() , FK_Y.Jb_l , 36*sizeof(double));
std::memcpy(arm_r.J.data() , FK_Y.Jb_r , 36*sizeof(double));
std::memcpy(arm_lr.J.data(), FK_Y.Jb_lr, 72*sizeof(double));
std::memcpy(arm_l.Jdot.data() , FK_Y.Jbdot_l , 36*sizeof(double));
std::memcpy(arm_r.Jdot.data() , FK_Y.Jbdot_r , 36*sizeof(double));
std::memcpy(arm_lr.Jdot.data(), FK_Y.Jbdot_lr, 72*sizeof(double));
std::memcpy(arm_l.V.data()  , FK_Y.V_l  ,  6*sizeof(double));
std::memcpy(arm_r.V.data()  , FK_Y.V_r  ,  6*sizeof(double));
std::memcpy(arm_lr.V.data()  , FK_Y.V_lr  ,  6*sizeof(double));


}
void IndyDualArm::updateFK(const Eigen::VectorXd&q,
    const Eigen::VectorXd& qdot, IndyDualArm::Arm &arm_l,IndyDualArm::Arm &arm_r,IndyDualArm::RelArm &arm_lr,const Eigen::Matrix<double, 6, 7> &lambda_l, const Eigen::Matrix<double, 6, 7> &lambda_r, const Eigen::Matrix<double, 6, 13> &lambda_lr)
{
/* ─── 입력 유효성 검사 ─── */
if(q.size()!=DOF || qdot.size()!=DOF)
throw std::invalid_argument("q / qdot size must be 12");

/* ─── ExtU 에 복사 ─── */
std::memcpy(FK_U.q     , q.data()    , DOF*sizeof(double));
std::memcpy(FK_U.qdot  , qdot.data() , DOF*sizeof(double));
std::memcpy(FK_U.lambda_l , lambda_l.data() , 42 * sizeof(double));
std::memcpy(FK_U.lambda_r , lambda_r.data() , 42 * sizeof(double));
std::memcpy(FK_U.lambda_lr,lambda_lr.data(), 78 * sizeof(double));

/* 1-step 수행 */
FK_step();

/* ─── 필수 출력 ─── */
std::memcpy(arm_l.T .data(), FK_Y.T_l , 16*sizeof(double));
std::memcpy(arm_r.T.data(), FK_Y.T_r , 16*sizeof(double));
std::memcpy(arm_lr.T.data(), FK_Y.T_lr , 16*sizeof(double));
std::memcpy(arm_l.J.data() , FK_Y.Jb_l , 36*sizeof(double));
std::memcpy(arm_r.J.data() , FK_Y.Jb_r , 36*sizeof(double));
std::memcpy(arm_lr.J.data(), FK_Y.Jb_lr, 72*sizeof(double));
std::memcpy(arm_l.Jdot.data() , FK_Y.Jbdot_l , 36*sizeof(double));
std::memcpy(arm_r.Jdot.data() , FK_Y.Jbdot_r , 36*sizeof(double));
std::memcpy(arm_lr.Jdot.data(), FK_Y.Jbdot_lr, 72*sizeof(double));
std::memcpy(arm_l.V.data()  , FK_Y.V_l  ,  6*sizeof(double));
std::memcpy(arm_r.V.data()  , FK_Y.V_r  ,  6*sizeof(double));
std::memcpy(arm_lr.V.data()  , FK_Y.V_lr  ,  6*sizeof(double));


}

/*───────── 3) Inverse Dynamics ───────────────────────*/
// /* 3) Inverse Dynamics : (q,q̇,q̈) → (M,c,g) */
// void updateID(const Eigen::VectorXd&q,
//     const Eigen::VectorXd& qdot, Arm &arm_l,Arm &arm_r,RelArm &arm_lr);
void IndyDualArm::updateID(const Eigen::VectorXd &q_lr,
    const Eigen::VectorXd &qdot_lr, IndyDualArm::Arm &l,IndyDualArm::Arm &r,IndyDualArm::RelArm &lr)
{
    if (q_lr.size() != DOF || qdot_lr.size() != DOF)
    throw std::invalid_argument("q / qdot size ≠ 12");
    // Eigen::VectorXd q_lr_temp=q_lr;
    // q_lr_temp(0) +=3.141592;
    COPY_FROM(q_lr.data(), ID_U.q, DOF);
    COPY_FROM(qdot_lr.data(), ID_U.qdot, DOF);

    ID_step();

    COPY_TO(lr.M.data(), ID_Y.M, DOF * DOF);

    COPY_TO(lr.c.data(), ID_Y.c, DOF);
    COPY_TO(lr.g.data(), ID_Y.g, DOF);

    r.M = lr.M.block<6,6>(6,6);
    l.M = lr.M.block<6,6>(0,0);
    r.c = lr.c.segment(6,6);
    l.c = lr.c.segment(0,6);
    r.g = lr.g.segment(6,6);
    l.g = lr.g.segment(0,6);
    lr.C = this->CoriolisMatrix(q_lr,qdot_lr);
    l.C = lr.C.block<6,6>(0,0);
    r.C = lr.C.block<6,6>(6,6);
    
}

// void IndyDualArm::inverseDynamicsNom(const Eigen::VectorXd &q_lr_nom,
//     const Eigen::VectorXd &qdot_lr_nom)
// {
// if (q_lr_nom.size() != DOF || qdot_lr_nom.size() != DOF)
// throw std::invalid_argument("q / qdot size ≠ 12");
// // Eigen::VectorXd q_lr_temp=q_lr;
// // q_lr_temp(0) +=3.141592;
// COPY_FROM(q_lr_nom.data(), ID_nom_U.q, DOF);
// COPY_FROM(qdot_lr_nom.data(), ID_nom_U.qdot, DOF);

// ID_nom_step();

// COPY_TO(this->nom_lr.M.data(), ID_nom_Y.M, DOF * DOF);

// COPY_TO(this->nom_lr.c.data(), ID_nom_Y.c, DOF);
// COPY_TO(this->nom_lr.g.data(), ID_nom_Y.g, DOF);

// this->nom_r.M = this->nom_lr.M.block<6,6>(6,6);
// this->nom_l.M = this->nom_lr.M.block<6,6>(0,0);
// this->nom_r.c = this->nom_lr.c.segment(6,6);
// this->nom_l.c = this->nom_lr.c.segment(0,6);
// this->nom_r.g = this->nom_lr.g.segment(6,6);
// this->nom_l.g = this->nom_lr.g.segment(0,6);

// }

/*───────── 4) Joint-Space Trajectory ─────────────────*/
 IndyDualArm::Des IndyDualArm::jointSpaceTrajectory(double t_sec,
                                       const Vector6d &q_start,
                                       const Vector6d &q_end,
                                       double T0,
                                       double Tf )
{
     IndyDualArm::Des des;
    JSTraj_U.gt = t_sec;
    COPY_FROM(q_start.data(), JSTraj_U.q_start, 6);
    COPY_FROM(q_end.data(), JSTraj_U.q_end, 6);
    JSTraj_U.T0 = T0;
    JSTraj_U.Tf = Tf;

    JSTraj_step();

    COPY_TO(des.q.data(), JSTraj_Y.q_des, 6);
    COPY_TO(des.qdot.data(), JSTraj_Y.qdot_des, 6);
    COPY_TO(des.qddot.data(), JSTraj_Y.qddot_des, 6);
    return des;

    
}

/*───────── 5) Task-Space Trajectory ──────────────────*/
IndyDualArm::Des IndyDualArm::taskSpaceTrajectory(double               t_sec,
    const Eigen::Matrix4d& T_start,
    const Eigen::Matrix4d& T_end,
    double               T0,
    double               Tf)
{
     IndyDualArm::Des des;
    COPY_FROM(T_start.data(), TSTraj_U.T_start, 16);
    COPY_FROM(T_end.data(), TSTraj_U.T_end, 16);
    TSTraj_U.gt = t_sec;
    TSTraj_U.T0 = T0;
    TSTraj_U.Tf = Tf;

    TSTraj_step();

    COPY_TO(des.T.data(), TSTraj_Y.T_t, 16);
    COPY_TO(des.V.data(), TSTraj_Y.V_t, 6);
    COPY_TO(des.Vdot.data(), TSTraj_Y.Vdot_t, 6);

    COPY_TO(des.p.data(), TSTraj_Y.p_t, 3);
    COPY_TO(des.pdot.data(), TSTraj_Y.pdot_t, 3);
    COPY_TO(des.pddot.data(), TSTraj_Y.pddot_t, 3);

    COPY_TO(des.xi.data(), TSTraj_Y.xi_t, 3);
    COPY_TO(des.xidot.data(), TSTraj_Y.xidot_t, 3);
    COPY_TO(des.xiddot.data(), TSTraj_Y.xiddot_t, 3);
    return des;
}


/* 전역 구조체는 codegen 헤더가 이미 선언
     extern ExtU_TaskSpaceController_T TaskSpaceController_U;
     extern ExtY_TaskSpaceController_T TaskSpaceController_Y; */

     Eigen::VectorXd IndyDualArm::taskSpaceController(
            const IndyDualArm::Arm l,
            const IndyDualArm::Arm r,
            const IndyDualArm::RelArm lr,
            const Eigen::VectorXd&               q,
            const Eigen::VectorXd&               qdot,
            const  IndyDualArm::Des &des_l,
            const  IndyDualArm::Des &des_r,
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
            const Eigen::VectorXd&               HinfK )
{
    Eigen::VectorXd                  tau_out;
    /* ─ 유효성 체크 (변경 없음) ─ */
    auto chk=[&](const char* n,const Eigen::VectorXd& v){
        if(v.size()!=DOF) throw std::invalid_argument(std::string(n)+" size ≠ 12");
    };
    chk("q",q); chk("qdot",qdot);
    chk("q_init" ,q_init );  chk("TaskKp",TaskKp);
    chk("TaskKv" ,TaskKv ); chk("HinfK"  ,HinfK );


    /* ▸ 전역 inport 구조체에 직접 복사 ---------------------------- */
    COPY_FROM(lr.M.data()       , TaskSpaceController_U.M        , DOF*DOF);
    COPY_FROM(lr.c.data()       , TaskSpaceController_U.c        , DOF);
    COPY_FROM(lr.g.data()       , TaskSpaceController_U.g        , DOF);



    COPY_FROM(q.data()       , TaskSpaceController_U.q        , DOF);
    COPY_FROM(qdot.data()    , TaskSpaceController_U.qdot     , DOF);

    COPY_FROM(l.T.data()     , TaskSpaceController_U.T_l      , 16);
    COPY_FROM(l.V.data()     , TaskSpaceController_U.V_l      ,  6);
    COPY_FROM(l.J.data()    , TaskSpaceController_U.Jb_l     , 36);
    COPY_FROM(l.Jdot.data() , TaskSpaceController_U.Jbdot_l  , 36);

    COPY_FROM(r.T.data()     , TaskSpaceController_U.T_r      , 16);
    COPY_FROM(r.V.data()     , TaskSpaceController_U.V_r      ,  6);
    COPY_FROM(r.J.data()    , TaskSpaceController_U.Jb_r     , 36);
    COPY_FROM(r.Jdot.data() , TaskSpaceController_U.Jbdot_r  , 36);

    COPY_FROM(des_l.T.data() , TaskSpaceController_U.T_des_l  , 16);
    COPY_FROM(des_l.V.data() , TaskSpaceController_U.V_des_l  ,  6);
    COPY_FROM(des_l.Vdot.data(),TaskSpaceController_U.Vdot_des_l,6);

    COPY_FROM(des_r.T.data() , TaskSpaceController_U.T_des_r  , 16);
    COPY_FROM(des_r.V.data() , TaskSpaceController_U.V_des_r  ,  6);
    COPY_FROM(des_r.Vdot.data(),TaskSpaceController_U.Vdot_des_r,6);

    TaskSpaceController_U.dt = dt;

    COPY_FROM(q_init.data()  , TaskSpaceController_U.q_init   , DOF);
    COPY_FROM(q_max.data()   , TaskSpaceController_U.q_max    , DOF);
    COPY_FROM(q_min.data()   , TaskSpaceController_U.q_min    , DOF);
    COPY_FROM(qdot_max.data(), TaskSpaceController_U.qdot_max , DOF);
    COPY_FROM(qdot_min.data(), TaskSpaceController_U.qdot_min , DOF);
    COPY_FROM(TaskKp.data()  , TaskSpaceController_U.TaskKp   , DOF);
    COPY_FROM(TaskKv.data()  , TaskSpaceController_U.TaskKv   , DOF);

    COPY_FROM(b0.data()      , TaskSpaceController_U.b0       , 2);
    COPY_FROM(a.data()       , TaskSpaceController_U.a        , 2);
    COPY_FROM(HinfK.data()   , TaskSpaceController_U.HinfK    , DOF);

    /* ▸ 1-step 실행 */
    TaskSpaceController_step();

    /* ▸ 출력 꺼내기 ----------------------------------------------- */
    tau_out.resize(DOF);
    COPY_TO(tau_out.data(), TaskSpaceController_Y.tau, DOF);
    return tau_out;
}


Eigen::MatrixXd IndyDualArm::MassMatrix(const Eigen::VectorXd &q)
{
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(q.size(),q.size());
    pinocchio::Data data(pimpl->model);
    Eigen::VectorXd v = pinocchio::randomConfiguration(pimpl->model);
    Eigen::VectorXd a = pinocchio::randomConfiguration(pimpl->model);
    Eigen::VectorXd q_ = q;
    q_(0) = q(0) +3.141592;
    pinocchio::crba(pimpl->model, data, q_);
    data.M.triangularView<Eigen::StrictlyLower>() =
        data.M.transpose().triangularView<Eigen::StrictlyLower>();
    M = data.M;
    return M;
}

Eigen::VectorXd IndyDualArm::GravityForces(const Eigen::VectorXd& q)
{
    Eigen::VectorXd grav = Eigen::VectorXd::Zero(q.size());
    pinocchio::Data data(pimpl->model);
    Eigen::VectorXd q_ = q;
    q_(0) = q(0) +3.141592;
    Eigen::VectorXd v = Eigen::VectorXd::Zero(pimpl->model.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(pimpl->model.nv);
    Eigen::VectorXd tau = pinocchio::rnea(pimpl->model, data, q_, v * 0, a * 0);
    pinocchio::computeGeneralizedGravity(pimpl->model, data, q_);
    grav = data.g;
    return grav;
}
Eigen::MatrixXd IndyDualArm::CoriolisMatrix(const Eigen::VectorXd& q,const Eigen::VectorXd& qdot)
{
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(q.size(),q.size());
    pinocchio::Data data(pimpl->model);

    Eigen::VectorXd v = qdot;
    Eigen::VectorXd a = Eigen::VectorXd::Zero(pimpl->model.nv);
    Eigen::VectorXd q_ = q;
    q_(0) = q(0) +3.141592;
    Eigen::VectorXd tau = pinocchio::rnea(pimpl->model, data, q_, v, a);
    pinocchio::computeCoriolisMatrix(pimpl->model, data, q_, v);
    C = data.C;
    return C;
}
Eigen::MatrixXd IndyDualArm::MassMatrixInverse(const Eigen::VectorXd& q)
{
    Eigen::MatrixXd Minv = Eigen::MatrixXd::Zero(q.size(),q.size());
    pinocchio::Data data(pimpl->model);
    Eigen::VectorXd q_ = q;
    q_(0) = q(0) +3.141592;
    pinocchio::computeMinverse(pimpl->model, data, q_);
    data.Minv.triangularView<Eigen::StrictlyLower>() = data.Minv.transpose().triangularView<Eigen::StrictlyLower>();
    Minv = data.Minv;
    return Minv;
}

Eigen::VectorXd IndyDualArm::HinfControl(const IndyDualArm::RelArm &lr,const Eigen::VectorXd q, const Eigen::VectorXd q_dot, const  IndyDualArm::RelDes &des, Eigen::VectorXd& e_int, double dt, const Eigen::VectorXd Hinf_K, const Eigen::VectorXd gamma)
{
    // gamma = invL2sqr
    Eigen::MatrixXd Hinf_Kp = Eigen::MatrixXd::Identity(q.size(),q.size()) * 100.0;
    Eigen::MatrixXd Hinf_Kv = Eigen::MatrixXd::Identity(q.size(),q.size())* 20.0;
    Eigen::MatrixXd Hinf_K_gamma = Eigen::MatrixXd::Identity(q.size(),q.size());
    for (int i = 0; i < q.size(); i++)
    {
        Hinf_K_gamma(i, i) = Hinf_K(i) + 1.0 / gamma(i);
    }
    Eigen::VectorXd q_des = des.q;
    Eigen::VectorXd q_dot_des = des.qdot;
    Eigen::VectorXd q_ddot_des = des.qddot;
    
    Eigen::VectorXd  e = q_des - q;
    Eigen::VectorXd  e_dot = q_dot_des - q_dot;
    e_int += e*dt;
    Eigen::MatrixXd Mmat = lr.M;
    Eigen::MatrixXd C = lr.C;
    Eigen::VectorXd G = lr.g;
    Eigen::VectorXd q_ddot_ref = q_ddot_des + Hinf_Kv * e_dot + Hinf_Kp * e;
    Eigen::VectorXd q_dot_ref = q_dot_des + Hinf_Kv *e + Hinf_Kp * e_int;
    Eigen::VectorXd torq = Mmat * q_ddot_ref + C * q_dot_ref + G + (Hinf_K_gamma) * (e_dot + Hinf_Kv * e + Hinf_Kp * e_int);
    return torq;
}

Eigen::VectorXd IndyDualArm::HinfControl(const IndyDualArm::Arm &arm, const Eigen::VectorXd q, const Eigen::VectorXd q_dot, const  IndyDualArm::Des &des, Eigen::VectorXd& e_int, double dt, const Eigen::VectorXd Hinf_K, const Eigen::VectorXd gamma)
{
    // gamma = invL2sqr
    Eigen::MatrixXd Hinf_Kp = Eigen::MatrixXd::Identity(q.size(),q.size()) * 100.0;
    Eigen::MatrixXd Hinf_Kv = Eigen::MatrixXd::Identity(q.size(),q.size())* 20.0;
    Eigen::MatrixXd Hinf_K_gamma = Eigen::MatrixXd::Identity(q.size(),q.size());
    for (int i = 0; i < q.size(); i++)
    {
        Hinf_K_gamma(i, i) = Hinf_K(i) + 1.0 / gamma(i);
    }
    Eigen::VectorXd q_des = des.q;
    Eigen::VectorXd q_dot_des = des.qdot;
    Eigen::VectorXd q_ddot_des = des.qddot;
    
    Eigen::VectorXd  e = q_des - q;
    Eigen::VectorXd  e_dot = q_dot_des - q_dot;
    e_int += e*dt;
    Eigen::MatrixXd Mmat = arm.M;
    Eigen::MatrixXd C = arm.C;
    Eigen::VectorXd G = arm.g;
    Eigen::VectorXd q_ddot_ref = q_ddot_des + Hinf_Kv * e_dot + Hinf_Kp * e;
    Eigen::VectorXd q_dot_ref = q_dot_des + Hinf_Kv *e + Hinf_Kp * e_int;
    Eigen::VectorXd torq = Mmat * q_ddot_ref + C * q_dot_ref + G + (Hinf_K_gamma) * (e_dot + Hinf_Kv * e + Hinf_Kp * e_int);
    return torq;
}
// VectorXd LR_Control::GravityForces(JVec q_)
// {
//     JVec G = JVec::Zero();
//     pinocchio::Data data(model);
//     Eigen::VectorXd v = pinocchio::randomConfiguration(model);
//     Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
//     Eigen::VectorXd q = pinocchio::randomConfiguration(model);
//     q = q_;
//     Eigen::VectorXd tau = pinocchio::rnea(model, data, q, v * 0, a * 0);
//     pinocchio::computeGeneralizedGravity(model, data, q);
//     G = data.g;
//     return G;
// }

// Eigen::Matrix<double,12,12>  LR_Control::CoriolisMatrix(JVec q_, JVec q_dot)
// {
//     MatrixNd C = MatrixNd::Zero();
//     pinocchio::Data data(model);

//     Eigen::VectorXd v = pinocchio::randomConfiguration(model);
//     Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
//     Eigen::VectorXd q = pinocchio::randomConfiguration(model);
//     q = q_;
//     v = q_dot;
//     Eigen::VectorXd tau = pinocchio::rnea(model, data, q, v, a);
//     pinocchio::computeCoriolisMatrix(model, data, q, v);
//     C = data.C;
//     return C;
// }
// VectorXd LR_Control::ForwardDynamics(JVec q, JVec q_dot, JVec tau)
// {
//     pinocchio::Data data(model);
//     JVec q_ddot = JVec::Zero();
//     pinocchio::integrate(model, q, q_dot, tau);
//     pinocchio::aba(model, data, q, q_dot, tau);
//     q_ddot = data.ddq;
//     return q_ddot;
// }
