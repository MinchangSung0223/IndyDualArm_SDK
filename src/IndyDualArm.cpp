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
#include <ruckig/ruckig.hpp>
using namespace ruckig;
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
#include "LR.h"
}

#define COPY_TO(dst, src, n) std::memcpy((dst), (src), (n) * sizeof(double))
#define COPY_FROM(src, dst, n) std::memcpy((dst), (src), (n) * sizeof(double))
struct IndyDualArm::Impl {
    pinocchio::Model model;
    pinocchio::Data  data;
        double dt;

    Ruckig<6> otg;  // control cycle
    InputParameter<6> input;
    OutputParameter<6> output;
   Impl() 
        : dt(0.001), otg(dt) // ✅ 초기화 리스트 사용
    {
        // 추가 초기화 코드가 있으면 여기에
    }
};
void IndyDualArm::initModules_()
{
    FD_initialize();
    FD_nom_initialize();
    LR_initialize();
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
//         lambda_l<< 0.00731290617519740	,-0.112209643268792,	-0.448444666974966,	-0.234147674440613,	-0.00655333111792072,	-0.0910987264418007,	0.000734266081960586,
// -0.468471605135988,	0.0253049384494602,	-0.00213555056392088,	0.178112574046688,	-0.0444084686750942,	0.0278335968117789,	0.00100927975706183,
// 0.584550039568942,	0.245985088392615,	-0.0294093003973933,	0.140807962066561,	0.144840166190116,	0.173596536662667,	0.0589714283937366,
// -2.08786077845327,	1.21135113727416,	0.00637692695353308,	-1.20451993849272,	1.21271647308494,	-1.20010955429783,	0.00708035443410816,
// -0.000419897553542710,	1.21383312381251,	-0.000125029976757511,	-1.21233388351448,	1.20890045805052,	-1.21075155284052,	-0.00745878481106931,
// 0.0169832813592034,	-1.20010117384431,	0.00391070634428324	,1.20682064118058,	-1.21527435018294,	1.19820866588080,	-0.0155294916452316;
        
        lambda_l<< 0,-0.1119,-0.4500,-0.2321,-0.0072,-0.0911, 0.0000,
-0.4695, 0.0247,0, 0.1815,-0.0447, 0.0286,-0.0000,
 0.5827, 0.2437,-0.0305, 0.1414, 0.1451, 0.1745, 0.0600,
-2.0910, 1.2092,0,-1.2092, 1.2092,-1.2092, 0.0000,
0, 1.2092,0,-1.2092, 1.2092,-1.2092,-0.0000,
0,-1.2092,0, 1.2092,-1.2092, 1.2092,0;

        lambda_r<< 0,-0.1119,-0.4500,-0.2321,-0.0072,-0.0911, 0.0000,
0.4695, 0.0247,0, 0.1815,-0.0447, 0.0286,-0.0000,
0.5827, 0.2437,-0.0305, 0.1414, 0.1451, 0.1745, 0.0600,
2.0910, 1.2092,0,-1.2092, 1.2092,-1.2092,-0.0000,
0, 1.2092,0,-1.2092, 1.2092,-1.2092,-0.0000,
0,-1.2092, 0.0000, 1.2092,-1.2092, 1.2092,0;
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
	Eigen::Matrix4d TransInv(const Eigen::Matrix4d& T) {
		Eigen::Matrix4d ret=Eigen::Matrix4d::Identity();
		ret<<T(0,0), T(1,0), T(2,0), - T(0,0)*T(0,3) - T(1,0)*T(1,3) - T(2,0)*T(2,3),
		 	 T(0,1), T(1,1), T(2,1), - T(0,1)*T(0,3) - T(1,1)*T(1,3) - T(2,1)*T(2,3), 
			 T(0,2), T(1,2), T(2,2), - T(0,2)*T(0,3) - T(1,2)*T(1,3) - T(2,2)*T(2,3),
			 0,0,0,1;
		return ret;
	}	
Eigen::MatrixXd Ad(const Eigen::Matrix4d& T) {
    Eigen::MatrixXd ad_ret = Eigen::MatrixXd::Zero(6,6);
    ad_ret<<T(0,0),T(0,1),T(0,2),T(1,3)*T(2,0) - T(2,3)*T(1,0), T(1,3)*T(2,1) - T(2,3)*T(1,1), T(1,3)*T(2,2) - T(2,3)*T(1,2),
            T(1,0),T(1,1),T(1,2),T(2,3)*T(0,0) - T(0,3)*T(2,0), T(2,3)*T(0,1) - T(0,3)*T(2,1), T(2,3)*T(0,2) - T(0,3)*T(2,2),
            T(2,0),T(2,1),T(2,2),T(0,3)*T(1,0) - T(1,3)*T(0,0), T(0,3)*T(1,1) - T(1,3)*T(0,1), T(0,3)*T(1,2) - T(1,3)*T(0,2),
            0,0,0,T(0,0),T(0,1),T(0,2),
            0,0,0,T(1,0),T(1,1),T(1,2),
            0,0,0,T(2,0),T(2,1),T(2,2);
    return ad_ret;
}
void IndyDualArm::LieScrewScurveTrajectory(const Eigen::Matrix4d X0,const Eigen::Matrix4d XT,const Vector6d V0,const Vector6d VT,const Vector6d dV0,const Vector6d dVT,Vector6d dlambda_max, Vector6d ddlambda_max,Vector6d dddlambda_max, double dt,std::vector<Eigen::Matrix4d>& T_des_list,std::vector<Vector6d>& V_des_list,std::vector<Vector6d>& V_des_dot_list,double& max_tt){
	Vector6d lambda_0,lambda_T,dlambda_0,dlambda_T,ddlambda_0,ddlambda_T,lambda_t,dlambda_t,ddlambda_t;
	lambda_0 = Vector6d::Zero();
	lambda_T = this->log6(TransInv(X0)*XT);
    Eigen::MatrixXd Admat = Ad(TransInv(X0));

	dlambda_0 = Admat*this->dexp6inv(-lambda_0)*V0;
	dlambda_T = Admat*this->dexp6inv(-lambda_T)*VT;
	ddlambda_0 = Admat*(this->dexp6inv(-lambda_0)*dV0+this->ddexp6inv(-lambda_0,-dlambda_0)*V0);
	ddlambda_T = Admat*(this->dexp6inv(-lambda_T)*dVT+this->ddexp6inv(-lambda_T,-dlambda_T)*VT);
	lambda_t=dlambda_t=ddlambda_t= Vector6d::Zero();


    Ruckig<6> otg(dt);  // control cycle
    InputParameter<6> input;
    OutputParameter<6> output;
    input.current_position = {lambda_0(0),lambda_0(1),lambda_0(2),lambda_0(3),lambda_0(4),lambda_0(5)};
    input.current_velocity  = {dlambda_0(0),dlambda_0(1),dlambda_0(2),dlambda_0(3),dlambda_0(4),dlambda_0(5)};
    input.current_acceleration  = {ddlambda_0(0),ddlambda_0(1),ddlambda_0(2),ddlambda_0(3),ddlambda_0(4),ddlambda_0(5)};

    input.target_position = {lambda_T(0),lambda_T(1),lambda_T(2),lambda_T(3),lambda_T(4),lambda_T(5)};
    input.target_velocity  = {dlambda_T(0),dlambda_T(1),dlambda_T(2),dlambda_T(3),dlambda_T(4),dlambda_T(5)};
    input.target_acceleration = {ddlambda_T(0),ddlambda_T(1),ddlambda_T(2),ddlambda_T(3),ddlambda_T(4),ddlambda_T(5)};

    input.max_velocity  = {dlambda_max(0),dlambda_max(1),dlambda_max(2),dlambda_max(3),dlambda_max(4),dlambda_max(5)};
    input.max_acceleration  = {ddlambda_max(0),ddlambda_max(1),ddlambda_max(2),ddlambda_max(3),ddlambda_max(4),ddlambda_max(5)};
    input.max_jerk  = {dddlambda_max(0),dddlambda_max(1),dddlambda_max(2),dddlambda_max(3),dddlambda_max(4),dddlambda_max(5)};
    
    while (otg.update(input, output) == Result::Working) {
        // std::cout << output.time << " | " << join(output.new_position) << std::endl;
         lambda_t <<output.new_position[0],output.new_position[1],output.new_position[2],output.new_position[3],output.new_position[4],output.new_position[5];
         dlambda_t<<output.new_velocity[0],output.new_velocity[1],output.new_velocity[2],output.new_velocity[3],output.new_velocity[4],output.new_velocity[5];
         ddlambda_t<<output.new_acceleration[0],output.new_acceleration[1],output.new_acceleration[2],output.new_acceleration[3],output.new_acceleration[4],output.new_acceleration[5];
	    Vector6d V_des = this->dexp6(-lambda_t)*dlambda_t;
	    Vector6d  V_des_dot =  this->dexp6(-lambda_t)*ddlambda_t+ this->ddexp6(-lambda_t,-dlambda_t)*dlambda_t;
	    Eigen::Matrix4d T_des = X0*exp6(lambda_t);         
         T_des_list.push_back(T_des);
         V_des_list.push_back(Ad(X0)*V_des);
         V_des_dot_list.push_back(Ad(X0)*V_des_dot);
         max_tt = output.time;
         output.pass_to_input(input);
    }
    

}	

void IndyDualArm::LieScrewScurveTrajectory(const Eigen::Matrix4d X0,
    const Eigen::Matrix4d XT,
    const Vector6d V0,
    const Vector6d VT,
    const Vector6d dV0,
    const Vector6d dVT,
    double p_max,
    double pdot_max,
    double pddot_max, 
    double w_max, 
    double wdot_max,
    double wddot_max,  
    double jerk_eta_max,
    double jerk_xi_max,
    double dt,
    std::vector<Eigen::Matrix4d>& T_des_list,
    std::vector<Vector6d>& V_des_list,
    std::vector<Vector6d>& V_des_dot_list,
    double& max_tt){
	Vector6d lambda_0,lambda_T,dlambda_0,dlambda_T,ddlambda_0,ddlambda_T,lambda_t,dlambda_t,ddlambda_t;
	lambda_0 = Vector6d::Zero();
	lambda_T = this->log6(TransInv(X0)*XT);

    Eigen::MatrixXd Admat = Ad(TransInv(X0));
    double norm_p_T = (this->dexp3(lambda_T.segment(3,3))*lambda_T.segment(0,3)).norm();
    if (norm_p_T>p_max) p_max = norm_p_T;
    double xidot_max  = w_max;
    double xiddot_max  = (wdot_max);
    double xidddot_max  = (wddot_max);
    double etadot_max = pdot_max;
    double etaddot_max = pddot_max;
     


	dlambda_0 = Admat*this->dexp6inv(-lambda_0)*V0;
	dlambda_T = Admat*this->dexp6inv(-lambda_T)*VT;
	ddlambda_0 = Admat*(this->dexp6inv(-lambda_0)*dV0+this->ddexp6inv(-lambda_0,-dlambda_0)*V0);
	ddlambda_T = Admat*(this->dexp6inv(-lambda_T)*dVT+this->ddexp6inv(-lambda_T,-dlambda_T)*VT);
	lambda_t=dlambda_t=ddlambda_t= Vector6d::Zero();

    

    Ruckig<6> otg(dt);  // control cycle
    InputParameter<6> input;
    OutputParameter<6> output;
    input.current_position = {lambda_0(0),lambda_0(1),lambda_0(2),lambda_0(3),lambda_0(4),lambda_0(5)};
    input.current_velocity  = {dlambda_0(0),dlambda_0(1),dlambda_0(2),dlambda_0(3),dlambda_0(4),dlambda_0(5)};
    input.current_acceleration  = {ddlambda_0(0),ddlambda_0(1),ddlambda_0(2),ddlambda_0(3),ddlambda_0(4),ddlambda_0(5)};

    input.target_position = {lambda_T(0),lambda_T(1),lambda_T(2),lambda_T(3),lambda_T(4),lambda_T(5)};
    input.target_velocity  = {dlambda_T(0),dlambda_T(1),dlambda_T(2),dlambda_T(3),dlambda_T(4),dlambda_T(5)};
    input.target_acceleration = {ddlambda_T(0),ddlambda_T(1),ddlambda_T(2),ddlambda_T(3),ddlambda_T(4),ddlambda_T(5)};

    input.max_velocity  = {etadot_max/sqrt(3),etadot_max/sqrt(3),etadot_max/sqrt(3),xidot_max/sqrt(3),xidot_max/sqrt(3),xidot_max/sqrt(3)};
    input.max_acceleration  = {etaddot_max/sqrt(3),etaddot_max/sqrt(3),etaddot_max/sqrt(3),xiddot_max/sqrt(3),xiddot_max/sqrt(3),xiddot_max/sqrt(3)};
    input.max_jerk  = {jerk_eta_max/sqrt(3),jerk_eta_max/sqrt(3),jerk_eta_max/sqrt(3),jerk_xi_max/sqrt(3),jerk_xi_max/sqrt(3),jerk_xi_max/sqrt(3)};
    
    while (otg.update(input, output) == Result::Working) {
        // std::cout << output.time << " | " << join(output.new_position) << std::endl;
         lambda_t <<output.new_position[0],output.new_position[1],output.new_position[2],output.new_position[3],output.new_position[4],output.new_position[5];
         dlambda_t<<output.new_velocity[0],output.new_velocity[1],output.new_velocity[2],output.new_velocity[3],output.new_velocity[4],output.new_velocity[5];
         ddlambda_t<<output.new_acceleration[0],output.new_acceleration[1],output.new_acceleration[2],output.new_acceleration[3],output.new_acceleration[4],output.new_acceleration[5];
	    Vector6d V_des = this->dexp6(-lambda_t)*dlambda_t;
	    Vector6d  V_des_dot =  this->dexp6(-lambda_t)*ddlambda_t+ this->ddexp6(-lambda_t,-dlambda_t)*dlambda_t;
	    Eigen::Matrix4d T_des = X0*exp6(lambda_t);         
         T_des_list.push_back(T_des);
         V_des_list.push_back(Ad(X0)*V_des);
         V_des_dot_list.push_back(Ad(X0)*V_des_dot);
         max_tt = output.time;
         output.pass_to_input(input);
    }
    

}	


void IndyDualArm::SetLieTrajectory(
    const Eigen::Matrix4d& X_start,
    const Eigen::Matrix4d& X_end,
    const Vector6d& V_start,
    const Vector6d& V_end,
    const Vector6d& Vdot_start,
    const Vector6d& Vdot_end,
    // const Vector6d& Vddot_start,
    // const Vector6d& Vddot_end,  
    const Vector6d& V_max,
    const Vector6d& Vdot_max,
    const Vector6d& Vddot_max,
    double dt)
{
     Vector6d lambda0 = Vector6d::Zero();
     Vector6d lambdaT = this->log6(TransInv(X_start)*X_end);
     Eigen::MatrixXd Admat = Ad(TransInv(X_start));
Vector6d lambda_dot0 =Admat*this->dexp6inv(-lambda0)*V_start;
Vector6d lambda_ddot0 =Admat*(this->dexp6inv(-lambda0)*Vdot_start+this->ddexp6inv(-lambda0,-lambda_dot0)*V_start);
// Vector6d lambda_dddot0= Admat*(this->ddexp6inv(-lambda0,-lambda_dot0)*V_start+this->dexp6inv(-lambda0)*Vddot_start+this->dddexp6inv(-lambda0,-lambda_dot0,-lambda_ddot0)*Vdot_start+this->ddexp6inv(-lambda0,-lambda_dot0)*Vddot_start);
Vector6d lambda_dotT = Admat*this->dexp6inv(-lambdaT)*V_end;
Vector6d lambda_ddotT =Admat*(this->dexp6inv(-lambdaT)*Vdot_end+this->ddexp6inv(-lambdaT,-lambda_dotT)*V_end);
// Vector6d lambda_dddotT=Admat*(this->dexp6inv(-lambdaT)*Vddot_end+2*this->ddexp6inv(-lambdaT,-lambda_dotT)*Vdot_end+this->dddexp6inv(-lambdaT,-lambda_dotT,-lambda_ddotT)*V_end);
    // pimpl->otg = Ruckig(dt,6);   

    Vector6d dlambda_max,ddlambda_max,dddlambda_max;
    dlambda_max = V_max;
    ddlambda_max=Vdot_max;
    dddlambda_max=Vddot_max;
    
    this->X_start =X_start;
  
    pimpl->input.current_position = {lambda0(0),lambda0(1),lambda0(2),lambda0(3),lambda0(4),lambda0(5)};
    pimpl->input.current_velocity  = {lambda_dot0(0),lambda_dot0(1),lambda_dot0(2),lambda_dot0(3),lambda_dot0(4),lambda_dot0(5)};
    pimpl->input.current_acceleration  = {lambda_ddot0(0),lambda_ddot0(1),lambda_ddot0(2),lambda_ddot0(3),lambda_ddot0(4),lambda_ddot0(5)};

    pimpl->input.target_position = {lambdaT(0),lambdaT(1),lambdaT(2),lambdaT(3),lambdaT(4),lambdaT(5)};
    pimpl->input.target_velocity  = {lambda_dotT(0),lambda_dotT(1),lambda_dotT(2),lambda_dotT(3),lambda_dotT(4),lambda_dotT(5)};
    pimpl->input.target_acceleration = {lambda_ddotT(0),lambda_ddotT(1),lambda_ddotT(2),lambda_ddotT(3),lambda_ddotT(4),lambda_ddotT(5)};

    pimpl->input.max_velocity  = {dlambda_max(0),dlambda_max(1),dlambda_max(2),dlambda_max(3),dlambda_max(4),dlambda_max(5)};
    pimpl->input.max_acceleration  = {ddlambda_max(0),ddlambda_max(1),ddlambda_max(2),ddlambda_max(3),ddlambda_max(4),ddlambda_max(5)};
    pimpl->input.max_jerk  = {dddlambda_max(0),dddlambda_max(1),dddlambda_max(2),dddlambda_max(3),dddlambda_max(4),dddlambda_max(5)};
}

IndyDualArm::Des IndyDualArm::GetLieTrajectory(double time)
{
    IndyDualArm::Des des;
    Vector6d lambda_t,lambda_dot_t,lambda_ddot_t;
    lambda_t = Vector6d::Zero();
    lambda_dot_t = Vector6d::Zero();
    lambda_ddot_t = Vector6d::Zero();
    
    while(pimpl->otg.update(pimpl->input, pimpl->output) == Result::Working ){
         lambda_t <<pimpl->output.new_position[0],pimpl->output.new_position[1],pimpl->output.new_position[2],pimpl->output.new_position[3],pimpl->output.new_position[4],pimpl->output.new_position[5];
         lambda_dot_t<<pimpl->output.new_velocity[0],pimpl->output.new_velocity[1],pimpl->output.new_velocity[2],pimpl->output.new_velocity[3],pimpl->output.new_velocity[4],pimpl->output.new_velocity[5];
         lambda_ddot_t<<pimpl->output.new_acceleration[0],pimpl->output.new_acceleration[1],pimpl->output.new_acceleration[2],pimpl->output.new_acceleration[3],pimpl->output.new_acceleration[4],pimpl->output.new_acceleration[5];
    }
    des.T =  this->X_start*this->exp6(lambda_t);
    des.V = Ad(X_start)*(this->dexp6(-lambda_t)*lambda_dot_t);
    des.Vdot = Ad(X_start)*(this->dexp6(-lambda_t)*lambda_ddot_t+ this->ddexp6(-lambda_t,-lambda_dot_t)*lambda_dot_t);
    // des.Vddot = this->dexp6(-lambda_t)*lambda_dddot_t+2*this->ddexp6(-lambda_t,-lambda_dot_t)*lambda_ddot_t+this->dddexp6(-lambda_t,-lambda_dot_t,-lambda_ddot_t)*lambda_dot_t;

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

Eigen::Matrix4d IndyDualArm::exp6(const Vector6d &lambda){
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    LR_U.exp6_lambda[0] = lambda(0);
    LR_U.exp6_lambda[1] = lambda(1);
    LR_U.exp6_lambda[2] = lambda(2);
    LR_U.exp6_lambda[3] = lambda(3);
    LR_U.exp6_lambda[4] = lambda(4);
    LR_U.exp6_lambda[5] = lambda(5);
    
    LR_step();
    int count = 0;
    for(int j =0;j<4;j++)
        for(int i =0;i<4;i++)
            T(i,j)=LR_Y.exp6_T[count++];
    return T;
}
Vector6d IndyDualArm::log6(const Eigen::Matrix4d &T){
        Vector6d lambda;
        lambda = Vector6d::Zero();
   int count = 0;
    for(int j =0;j<4;j++)
        for(int i =0;i<4;i++)
            LR_U.log6_T[count++]=T(i,j);
    
    LR_step();
    for(int i= 0;i<6;i++)
        lambda(i)=LR_Y.lambda[i].re;
    return lambda;
}

Eigen::MatrixXd IndyDualArm::dexp6(const Vector6d &lambda){
    Eigen::MatrixXd ret = Eigen::MatrixXd::Identity(6,6);
    LR_U.dexp6_lambda[0] = lambda(0);
    LR_U.dexp6_lambda[1] = lambda(1);
    LR_U.dexp6_lambda[2] = lambda(2);
    LR_U.dexp6_lambda[3] = lambda(3);
    LR_U.dexp6_lambda[4] = lambda(4);
    LR_U.dexp6_lambda[5] = lambda(5);
    
    LR_step();
    int count = 0;
    for(int j =0;j<6;j++)
        for(int i =0;i<6;i++)
            ret(i,j)=LR_Y.dexp6[count++];
    return ret;
}

Eigen::MatrixXd IndyDualArm::ddexp6(const Vector6d &lambda,const Vector6d &lambdadot){
    Eigen::MatrixXd ret = Eigen::MatrixXd::Identity(6,6);
    for(int i=0;i<6;i++){
         LR_U.ddexp6_lambda[i] = lambda(i);
         LR_U.ddexp6_lambdadot[i] = lambdadot(i);
    }
    LR_step();
    int count = 0;
    for(int j =0;j<6;j++)
        for(int i =0;i<6;i++)
            ret(i,j)=LR_Y.ddexp6[count++];
    return ret;
}

Eigen::MatrixXd IndyDualArm::dddexp6(const Vector6d &lambda,const Vector6d &lambdadot,const Vector6d &lambdaddot){
    Eigen::MatrixXd ret = Eigen::MatrixXd::Identity(6,6);
    for(int i=0;i<6;i++){
         LR_U.dddexp6_lambda[i] = lambda(i);
         LR_U.dddexp6_lambdadot[i] = lambdadot(i);
        LR_U.dddexp6_lambdadot[i] = lambdaddot(i);
    }
    LR_step();
    int count = 0;
    for(int j =0;j<6;j++)
        for(int i =0;i<6;i++)
            ret(i,j)=LR_Y.dddexp6[count++];
    return ret;
}


Eigen::MatrixXd IndyDualArm::dexp3(const Eigen::Vector3d &xi){
    Eigen::MatrixXd ret = Eigen::MatrixXd::Identity(3,3);
    for(int i=0;i<3;i++){
         LR_U.dexp3_xi[i] = xi(i);
    }
    LR_step();
    int count = 0;
    for(int j =0;j<3;j++)
        for(int i =0;i<3;i++)
            ret(i,j)=LR_Y.dexp3[count++];
    return ret;
}

Eigen::MatrixXd IndyDualArm::ddexp3(const Eigen::Vector3d &xi,const Eigen::Vector3d &xidot){
    Eigen::MatrixXd ret = Eigen::MatrixXd::Identity(3,3);
    for(int i=0;i<3;i++){
         LR_U.ddexp3_xi[i] = xi(i);
        LR_U.ddexp3_xidot[i] = xidot(i);

    }
    LR_step();
    int count = 0;
    for(int j =0;j<3;j++)
        for(int i =0;i<3;i++)
            ret(i,j)=LR_Y.ddexp3[count++];
    return ret;
}

Eigen::MatrixXd IndyDualArm::dddexp3(const Eigen::Vector3d &xi,const Eigen::Vector3d &xidot,const Eigen::Vector3d &xiddot){
    Eigen::MatrixXd ret = Eigen::MatrixXd::Identity(3,3);
    for(int i=0;i<3;i++){
         LR_U.dddexp3_xi[i] = xi(i);
        LR_U.dddexp3_xidot[i] = xidot(i);
        LR_U.dddexp3_xidot[i] = xiddot(i);

    }
    LR_step();
    int count = 0;
    for(int j =0;j<3;j++)
        for(int i =0;i<3;i++)
            ret(i,j)=LR_Y.dddexp3[count++];
    return ret;
}


Eigen::MatrixXd IndyDualArm::dexp6inv(const Vector6d &lambda){
    Eigen::MatrixXd ret = Eigen::MatrixXd::Identity(6,6);
    LR_U.dexp6inv_lambda[0] = lambda(0);
    LR_U.dexp6inv_lambda[1] = lambda(1);
    LR_U.dexp6inv_lambda[2] = lambda(2);
    LR_U.dexp6inv_lambda[3] = lambda(3);
    LR_U.dexp6inv_lambda[4] = lambda(4);
    LR_U.dexp6inv_lambda[5] = lambda(5);
    
    LR_step();
    int count = 0;
    for(int j =0;j<6;j++)
        for(int i =0;i<6;i++)
            ret(i,j)=LR_Y.dexp6inv[count++];
    return ret;
}

Eigen::MatrixXd IndyDualArm::ddexp6inv(const Vector6d &lambda,const Vector6d &lambdadot){
    Eigen::MatrixXd ret = Eigen::MatrixXd::Identity(6,6);
    for(int i=0;i<6;i++){
         LR_U.ddexp6inv_lambda[i] = lambda(i);
         LR_U.ddexp6inv_lambdadot[i] = lambdadot(i);
    }
    LR_step();
    int count = 0;
    for(int j =0;j<6;j++)
        for(int i =0;i<6;i++)
            ret(i,j)=LR_Y.ddexp6inv[count++];
    return ret;
}

Eigen::MatrixXd IndyDualArm::dddexp6inv(const Vector6d &lambda,const Vector6d &lambdadot,const Vector6d &lambdaddot){
    Eigen::MatrixXd ret = Eigen::MatrixXd::Identity(6,6);
    for(int i=0;i<6;i++){
         LR_U.dddexp6inv_lambda[i] = lambda(i);
         LR_U.dddexp6inv_lambdadot[i] = lambdadot(i);
        LR_U.dddexp6inv_lambdadot[i] = lambdaddot(i);
    }
    LR_step();
    int count = 0;
    for(int j =0;j<6;j++)
        for(int i =0;i<6;i++)
            ret(i,j)=LR_Y.dddexp6inv[count++];
    return ret;
}


Eigen::MatrixXd IndyDualArm::dexp3inv(const Eigen::Vector3d &xi){
    Eigen::MatrixXd ret = Eigen::MatrixXd::Identity(3,3);
    for(int i=0;i<3;i++){
         LR_U.dexp3inv_xi[i] = xi(i);
    }
    LR_step();
    int count = 0;
    for(int j =0;j<3;j++)
        for(int i =0;i<3;i++)
            ret(i,j)=LR_Y.dexp3inv[count++];
    return ret;
}

Eigen::MatrixXd IndyDualArm::ddexp3inv(const Eigen::Vector3d &xi,const Eigen::Vector3d &xidot){
    Eigen::MatrixXd ret = Eigen::MatrixXd::Identity(3,3);
    for(int i=0;i<3;i++){
         LR_U.ddexp3inv_xi[i] = xi(i);
        LR_U.ddexp3inv_xidot[i] = xidot(i);

    }
    LR_step();
    int count = 0;
    for(int j =0;j<3;j++)
        for(int i =0;i<3;i++)
            ret(i,j)=LR_Y.ddexp3inv[count++];
    return ret;
}

Eigen::MatrixXd IndyDualArm::dddexp3inv(const Eigen::Vector3d &xi,const Eigen::Vector3d &xidot,const Eigen::Vector3d &xiddot){
    Eigen::MatrixXd ret = Eigen::MatrixXd::Identity(3,3);
    for(int i=0;i<3;i++){
         LR_U.dddexp3inv_xi[i] = xi(i);
        LR_U.dddexp3inv_xidot[i] = xidot(i);
        LR_U.dddexp3inv_xidot[i] = xiddot(i);

    }
    LR_step();
    int count = 0;
    for(int j =0;j<3;j++)
        for(int i =0;i<3;i++)
            ret(i,j)=LR_Y.dddexp3inv[count++];
    return ret;
}


// typedef struct {
//   real_T log6_T[16];                   /* '<Root>/log6_T' */
//   real_T exp6_lambda[6];               /* '<Root>/exp6_lambda' */
//   real_T dexp6_lambda[6];              /* '<Root>/dexp6_lambda' */
//   real_T ddexp6_lambda[6];             /* '<Root>/ddexp6_lambda' */
//   real_T ddexp6_lambdadot[6];          /* '<Root>/ddexp6_lambdadot' */
//   real_T dddexp6_lambda[6];            /* '<Root>/dddexp6_lambda' */
//   real_T dddexp6_lambdadot[6];         /* '<Root>/dddexp6_lambdadot' */
//   real_T dddexp6_lambdaddot[6];        /* '<Root>/dddexp6_lambdaddot' */
//   real_T dexp3_xi[3];                  /* '<Root>/dexp3_xi' */
//   real_T ddexp3_xi[3];                 /* '<Root>/ddexp3_xi' */
//   real_T ddexp3_xidot[3];              /* '<Root>/ddexp3_xidot' */
//   real_T dddexp3_xi[3];                /* '<Root>/dddexp3_xi' */
//   real_T dddexp3_xidot[3];             /* '<Root>/dddexp3_xidot' */
//   real_T dddexp3_xiddot[3];            /* '<Root>/dddexp3_xiddot' */
//   real_T dexp3inv_xi[6];               /* '<Root>/dexp3inv_xi' */
//   real_T ddexp3inv_xi[3];              /* '<Root>/ddexp3inv_xi' */
//   real_T ddexp3inv_xidot[3];           /* '<Root>/ddexp3inv_xidot' */
//   real_T dddexp3inv_xi[3];             /* '<Root>/dddexp3inv_xi' */
//   real_T dddexp3inv_xidot[3];          /* '<Root>/dddexp3inv_xidot' */
//   real_T dddexp3inv_xiddot[3];         /* '<Root>/dddexp3inv_xiddot' */
//   real_T dexp6inv_lambda[6];           /* '<Root>/dexp6inv_lambda' */
//   real_T ddexp6inv_lambda[6];          /* '<Root>/ddexp6inv_lambda' */
//   real_T ddexp6inv_lambdadot[6];       /* '<Root>/ddexp6inv_lambdadot' */
//   real_T dddexp6inv_lambda[6];         /* '<Root>/dddexp6inv_lambda' */
//   real_T dddexp6inv_lambdadot[6];      /* '<Root>/dddexp6inv_lambdadot' */
//   real_T dddexp6inv_lambdaddot[6];     /* '<Root>/dddexp6inv_lambdaddot' */
// } ExtU_LR_T;

// /* External outputs (root outports fed by signals with default storage) */
// typedef struct {
//   real_T log6_lambda[6];              /* '<Root>/log6_lambda' */
//   real_T exp6_T[16];                   /* '<Root>/exp6_T' */
//   real_T dexp6[36];                    /* '<Root>/dexp6' */
//   real_T ddexp6[36];                   /* '<Root>/ddexp6' */
//   real_T dddexp6[36];                  /* '<Root>/dddexp6' */
//   real_T dexp3[9];                     /* '<Root>/dexp3' */
//   real_T ddexp3[9];                    /* '<Root>/ddexp3' */
//   real_T dddexp3[9];                   /* '<Root>/dddexp3' */
//   real_T dexp3inv[9];                  /* '<Root>/dexp3inv' */
//   real_T ddexp3inv[9];                 /* '<Root>/ddexp3inv' */
//   real_T dddexp3inv[9];                /* '<Root>/dddexp3inv' */
//   real_T dexp6inv[36];                 /* '<Root>/dexp6inv' */
//   real_T ddexp6inv[36];                /* '<Root>/ddexp6inv' */
//   real_T dddexp6inv[36];               /* '<Root>/dddexp6inv' */
// } ExtY_LR_T;
