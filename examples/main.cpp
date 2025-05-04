#include "sim_ctx.hpp"
#include <IndyDualArm.hpp>
#include <Eigen/Core>
#include <spdlog/spdlog.h>
#include <ctime>
#include <iostream>
using namespace Eigen;
using Vector6d = Matrix<double, 6, 1>;
constexpr long CYCLE_NS = 1'000'000; // 1 ms = 1 kHz

int main()
{
    /* ── (1) 시뮬레이터 컨텍스트 ───────────────────────── */
    SimCtx ctx("../indy7/indy7_dualArm.urdf","../indy7/indy7_dualArm_vis.urdf");

    /* ── (2) IndyDualArm 래퍼 ─────────────────────────── */
    IndyDualArm arm;
    IndyDualArm::Arm l,r,nom_l,nom_r;
    IndyDualArm::RelArm lr,nom_lr;
    VectorXd q(12), qdot(12), q_nom(12), qdot_nom(12), q_init(12);
    qdot.setZero();
    qdot_nom.setZero();
    q << 0.165747, 0.819274, 1.26369, 0.604148, 1.0515, -0.139217,
        -0.165747, -0.819274, -1.26369, -0.604148, -1.0515, 0.139217;
    q_init = q;
    q_nom = q;
    std::string urdf_path = "../indy7/indy7_dualArm.urdf";
    arm.initialize(urdf_path, q);

    const double Tf = 100.0; // [s] 시뮬 길이
    double t = 0.0;
    int visDiv = static_cast<int>(1000.0 / SimCtx::visHz + 0.5); // 17
    int visCnt = 0;
    timespec wake{};
    clock_gettime(CLOCK_MONOTONIC, &wake);
    std::cout<<"aaaaaaaaaa";

    // spdlog::info("=== simulation start ===");
    // std::cout<<"aaaaaaaaaa";


    VectorXd HinfK(12), gamma(12),HinfK2(12);
    HinfK << 50, 50, 25, 15, 5, 0.5, 50, 50, 25, 15, 5, 0.5;
    HinfK2<<500,500,300,100,50,1,500,500,300,100,50,1;
    gamma << 1000, 800, 800, 500, 500, 300, 1000, 800, 800, 500, 500, 300;
    double dt = 0.001;
    // std::cout<<"aaaaaaaaaa";

    Eigen::Matrix4d T_start_l,T_start_r,T_start_lr;
    Eigen::Matrix4d T_end_l,T_end_r,T_end_lr;
    Eigen::MatrixXd J_l(6,6),J_r(6,6),J_lr(6,12),Jdot_l(6,6),Jdot_r(6,6),Jdot_lr(6,12);
    Vector6d V_l,V_r,V_lr;

    arm.updateFK(q,qdot,l,r,lr);
    T_start_l = l.T;
    T_start_r = r.T;
    T_start_lr = lr.T;
    T_end_l = T_start_l;
    T_end_r = T_start_r;
    T_end_l(0,3) +=0.1;
    T_end_r(0,3) +=0.1;

    T_end_lr = T_start_lr;
    
    VectorXd TaskKp(12), TaskKv(12);
    Vector2d b0;
    Vector2d a;
    TaskKp << 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6;
    TaskKv << 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6;
    TaskKv = TaskKv * dt * 30.0;

    b0 << 100, 100;
    a << 0.01, 0.01;
    VectorXd eint_nr(12), eint_dn(12), eint_dr(12);
    eint_nr.setZero();
    eint_dn.setZero();
    eint_dr.setZero();
   
    
    while (t < Tf + 2.0)
    {

    //     /*───────────────────────────Controller ─────────────────────────── */

       IndyDualArm::Des task_des_l=arm.taskSpaceTrajectory(t, T_start_l, T_end_l, 9, 9.1);
       IndyDualArm::Des task_des_r=arm.taskSpaceTrajectory(t, T_start_r, T_end_r, 9, 9.1);
       IndyDualArm::Des joint_des_l=arm.jointSpaceTrajectory(t, q_init.segment(0,6), q_init.segment(6,6), 0, 10);
       IndyDualArm::Des joint_des_r=arm.jointSpaceTrajectory(t, q_init.segment(6,6), q_init.segment(0,6), 0, 10);
       IndyDualArm::RelDes joint_des_lr= arm.setTraj(joint_des_l, joint_des_r);
       
        arm.updateFK(q,qdot,l,r,lr);
        arm.updateID(q,qdot,l,r,lr);
        arm.updateFK(q_nom,qdot_nom,nom_l,nom_r,nom_lr);
        arm.updateID(q_nom,qdot_nom,nom_l,nom_r,nom_lr); 

        VectorXd tau_hinf = arm.HinfControl(lr,q, qdot, joint_des_lr, eint_dr, dt, HinfK, gamma);
        VectorXd tau_hinf_nom = arm.HinfControl(nom_lr,q_nom, qdot_nom, joint_des_lr, eint_dn, dt, HinfK, gamma);
 
        VectorXd tau_tsc = arm.taskSpaceController(l,r,lr,q, qdot, task_des_l, task_des_r, dt, TaskKp, TaskKv, b0, a, HinfK2);
        VectorXd tau_tsc_nom = arm.taskSpaceController(nom_l,nom_r,nom_lr,q_nom, qdot_nom, task_des_l, task_des_r, dt, TaskKp, TaskKv, b0, a, HinfK2);
        VectorXd tau_a = arm.NRIC(HinfK, q, qdot, q_nom, qdot_nom, eint_nr, dt);


        arm.forwardDynamics(tau_tsc+tau_a, q, qdot);        // 한 스텝 적분
        arm.forwardDynamicsNom(tau_tsc_nom, q_nom, qdot_nom); // 한 스텝 적분
        // std::cout<<e_dn.transpose()<<std::endl;

        /*─────────────────────────────────────────────────────── */

        /* ── 60 Hz 시각화 ─────────────────────────── */
        if (++visCnt >= visDiv)
        {
            ctx.render(q,q_nom);
            visCnt = 0;
        }
        /* 주기 동기화 (1 ms) ------------------------ */
        wake.tv_nsec += CYCLE_NS;
        if (wake.tv_nsec >= 1'000'000'000L)
        {
            wake.tv_nsec -= 1'000'000'000L;
            ++wake.tv_sec;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wake, nullptr);

        t += SimCtx::dt;
    }
    arm.terminate();
    spdlog::info("=== simulation finished ===");
    return 0;
}
