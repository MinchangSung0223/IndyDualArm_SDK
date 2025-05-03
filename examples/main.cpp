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
    SimCtx ctx("../indy7/indy7_dualArm.urdf");

    /* ── (2) IndyDualArm 래퍼 ─────────────────────────── */
    IndyDualArm arm;
    
    VectorXd q(12),qdot(12),q_nom(12), qdot_nom(12);
    qdot.setZero();
    qdot_nom.setZero();
    q << 0.165747, 0.819274, 1.26369, 0.604148, 1.0515, -0.139217,
        -0.165747, -0.819274, -1.26369, -0.604148, -1.0515, 0.139217;
    q_nom = q;
    arm.initialize(q);


    const double Tf = 100.0; // [s] 시뮬 길이
    double t = 0.0;
    int visDiv = static_cast<int>(1000.0 / SimCtx::visHz + 0.5); // 17
    int visCnt = 0;
    timespec wake{};
    clock_gettime(CLOCK_MONOTONIC, &wake);
    spdlog::info("=== simulation start ===");

    arm.des_l.q_start = q.segment(0, 6);
    arm.des_l.q_end = q.segment(6, 6);
    arm.des_r.q_start = q.segment(6, 6);
    arm.des_r.q_end = q.segment(0, 6);

    VectorXd HinfK(12);
    HinfK << 500, 500, 100, 50, 10, 1, 500, 500, 100, 50, 10, 1;
    VectorXd eint(12);
    eint.setZero();
    double dt = 0.001;

    arm.forwardKinematics(q, VectorXd::Zero(12));
    arm.des_l.T_start = arm.l.T;
    arm.des_l.T_end = arm.des_l.T_start;
    arm.des_l.T_end(2, 3) = arm.des_l.T_end(2, 3) + 0.5;
    arm.des_r.T_start = arm.r.T;
    arm.des_r.T_end = arm.des_r.T_start;
    arm.des_r.T_end(2, 3) = arm.des_r.T_end(2, 3) + 0.5;

    VectorXd TaskKp(12), TaskKv(12);
    Vector2d b0;
    Vector2d a;
    TaskKp << 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6;
    TaskKv << 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6;
    TaskKv = TaskKv * dt * 30.0;

    b0 << 100, 100;
    a << 0.01, 0.01;
    VectorXd eint_nr(12),eint_dn(12),eint_dr(12);
    eint_nr.setZero();
    eint_dn.setZero();
    eint_dr.setZero();
    while (t < Tf + 2.0)
    {

        /*───────────────────────────Controller ─────────────────────────── */

        arm.taskSpaceTrajectory(t, arm.des_l.T_start, arm.des_l.T_end, 0, 10, arm.des_l);
        arm.taskSpaceTrajectory(t, arm.des_r.T_start, arm.des_r.T_end, 0, 10, arm.des_r);
        arm.jointSpaceTrajectory(t, arm.des_l.q_start, arm.des_l.q_end, 0, 10, arm.des_l);
        arm.jointSpaceTrajectory(t, arm.des_r.q_start, arm.des_r.q_end, 0, 10, arm.des_r);
        arm.setTraj(arm.des_l, arm.des_r);

        arm.forwardKinematics(q, qdot);
        arm.inverseDynamics(q, qdot);
        arm.inverseDynamicsNom(q_nom, qdot_nom);
        
        VectorXd tau_hinf = arm.hinfController(q, qdot, HinfK*0.1,0.001,eint_dr);
        VectorXd e_dn = arm.des_lr.q - q_nom;
        VectorXd edot_dn = arm.des_lr.qdot - qdot_nom;
        eint_dn = eint_dn+ e_dn*dt;
        VectorXd tau_hinf_nom = arm.nom_lr.M*(arm.des_lr.qddot+20*edot_dn+100*e_dn)+arm.nom_lr.c +arm.nom_lr.g+0.1*HinfK.asDiagonal()*(edot_dn+20.0*e_dn+100.0*eint_dn);
        

        VectorXd tau_tsc = arm.taskSpaceController(q, qdot, dt, TaskKp, TaskKv, b0, a, HinfK);
        // VectorXd tau_a = arm.NRIC(HinfK*0.001,q,qdot,q_nom,qdot_nom,eint_nr,dt);
        arm.forwardDynamics(tau_hinf, q, qdot); // 한 스텝 적분
        arm.forwardDynamicsNom(tau_hinf_nom, q_nom, qdot_nom); // 한 스텝 적분
        std::cout<<e_dn.transpose()<<std::endl;

        

        /*─────────────────────────────────────────────────────── */

        /* ── 60 Hz 시각화 ─────────────────────────── */
        if (++visCnt >= visDiv)
        {
            ctx.render(q);
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
