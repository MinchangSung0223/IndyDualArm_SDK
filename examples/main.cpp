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
    VectorXd q(12),qdot(12);
    qdot.setZero();
    q << 0.165747, 0.819274, 1.26369, 0.604148, 1.0515, -0.139217,
        -0.165747, -0.819274, -1.26369, -0.604148, -1.0515, 0.139217;
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
        VectorXd tau_hinf = arm.hinfController(q, qdot, HinfK);
        VectorXd tau_tsc = arm.taskSpaceController(q, qdot, dt, TaskKp, TaskKv, b0, a, HinfK);
        arm.forwardDynamics(tau_tsc, q, qdot); // 한 스텝 적분

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
