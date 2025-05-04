/*─────────────────────────────────────────────────────────
 * sim_ctx.hpp – Bullet + Robot 래퍼 (IndyDualArm 은 관여 X)
 *─────────────────────────────────────────────────────────*/
#pragma once
#include "Robot.h"
#include <spdlog/spdlog.h>
#include <Eigen/Core>
#include <memory>

struct SimCtx
{
    static constexpr double dt       = 1.0 / 1000.0;   // Bullet step = 1 kHz
    static constexpr double g_z      = -9.81;          // gravity
    static constexpr int    visHz    = 60;             // render rate (Hz)

    explicit SimCtx(const char* urdfPath,const char* vis_urdfPath)
    {
        if (!sim.connect(eCONNECT_GUI))
            throw std::runtime_error("Bullet GUI 연결 실패");

        sim.setTimeStep(dt);
        sim.setGravity(btVector3(0,0,g_z));
        sim.configureDebugVisualizer(COV_ENABLE_GUI, 0);
        sim.configureDebugVisualizer( COV_ENABLE_SHADOWS, 0);
        btVector3 targetPos(0.01,-0.01,0.99);
        sim.resetDebugVisualizerCamera(1.30, -25.8, 90.20, targetPos);
        b3RobotSimulatorLoadUrdfFileArgs arg;
        arg.m_flags |= URDF_USE_INERTIA_FROM_FILE | URDF_USE_SELF_COLLISION;
        int id = sim.loadURDF(urdfPath, arg);
        int vis_id = sim.loadURDF(vis_urdfPath, arg);
        
        robot  = std::make_unique<Robot>(&sim, id);
        vis_robot  = std::make_unique<Robot>(&sim, vis_id);
        

        spdlog::info("Bullet 초기화 완료 (URDF id = {})", id);
        spdlog::info("Bullet 초기화 완료 (URDF vis_id = {})", vis_id);
        
    }

    /* 60 Hz 마다 호출 → joint 갱신 & Bullet 1 step */
    inline void render(const Eigen::VectorXd& q,const Eigen::VectorXd& q_vis)
    {
        robot->reset_q(q);
        vis_robot->reset_q(q_vis);

        sim.stepSimulation();
    }

    b3RobotSimulatorClientAPI sim;     // 공개해도 무방
private:
    std::unique_ptr<Robot>   robot;
    std::unique_ptr<Robot>   vis_robot;
    
};
