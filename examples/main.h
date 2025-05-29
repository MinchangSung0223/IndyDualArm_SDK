#include <fstream>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <iostream>
#include <IndyDualArm.hpp>
#include <Eigen/Core>
#include <spdlog/spdlog.h>
#include <LR/include/liegroup_robotics.h>
#include <LR/include/LR_Trajectory.h>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <iostream>
using namespace Eigen;
using namespace lr;
using Vector6d = Matrix<double, 6, 1>;
constexpr long CYCLE_NS = 1'000'000; // 1 ms = 1 kHz

using Vector6d = Eigen::Matrix<double, 6, 1>;
IndyDualArm arm;
IndyDualArm::Arm l, r, nom_l, nom_r;
IndyDualArm::RelArm lr_, nom_lr;
IndyDualArm::Des task_des_l;
IndyDualArm::Des task_des_r;
IndyDualArm::Des task_des_lr;
IndyDualArm::Des joint_des_l;
IndyDualArm::Des joint_des_r;
IndyDualArm::RelDes joint_des_lr;
void saveTrajectoryToCSV(const std::string& filename,
                         const std::vector<Eigen::Matrix4d>& T_des_list,
                         const std::vector<Vector6d>& V_des_list,
                         const std::vector<Vector6d>& V_des_dot_list) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    // Header
    file << "qw,qx,qy,qz,px,py,pz,"
         << "Vx,Vy,Vz,wx,wy,wz,"
         << "Vx_dot,Vy_dot,Vz_dot,wx_dot,wy_dot,wz_dot";

    size_t N = T_des_list.size();
    for (size_t i = 0; i < N; ++i) {
        const Eigen::Matrix3d R = T_des_list[i].block<3,3>(0,0);
        const Eigen::Vector3d p = T_des_list[i].block<3,1>(0,3);
        Eigen::Quaterniond q(R);

        const Vector6d& V = V_des_list[i];
        const Vector6d& Vdot = V_des_dot_list[i];

        file << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << ","
             << p(0) << "," << p(1) << "," << p(2) << ",";

        for (int j = 0; j < 6; ++j)
            file << V(j) << (j < 5 ? "," : ",");

        for (int j = 0; j < 6; ++j)
            file << Vdot(j) << (j < 5 ? "," : "");

        file << "\n";
    }

    file.close();
    std::cout << "CSV saved to: " << filename << std::endl;
}

void saveTrajectoryToCSV(const std::string& filename,
                         const std::vector<Eigen::Matrix4d>& T_des_list,
                         const std::vector<Vector6d>& V_des_list,
                         const std::vector<Vector6d>& V_des_dot_list,
                         const std::vector<Vector6d>& lambda_list,
                         const std::vector<Vector6d>& lambdadot_list,
                         const std::vector<Vector6d>& lambdaddot_list) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    // Header
    file << "qw,qx,qy,qz,px,py,pz,"
         << "Vx,Vy,Vz,wx,wy,wz,"
         << "Vx_dot,Vy_dot,Vz_dot,wx_dot,wy_dot,wz_dot,"
         << "lambda1,lambda2,lambda3,lambda4,lambda5,lambda6,"
         << "lambdadot1,lambdadot2,lambdadot3,lambdadot4,lambdadot5,lambdadot6,"
         << "lambdaddot1,lambdaddot2,lambdaddot3,lambdaddot4,lambdaddot5,lambdaddot6\n";

    size_t N = T_des_list.size();
    for (size_t i = 0; i < N; ++i) {
        const Eigen::Matrix3d R = T_des_list[i].block<3,3>(0,0);
        const Eigen::Vector3d p = T_des_list[i].block<3,1>(0,3);
        Eigen::Quaterniond q(R);

        const Vector6d& V = V_des_list[i];
        const Vector6d& Vdot = V_des_dot_list[i];
        const Vector6d& lambda = lambda_list[i];
        const Vector6d& lambdadot = lambdadot_list[i];
        const Vector6d& lambdaddot = lambdaddot_list[i];

        file << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << ","
             << p(0) << "," << p(1) << "," << p(2) << ",";

        for (int j = 0; j < 6; ++j)
            file << V(j) << ",";

        for (int j = 0; j < 6; ++j)
            file << Vdot(j) << ",";

        for (int j = 0; j < 6; ++j)
            file << lambda(j) << ",";

        for (int j = 0; j < 6; ++j)
            file << lambdadot(j) << ",";

        for (int j = 0; j < 6; ++j)
            file << lambdaddot(j) << (j < 5 ? "," : "");

        file << "\n";
    }

    file.close();
    std::cout << "CSV saved to: " << filename << std::endl;
}

void saveLambda(const std::string& filename,
                const Vector6d& lambda,
                const Vector6d& lambdadot,
                double t)
{
    static std::unordered_map<std::string, bool> first_call;
    static std::unordered_map<std::string, std::ofstream> file_map;

    std::ofstream& file = file_map[filename];
    if (!file.is_open()) {
        file.open(filename, std::ios::out | std::ios::app);
        if (!file.is_open() || file.fail()) return;
        file << std::fixed << std::setprecision(10);
    }

    if (first_call[filename]) {
        file << "t";
        for (int i = 0; i < 6; ++i) file << ",lambda" << (i + 1);
        for (int i = 0; i < 6; ++i) file << ",lambdadot" << (i + 1);
        file << "\n";
        first_call[filename] = false;
    }

    file << t;
    for (int i = 0; i < 6; ++i) file << "," << lambda[i];
    for (int i = 0; i < 6; ++i) file << "," << lambdadot[i];
    file << "\n";
}

void saveQdot(const Eigen::VectorXd& qdot, double t) {
    static bool first_call = true;
    static std::ofstream file("log_qdot.csv", std::ios::out | std::ios::app);
    if (!file.is_open() || file.fail()) return;

    file << std::fixed << std::setprecision(10);

    if (first_call) {
        file << "t";
        for (int i = 0; i < qdot.size(); ++i) file << ",dq" << (i+1);
        file << "\n";
        first_call = false;
    }

    file << t;
    for (int i = 0; i < qdot.size(); ++i) file << "," << qdot[i];
    file << "\n";
}

void saveQ(const Eigen::VectorXd& q, double t) {
    static bool first_call = true;
    static std::ofstream file("log_q.csv", std::ios::out | std::ios::app);
    if (!file.is_open() || file.fail()) return;

    file << std::fixed << std::setprecision(10);

    if (first_call) {
        file << "t";
        for (int i = 0; i < q.size(); ++i) file << ",q" << (i+1);
        file << "\n";
        first_call = false;
    }

    file << t;
    for (int i = 0; i < q.size(); ++i) file << "," << q[i];
    file << "\n";
}

VectorXd flipVector(VectorXd qdot)
{
    VectorXd q_lr = qdot;
    q_lr(0) = -qdot(5);
    q_lr(1) = -qdot(4);
    q_lr(2) = -qdot(3);
    q_lr(3) = -qdot(2);
    q_lr(4) = -qdot(1);
    q_lr(5) = -qdot(0);

    q_lr.segment(6, 6) = qdot.segment(6, 6);
    return q_lr;
}
Eigen::Matrix<double, 6, 1>
saturate(const Eigen::Matrix<double, 6, 1> &v,
         const Eigen::Matrix<double, 6, 1> &lower,
         const Eigen::Matrix<double, 6, 1> &upper)
{
    return v.cwiseMin(upper).cwiseMax(lower);
}



void saveTaskSpace(const Matrix4d& Tl, const Matrix4d& Tr,
                   const Vector6d& Vl, const Vector6d& Vr,
                   double t) {
    static bool first_call = true;
    static std::ofstream file("task_log.csv", std::ios::out | std::ios::app);
    if (!file.is_open() || file.fail()) return;

    file << std::fixed << std::setprecision(10);

    if (first_call) {
        file << "t,"
             << "Tl_qw,Tl_qx,Tl_qy,Tl_qz,Tl_px,Tl_py,Tl_pz,"
             << "Tr_qw,Tr_qx,Tr_qy,Tr_qz,Tr_px,Tr_py,Tr_pz,"
             << "Vl_1,Vl_2,Vl_3,Vl_4,Vl_5,Vl_6,"
             << "Vr_1,Vr_2,Vr_3,Vr_4,Vr_5,Vr_6\n";
        first_call = false;
    }

    auto matToQuatPos = [](const Matrix4d& T) {
        Eigen::Quaterniond q(T.block<3,3>(0,0));
        Eigen::Vector3d p = T.block<3,1>(0,3);
        return std::vector<double>{q.w(), q.x(), q.y(), q.z(), p[0], p[1], p[2]};
    };

    std::vector<double> qpos_l = matToQuatPos(Tl);
    std::vector<double> qpos_r = matToQuatPos(Tr);

    file << t;
    for (double v : qpos_l) file << "," << v;
    for (double v : qpos_r) file << "," << v;
    for (int i = 0; i < 6; ++i) file << "," << Vl[i];
    for (int i = 0; i < 6; ++i) file << "," << Vr[i];
    file << "\n";
}


Eigen::VectorXd SPDEC(const Eigen::MatrixXd &M,
                      const Eigen::MatrixXd &C,
                      const Eigen::MatrixXd &B,
                      const Eigen::VectorXd &q,
                      const Eigen::VectorXd &qdot,
                      const Matrix4d &T_d,
                      const Vector6d &V_d,
                      const Vector6d &Vdot_d,
                      const Matrix4d &T,
                      const Eigen::MatrixXd &J,
                      const Eigen::MatrixXd &Jdot,
                      const Eigen::VectorXd &Kp,
                      const Eigen::VectorXd &Kd,
                      double dt)
{
    const std::size_t n = q.size();

    // ---------------------------------------------------------------------
    // Pose & velocity errors in exponential coordinates
    // ---------------------------------------------------------------------
    const Matrix4d T_tilde = TransInv(T) * T_d;            // current → desired
    const Vector6d lambda = arm.log6(T_tilde); // pose error

    const Matrix6d AdTilde = Ad(T_tilde);

    const Eigen::VectorXd V = J * qdot;         // actual spatial vel.
    const Vector6d V_tilde = AdTilde * V_d - V; // velocity error
    const Matrix6d Dinv = dlog6(lambda);        // inverse of dexp
    const Matrix6d D = dexp6(lambda);           // forward dexp

    const Vector6d lambdadot = Dinv * V_tilde;       // time‑derivative of λ
    const Matrix6d Ddot = ddexp6(lambda, lambdadot); // derivative of dexp

    // Gain matrices (diagonal)
    const Matrix6d Kp_mat = Kp.asDiagonal();
    const Matrix6d Kd_mat = Kd.asDiagonal();

    // ---------------------------------------------------------------------
    // Γ₁ term (6×1)
    // ---------------------------------------------------------------------
    const MatrixXd ast1 =(dt*Kd_mat+dt*dt*Kp_mat)*Ddot*AdTilde;
    const MatrixXd ast2 = ((dt*Kp_mat+Kd_mat)*D+(dt*Kd_mat+dt*dt*Kp_mat)*Ddot)*AdTilde;
    const MatrixXd ast3 =(-1)*((dt*Kd_mat+dt*dt*Kp_mat)*(Ddot+D+D*ad(AdTilde*V_d))+(dt*Kp_mat+Kd_mat)*D);
       
    // ---------------------------------------------------------------------
    // Γ₂ term (6×n)
    // ---------------------------------------------------------------------


    // ---------------------------------------------------------------------
    // Joint‑space acceleration command
    // ---------------------------------------------------------------------
    const Eigen::MatrixXd A = M + J.transpose() * Dinv.transpose() * Kd_mat * dt * D * J;
    const Eigen::VectorXd rhs =
        J.transpose() * Dinv.transpose() * (Kp_mat*lambda+ast1*Vdot_d+ast2*V_d+ast3*J*qdot) - B * qdot - C * qdot;

    // Use a robust solver (LDLT assumes M ≻ 0) --------------------------------
    return A.ldlt().solve(rhs);
}


#include <Eigen/Dense>

using Eigen::VectorXd;

void saveState(const VectorXd& q, const VectorXd& qdot, const VectorXd& tau, double t) {
    static bool first_call = true;
    static std::ofstream file("state_log.csv", std::ios::out | std::ios::app);
    if (!file.is_open() || file.fail()) return;

    file << std::fixed << std::setprecision(10);

    if (first_call) {
        file << "t";
        for (int i = 0; i < q.size(); ++i) file << ",q" << (i+1);
        for (int i = 0; i < qdot.size(); ++i) file << ",dq" << (i+1);
        for (int i = 0; i < tau.size(); ++i) file << ",tau" << (i+1);
        file << "\n";
        first_call = false;
    }

    file << t;
    for (int i = 0; i < q.size(); ++i) file << "," << q[i];
    for (int i = 0; i < qdot.size(); ++i) file << "," << qdot[i];
    for (int i = 0; i < tau.size(); ++i) file << "," << tau[i];
    file << "\n";
}

Eigen::VectorXd  PDEC(const Eigen::MatrixXd &B,
                      const Eigen::VectorXd &q,
                      const Eigen::VectorXd &qdot,
                      const Matrix4d &T_d,
                      const Vector6d &V_d,
                      const Vector6d &Vdot_d,
                      const Matrix4d &T,
                      const Eigen::MatrixXd &J,
                      const Eigen::MatrixXd &Jdot,
                      const Eigen::VectorXd &Kp,
                      const Eigen::VectorXd &Kd,
                      double dt){
                        
    const Matrix4d T_tilde = TransInv(T) * T_d;            // current → desired
    const Vector6d lambda = se3ToVec(MatrixLog6(T_tilde)); // pose error

    const Matrix6d AdTilde = Ad(T_tilde);

    const Eigen::VectorXd V = J * qdot;         // actual spatial vel.
    const Vector6d V_tilde = AdTilde * V_d - V; // velocity error
    const Matrix6d Dinv = dlog6(lambda);        // inverse of dexp
    const Matrix6d D = dexp6(lambda);           // forward dexp

    const Vector6d lambdadot = Dinv * V_tilde;       // time‑derivative of λ
    const Matrix6d Ddot = ddexp6(lambda, lambdadot); // derivative of dexp
     const Matrix6d Kp_mat = Kp.asDiagonal();
    const Matrix6d Kd_mat = Kd.asDiagonal();
    VectorXd tau(6);
    tau = J.transpose() * Dinv.transpose() * (Kp_mat*lambda+Kd_mat*lambdadot)- B * qdot;
    return tau;
}