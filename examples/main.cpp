#include "sim_ctx.hpp"

#include "main.h"


void computeLambda(Eigen::Matrix4d T,Vector6d V,Eigen::Matrix4d T_d,Vector6d V_d,Vector6d Vdot_d ,Vector6d & lambda,Vector6d & lambdadot){
    const Matrix4d T_tilde = TransInv(T) * T_d; 
    lambda = se3ToVec(MatrixLog6(T_tilde)); 
    const Vector6d V_tilde = Ad(T_tilde) * V_d - V;
    const Matrix6d Dinv = dlog6(lambda);
    const Matrix6d D = dexp6(lambda);
    lambdadot = Dinv * V_tilde;
}

void setTrajectory(std::vector<Eigen::Matrix4d>& T_des_list_l,std::vector<Vector6d>& V_des_list_l,std::vector<Vector6d>& V_des_dot_list_l,
    std::vector<Eigen::Matrix4d>& T_des_list_r,std::vector<Vector6d>& V_des_list_r,std::vector<Vector6d>& V_des_dot_list_r){
    VectorXd q(12), qdot(12);
    qdot.setZero();
    
    q << 0.165747, 0.819274, 1.26369, 0.604148, 1.0515, -0.139217,
    -0.165747, -0.819274, -1.26369, -0.604148, -1.0515, 0.139217;
    arm.updateFK(q, qdot, l, r, lr_);
        Vector6d disp,disp_lr;
    disp<<0,0.1,0.0,0,0,1.5708;
     disp_lr<<0,0,-0.1,0,0,0;
    task_des_l.T_start = l.T;
    task_des_l.T_end = l.T*lr::MatrixExp6(lr::VecTose3(disp));
    task_des_r.T_start = r.T;

    task_des_r.T_end = r.T*lr::MatrixExp6(lr::VecTose3(disp));
    task_des_lr.T_start = lr_.T;

    task_des_lr.T_end = lr_.T*lr::MatrixExp6(lr::VecTose3(disp_lr));;

    Vector6d V_start_r,V_end_r,Vdot_start_r,Vdot_end_r,
    V_start_l,V_end_l,Vdot_start_l,Vdot_end_l,dlambda_max,ddlambda_max,dddlambda_max;
    V_start_l<<0,0,0,0,0,0;
    V_end_l<<0,0,0,0,0,0;
    Vdot_start_l<<0,0,0,0,0,0;
    Vdot_end_l<<0,0,0,0,0,0;

    // V_start_r<<1,2,3,1,2,3;
    // V_end_r<<-1,-2,-3,-1,-2,-3;
    // Vdot_start_r<<10,20,30,10,20,30;
    // Vdot_end_r<<-10,-20,-30,-10,-20,-30;
    V_start_r<<0,0,0,0,0,0;
    V_end_r<<0,0,0,0,0,0;
    Vdot_start_r<<0,0,0,0,0,0;
    Vdot_end_r<<0,0,0,0,0,0;

    dlambda_max<<0.1,0.1,0.1,0.1,0.1,0.1;
    ddlambda_max<<1,1,1,1,1,1;
    dddlambda_max<<10,10,10,10,10,10;
    dlambda_max=dlambda_max;
    ddlambda_max=ddlambda_max;
    dddlambda_max=dddlambda_max;

    double max_tt;
    LR_Trajectory traj;
    Matrix4d T_start,T_end;
    T_start = Matrix4d::Identity();
    T_start(0,3) = 0.5;
    T_start(1,3) = 0.5;
    T_start(2,3) = 0.5;
    

    T_end = Matrix4d::Identity();
    T_end<<           -1      ,      0 , -1.2246e-16  ,          1,
            0      ,      1 ,           0    ,        1,
   1.2246e-16        ,    0      ,     -1  ,          1,
            0       ,     0    ,        0   ,         1;
    std::vector<Vector6d> lambda_list;
    std::vector<Vector6d> lambdadot_list;
    std::vector<Vector6d> lambdaddot_list;
    traj.LieScrewScurveTrajectory(task_des_r.T_start,task_des_r.T_end ,
        V_start_r,
        V_end_r,
        Vdot_start_r,
        Vdot_end_r,
        dlambda_max,ddlambda_max,dddlambda_max,
        0.001,
        T_des_list_r,
        V_des_list_r,
        V_des_dot_list_r,
        lambda_list,
        lambdadot_list,
        lambdaddot_list,
        max_tt);
    traj.LieScrewScurveTrajectory(task_des_l.T_start ,task_des_l.T_end ,
        V_start_l,
        V_end_l,
        Vdot_start_l,
        Vdot_end_l,
        dlambda_max,ddlambda_max,dddlambda_max,
        0.001,
        T_des_list_l,
        V_des_list_l,
        V_des_dot_list_l,
        max_tt);


    task_des_r.T = task_des_r.T_start;
    task_des_l.T = task_des_l.T_start;

    saveTrajectoryToCSV("trajectory_r.csv", T_des_list_r, V_des_list_r, V_des_dot_list_r,lambda_list,lambdadot_list,lambdaddot_list);
    std::cout<<"Save Done--Size:"<<lambda_list.size()<<std::endl;

}
int main()
{
    /* ── (1) 시뮬레이터 컨텍스트 ───────────────────────── */
    SimCtx ctx("../indy7/indy7_dualArm.urdf", "../indy7/indy7_dualArm_vis.urdf");

    /* ── (2) IndyDualArm 래퍼 ─────────────────────────── */
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


    MatrixXd B, Bl, Br,Bnomr,Bnoml;
    double b0 = 1;
    double a = 0.01;
    int count_l=0;
    int count_r=0;
    std::vector<Eigen::Matrix4d> T_des_list_r;
    std::vector<Vector6d> V_des_list_r;
    std::vector<Vector6d> V_des_dot_list_r;
    std::vector<Eigen::Matrix4d> T_des_list_l;
    std::vector<Vector6d> V_des_list_l;
    std::vector<Vector6d> V_des_dot_list_l;
     setTrajectory( T_des_list_l,V_des_list_l, V_des_dot_list_l,T_des_list_r,V_des_list_r, V_des_dot_list_r);
    double dt = 0.001;
    Vector6d SPDECKp, SPDECKd;
    VectorXd tau_a = VectorXd::Zero(12);
    VectorXd e_nr(12),edot_nr(12),eint_nr(12);
    e_nr.setZero();edot_nr.setZero();eint_nr.setZero();
    
    

    // SPDECKp << 100000, 100000, 100000, 1000, 1000, 1000;
    SPDECKp << 100000, 100000, 100000, 100000, 100000,100000;

    VectorXd Ka = VectorXd::Zero(12);
    Ka<<100,100,50,30,10,0.1,100,100,50,30,10,0.1;
    // Ka = Ka*0.01;
    SPDECKd<< 100000*dt*30, 100000*dt*30, 100000*dt*30, 100000*dt*100, 100000*dt*100, 100000*dt*100;
    MatrixXd N=MatrixXd::Zero(12,12);
    N.block<3,3>(0,0) = MatrixXd::Identity(6,6);
    Matrix4d T_start_r,T_end_r,T_end_r2;
    arm.updateFK(q, qdot, l, r, lr_);
    T_start_r= r.T;
    T_end_r= r.T;
    T_end_r2 = r.T;
    T_end_r(1,3) +=0.1; 
    T_end_r2(1,3) +=0.1;
    Vector3d disp_xi;
    disp_xi<<1.5708/2.0,0,0;
    T_end_r.block<3,3>(0,0) = T_end_r.block<3,3>(0,0)*MatrixExp3(VecToso3(disp_xi));
    T_end_r2.block<3,3>(0,0) = T_end_r2.block<3,3>(0,0)*MatrixExp3(VecToso3((-1)*disp_xi));


    Matrix4d T_start_lr,T_end_lr,T_end_lr2;
    // task_des_lr.T <<   -1.0000,   -0.0000  , -0.000  ,  0.0000,
    //                     0.0000 ,   1.0000 ,  -0.00  ,  0.000,
    //                     0.000  , -0.00 ,  -1.0000 ,   0.500,
    //                         0    ,     0  ,       0 ,   1.0000;
    T_start_lr  <<   -1.0000,   -0.0000  , -0.000  ,  0.0000,
                            0.0000 ,   1.0000 ,  -0.00  ,  0.000,
                            0.000  , -0.00 ,  -1.0000 ,   0.500,
                                0    ,     0  ,       0 ,   1.0000;
    T_end_lr=T_start_lr;
    T_end_lr2=T_start_lr;
    T_end_lr(2,3) -=0.25;
    T_end_lr2(2,3) +=0.25;
     
    task_des_lr.V = Vector6d::Zero();
    task_des_lr.Vdot = Vector6d::Zero();
    while (t < Tf + 2.0)
    {

        //     /*───────────────────────────Controller ─────────────────────────── */
        if(t<5){
            task_des_r = arm.taskSpaceTrajectory(t, T_start_r, T_end_r,0, 5);
            task_des_lr = arm.taskSpaceTrajectory(t, T_start_lr, T_end_lr,0, 5);
        }
        else if(t>=5&& t<10){
            task_des_r = arm.taskSpaceTrajectory(t, T_end_r, T_start_r,5, 10);
            task_des_lr = arm.taskSpaceTrajectory(t, T_end_lr, T_start_lr,5, 10);

        }
        else if(t>=10&& t<15){
            task_des_r = arm.taskSpaceTrajectory(t, T_start_r, T_end_r2,10, 15);
            task_des_lr = arm.taskSpaceTrajectory(t, T_start_lr, T_end_lr2,10, 15);

        }
        else if(t>=15&& t<20){
            task_des_r = arm.taskSpaceTrajectory(t, T_end_r2, T_start_r,15, 20);
             task_des_lr = arm.taskSpaceTrajectory(t, T_end_lr2, T_start_lr,15, 20);
        }else{
                 task_des_r = arm.taskSpaceTrajectory(t, T_end_r2, T_start_r,15, 20);
             task_des_lr = arm.taskSpaceTrajectory(t, T_end_lr2, T_start_lr,15, 20);
        }
        
        // task_des_lr = arm.taskSpaceTrajectory(t, task_des_lr.T_start, task_des_lr.T_end,10, 15);
        // joint_des_l = arm.jointSpaceTrajectory(t, q_init.segment(0, 6), q_init.segment(6, 6), 0, 10);
        // joint_des_r = arm.jointSpaceTrajectory(t, q_init.segment(6, 6), q_init.segment(0, 6), 0, 10);
        // joint_des_lr = arm.setTraj(joint_des_l, joint_des_r);
       
        
        
        arm.updateFK(q, qdot, l, r, lr_);
        arm.updateID(q, qdot, l, r, lr_);
        arm.updateFK(q_nom, qdot_nom, nom_l, nom_r, nom_lr);
        arm.updateID(q_nom, qdot_nom, nom_l, nom_r, nom_lr);
        // Matrix4d Ttilde = TransInv(lr.T) * task_des_lr.T;
        // Vector6d V_tilde = Ad(TransInv(Ttilde)) * task_des_lr.V - lr.V;
        // if (count_r<T_des_list_r.size()){
        //     task_des_r.T= T_des_list_r.at(count_r);
        //     task_des_r.V= V_des_list_r.at(count_r);
        //     task_des_r.Vdot= V_des_dot_list_r.at(count_r++);
        // }else{
        //     task_des_r.T = task_des_r.T_end;
        //     task_des_r.V = task_des_r.V_end;
        //     task_des_r.Vdot = task_des_r.Vdot_end;
            
        // }
        // if (count_l<T_des_list_l.size()){
        //     task_des_l.T= T_des_list_l.at(count_l);
        //     task_des_l.V= V_des_list_l.at(count_l);
        //     task_des_l.Vdot= V_des_dot_list_l.at(count_l++);
        // }else{
        //     task_des_l.T = task_des_l.T_end;
        //     task_des_l.V = task_des_l.V_end;
        //     task_des_l.Vdot = task_des_l.Vdot_end;
            
        // }
     
     
        Bnomr = b0 * exp(-nom_r.J.determinant() * nom_r.J.determinant() / a / a) * MatrixXd::Identity(6, 6);  
        Bnoml = b0 * exp(-nom_l.J.determinant() * nom_l.J.determinant() / a / a) * MatrixXd::Identity(6, 6);  
 
        VectorXd qddot_nom_r = SPDEC(nom_r.M,nom_r.C,Bnomr,q_nom.segment(6, 6), qdot_nom.segment(6, 6),
                                    task_des_r.T, task_des_r.V, task_des_r.Vdot,nom_r.T,nom_r.J,nom_r.Jdot,
                                    SPDECKp, SPDECKd, 0.001);
        // VectorXd qddot_nom_l = SPDEC(nom_l.M,nom_l.C,Bnoml,q_nom.segment(0, 6), qdot_nom.segment(0, 6),
        //                             task_des_l.T, task_des_l.V, task_des_l.Vdot,nom_l.T,nom_l.J,nom_l.Jdot,
        //                             SPDECKp, SPDECKd, 0.001);
        VectorXd qddot_nom_l = SPDEC(nom_l.M,nom_l.C,Bnoml,q_nom.segment(0, 6), qdot_nom.segment(0, 6),
                                    task_des_lr.T, task_des_lr.V, task_des_lr.Vdot,nom_lr.T,(-1)*Ad(TransInv(nom_lr.T))*nom_l.J,
                                    (-1)*Ad(TransInv(nom_lr.T))*nom_l.Jdot-ad(Ad(TransInv(nom_lr.T))*nom_l.V-nom_r.V)*Ad(TransInv(nom_lr.T))*nom_l.J,
                                    SPDECKp, SPDECKd, 0.001);                                    
        VectorXd qddot_nom(12);
        qddot_nom.segment(0,6) = qddot_nom_l;
        qddot_nom.segment(6,6) = qddot_nom_r;
        qdot_nom = qdot_nom+qddot_nom*dt;
        q_nom = q_nom+qdot_nom*dt;
        

        VectorXd tau_cr = PDEC(Bnomr,q_nom.segment(6, 6), qdot_nom.segment(6, 6),
                                    task_des_r.T, task_des_r.V, task_des_r.Vdot,nom_r.T,nom_r.J,nom_r.Jdot,
                                    SPDECKp, SPDECKd, 0.001);
        //  VectorXd tau_cl = PDEC(Bnoml,q_nom.segment(0, 6), qdot_nom.segment(0, 6),
        //                             task_des_l.T, task_des_l.V, task_des_l.Vdot,nom_l.T,nom_l.J,nom_l.Jdot,
        //                             SPDECKp, SPDECKd, 0.001);
        VectorXd tau_cl = PDEC(Bnoml,q_nom.segment(0, 6), qdot_nom.segment(0, 6),
                                    task_des_lr.T, task_des_lr.V, task_des_lr.Vdot,nom_lr.T,(-1)*Ad(TransInv(nom_lr.T))*nom_l.J,(-1)*Ad(TransInv(nom_lr.T))*nom_l.Jdot-ad(Ad(TransInv(nom_lr.T))*nom_l.V-nom_r.V)*Ad(TransInv(nom_lr.T))*nom_l.J,
                                    SPDECKp, SPDECKd, 0.001);
        VectorXd tau_c = VectorXd::Zero(12); 
        tau_c.segment(0,6) =tau_cl+ l.g;
        tau_c.segment(6,6) =tau_cr+ r.g;
        e_nr = q_nom- q;
        edot_nr = qdot_nom - qdot;
        eint_nr += e_nr*dt;

        
        tau_a = Ka.asDiagonal()*(edot_nr+20*e_nr+100*eint_nr);

            // saveState(q, qdot, tau_c + tau_a, t);
      

        Vector6d lambda_l,lambdadot_l,lambda_lr,lambdadot_lr,lambda_r,lambdadot_r,lambda_nom_r,lambdadot_nom_r;

        computeLambda(l.T,l.V,task_des_l.T,task_des_l.V,task_des_l.Vdot ,lambda_l,lambdadot_l);
        computeLambda(lr_.T,lr_.V,task_des_lr.T,task_des_lr.V,task_des_lr.Vdot ,lambda_lr,lambdadot_lr);
        computeLambda(r.T,r.V,task_des_r.T,task_des_r.V,task_des_r.Vdot ,lambda_r,lambdadot_r);
        computeLambda(nom_r.T,nom_r.V,task_des_r.T,task_des_r.V,task_des_r.Vdot ,lambda_nom_r,lambdadot_nom_r);

        saveLambda("lambda_l.csv", lambda_l, lambdadot_l, t);
        saveLambda("lambda_r.csv", lambda_r, lambdadot_r, t);
        saveLambda("lambda_lr.csv", lambda_lr, lambdadot_lr, t);

        std::cout<<"r:"<<lambda_r.transpose()<<std::endl;
        std::cout<<"nom_r:"<<lambda_nom_r.transpose()<<std::endl;

        saveTaskSpace(l.T, r.T, l.V, r.V,  t);
        saveQ( q,  t);
        saveQdot( qdot,  t);
   
        // lambda = arm.log6(Ttilde);
        // lambdadot = arm.dexp6inv(lambda) * V_tilde;

        // VectorXd qdot_lr = flipVector(qdot);
        // MatrixXd J1 = MatrixXd::Zero(6, 12);
        // MatrixXd J2 = lr.J;
        // J1.block<6, 6>(0, 6) = r.J;

        // Vector6d F1 = Vector6d::Zero();

        // Matrix4d Ttilde_1 = TransInv(r.T) * task_des_r.T;
        // Vector6d V_tilde_1 = task_des_r.V - Ad(TransInv(Ttilde_1)) * r.V;
        // Vector6d lambda_1 = arm.log6(Ttilde_1);
        // Vector6d lambdadot_1 = arm.dexp6inv(-lambda_1) * V_tilde_1;

        // F1 = (Kp.asDiagonal() * lambda_1 + Kd.asDiagonal() * lambdadot_1);
        // Vector6d Frel = (Kp.asDiagonal() * lambda + Kd.asDiagonal() * lambdadot);
        // J2.block<6, 6>(0, 6) = J2.block<6, 6>(0, 6) * 0;


        // VectorXd tau_con = J1.transpose() * F1 * 0 + J2.transpose() * (arm.dexp6(lambda).transpose() * Frel);


        // Matrix4d Ttilde_n = TransInv(nom_r.T) * task_des_r.T;
        // Vector6d V_tilde_n = Ad(Ttilde_n) * task_des_r.V-nom_r.V ;
        // Vector6d lambda_n = arm.log6(Ttilde_n);
        // Vector6d lambdadot_n = arm.dexp6inv(lambda_n) * V_tilde_n;

        // VectorXd tau_c = VectorXd::Zero(12); 
        // tau_c.segment(6,6) = nom_r.J.transpose()*dexp6(lambda_n).transpose()*(SPDECKp.asDiagonal()*lambda_n+SPDECKd.asDiagonal()*lambdadot_n)+ nom_r.g-Bnomr*qdot_nom;
        // tau_c.segment(0,6) = l.g;
        // tau_con = flipVector(tau_con);
        // VectorXd tau_tsc = lr.g + lr.c + tau_con - B * qdot;
        // VectorXd tau_a = VectorXd::Zero(12);
        // e_nr = q_nom.segment(6,6) - q.segment(6,6);
        // edot_nr = qdot_nom.segment(6,6) - qdot.segment(6,6);
        // eint_nr += e_nr*dt;
        // VectorXd Ka = VectorXd::Zero(6);
        // Ka<<500,500,150,50,30,0.01;
        // tau_a.segment(6,6) = Ka.asDiagonal()*(edot_nr+20*e_nr+100*eint_nr);
        // joint_des_lr.q.segment(6,6) = q_nom.segment(6,6);
        // joint_des_lr.qdot.segment(6,6) = qdot_nom.segment(6,6);
        // joint_des_lr.qddot.segment(6,6) = qddot_nom.segment(6,6)*0;

        // // VectorXd tau_hinf = arm.HinfControl(lr, q, qdot, joint_des_lr, eint_dr, dt, HinfK, gamma);
        //  arm.forwardDynamics(tau_c+tau_a, q, qdot);            // 한 스텝 적분
        // arm.forwardDynamicsNom(tau_tsc, q_nom, qdot_nom); // 한 스텝 적분
        arm.forwardDynamics(tau_c+tau_a, q, qdot);

        //    q = q_nom;
        /*─────────────────────────────────────────────────────── */

        /* ── 60 Hz 시각화 ─────────────────────────── */
        if (++visCnt >= visDiv)
        {
            ctx.render(q, q_nom);
            ctx.setAxis(task_des_r.T*TransInv(task_des_lr.T),task_des_r.T);
            ctx.setDesAxis(l.T,r.T);
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
