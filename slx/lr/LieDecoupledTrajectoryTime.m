function [X_traj,V_traj,Vdot_traj]=LieDecoupledTrajectory(X_start,X_end,V_start,V_end,Vdot_start,Vdot_end,Tf,N)
timegap = Tf / (N - 1);
X_traj = cell(1, N);
V_traj =  cell(1, N);
Vdot_traj =  cell(1, N);

R_start = X_start(1:3,1:3);
R_end = X_end(1:3,1:3);

xi0 = zeros(3,1);
xiT = so3ToVec(MatrixLog3((R_start)'*R_end));
% xiT = [xiT(4:6);xiT(1:3)];

xi_dot0 = V_start(4:6);
xi_ddot0 =Vdot_start(4:6);
xi_dotT = dlog3(-xiT)*V_end(4:6);
xi_ddotT =dlog3(-xiT)*Vdot_end(4:6)+ddlog3(-xiT,-xi_dotT)*V_end(4:6);

px0 = X_start(1,4);
pxT = X_end(1,4);
vx0 = V_start(1); %% if V is body twist , need to change pdotx
vxT = V_end(1); %% if V is body twist , need to change pdotx
ax0 = Vdot_start(1); %% if V is body twist , need to change pddotx
axT = Vdot_end(1); %% if V is body twist , need to change pdotx



py0 = X_start(2,4);
pyT = X_end(2,4);
vy0 = V_start(2); %% if V is body twist , need to change pdoty
vyT = V_end(2); %% if V is body twist , need to change pdotx

ay0 = Vdot_start(2); %% if V is body twist , need to change pddoty
ayT = Vdot_end(2); %% if V is body twist , need to change pdotx

pz0 = X_start(3,4);
pzT = X_end(3,4);
vz0 = V_start(2); %% if V is body twist , need to change pdotz
vzT = V_end(3); %% if V is body twist , need to change pdotx

az0 = Vdot_start(3); %% if V is body twist , need to change pddotz
azT = Vdot_end(3); %% if V is body twist , need to change pdotx


for i = 1: N
    t = timegap * (i - 1);
    xi_t = zeros(3,1);
    xi_dot_t = zeros(3,1);
    xi_ddot_t = zeros(3,1);
    for j =1:1:3
        [xi_t_,xi_dot_t_,xi_ddot_t_]=QuinticTimeScalingKinematics(xi0(j),xiT(j),xi_dot0(j),xi_dotT(j),xi_ddot0(j),xi_ddotT(j),Tf,t);
        xi_t(j) = xi_t_;
        xi_dot_t(j) = xi_dot_t_;
        xi_ddot_t(j) = xi_ddot_t_;
    end
    [px_t,vx_t,ax_t]=QuinticTimeScalingKinematics(px0,pxT,vx0,vxT,ax0,axT,Tf,t);
    [py_t,vy_t,ay_t]=QuinticTimeScalingKinematics(py0,pyT,vy0,vyT,ay0,ayT,Tf,t);
    [pz_t,vz_t,az_t]=QuinticTimeScalingKinematics(pz0,pzT,vz0,vzT,az0,azT,Tf,t);

    px = px_t;
    py = py_t;
    pz = pz_t;

    p = [px,py,pz]';
    V = zeros(6,1);
    Vdot = zeros(6,1);
    V(1) = vx_t;
    V(2) = vy_t;
    V(3) = vz_t;
    V(4:6) = dexp3(-xi_t)*xi_dot_t;
    Vdot(1) = ax_t;
    Vdot(2) = ay_t;
    Vdot(3) = az_t;
    Vdot(4:6) = dexp3(-xi_t)*xi_ddot_t+ ddexp3(-xi_t,-xi_dot_t)*xi_dot_t;
    R = R_start*MatrixExp3(VecToso3(xi_t));
    
    T = RpToTrans(R,p);
    X_traj{i} = T;
    V_traj{i}= V;
    Vdot_traj{i}= Vdot;

end

end


function [s,ds,dds]=QuinticTimeScalingKinematics(s0,sT,ds0,dsT,dds0,ddsT,Tf,t)
    x(1) = s0;
    x(2) = ds0;
    x(3) = dds0/2.0;
    x(4) = -(10*s0 - 10*sT + 2*Tf*(3*ds0 + 2*dsT) + (Tf^2*(3*dds0 - ddsT))/2)/Tf^3;
    x(5) = (((3*dds0)/2 - ddsT)*Tf^2 + (8*ds0 + 7*dsT)*Tf + 15*s0 - 15*sT)/Tf^4;
    x(6) = -(6*s0 - 6*sT + (Tf^2*(dds0 - ddsT))/2 + 3*Tf*(ds0 + dsT))/Tf^5;
    s = x(1)+x(2)*t+x(3)*t^2+x(4)*t^3+x(5)*t^4+x(6)*t^5;
    ds = x(2)+2*x(3)*t+3*x(4)*t^2+4*x(5)*t^3+5*x(6)*t^4;
    dds = 2*x(3)+2*3*x(4)*t+3*4*x(5)*t^2+4*5*x(6)*t^3;
end