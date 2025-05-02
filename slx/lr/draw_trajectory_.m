X_start = eye(4);
X_end = X_start*RpToTrans(eul2rotm([pi/2,pi/2,pi/2]),[0,0,0.1]');
V_start = zeros(6,1);
V_end= zeros(6,1);
Vdot_start = zeros(6,1);
Vdot_end= zeros(6,1);
N = 500
Tf = 5;
% [X_traj,V_traj,Vdot_traj]=LieScrewTrajectory(X_start,X_end,V_start,V_end,Vdot_start,Vdot_end,5,100);
[X_traj,V_traj,Vdot_traj]=LieDecoupledTrajectory(X_start,X_end,V_start,V_end,Vdot_start,Vdot_end,5,N);

Fext = -[1,1,1,1,1,1]';
Fdes = [0,0,0,0,0,0]';
dt =Tf/N;
x_state = zeros(12,1);
Avec = [1,1,1,2,2,2];
Dvec = [30,20,10,60,40,20];
Kvec = [100,100,100,200,200,200];

p_des_list = zeros(N,3);
F_err_list = zeros(N,6);
p_des_prime_list = zeros(N,3);
xi_des_list = zeros(N,3);
xi_des_prime_list=zeros(N,3);
eul_des_list = zeros(N,3);
eul_des_prime_list=zeros(N,3);
Ferr = zeros(6,1);
Ferr(1:3) = Fdes(1:3)-Fext(1:3);
Ferr(4:6) = R_des'*Fdes(4:6)-Fext(4:6);

[t_,x_dot] = ode45(@(t_,x_dot) SingleBodyDynamics(t_,x_state,Ferr,Avec,Dvec,Kvec),[0, dt] , x_state ); 
x_state = x_dot(end,:)';
t_list = linspace(0,Tf,N);
drawCount = 0;
figure(2);
subplot(3,4,[4,8])
for i =1:1:length(X_traj)
    if i>length(X_traj)/2-1
        Fext = Fext*0;
    end
    Ferr = zeros(6,1);
    Ferr(1:3) = Fdes(1:3)-Fext(1:3);
     T_des = X_traj{i};
    V_des = V_traj{i};
    [R_des,p_des]=TransToRp(T_des);

    Ferr(4:6) = R_des'*Fdes(4:6)-Fext(4:6);
    [t_,x_dot] = ode45(@(t_,x_dot) SingleBodyDynamics(t_,x_state,Ferr,Avec,Dvec,Kvec),[0, dt] , x_state ); 
    p = x_state(1:3);    
    xi = x_state(4:6);
    pdot = x_state(7:9);
    xidot = x_state(10:12);
    x_state = x_dot(end,:)';
    pddot=(diag(Avec(1:3)))\(Ferr(1:3)-diag(Dvec(1:3))*pdot - diag(Kvec(1:3))*p);
    xiddot=(diag(Avec(4:6)))\(Ferr(4:6)-diag(Dvec(4:6))*xidot - diag(Kvec(4:6))*p);
    lambda = [eta;xi];
    lambdadot = [etadot;xidot];
    lambdaddot = [etaddot;xiddot];
   
    Vdot_des = Vdot_traj{i};
    T_tilde_prime = exp6(lambda);
    V_tilde_prime = dexp6(lambda)*lambdadot;
    Vdot_tilde_prime = ddexp6(lambda,lambdadot)*lambdadot+dexp6(lambda)*lambdaddot;
    T_des_prime = T_des*T_tilde_prime;
    V_des_prime = Adjoint(TransInv(T_tilde_prime))*(V_tilde_prime+V_des);
    Vdot_des_prime=Adjoint(TransInv(T_tilde_prime))*(Vdot_tilde_prime-ad(V_tilde_prime)*V_des_prime+Vdot_des);
    [R_des_prime,p_des_prime]=TransToRp(T_des_prime);
    
    eta = dlog3(xi)*p;
    etadot = ddlog3(xi,xidot)*p+dlog3(xi)*pdot;
    etaddot = dddexpInvSo3(xi,xidot,xiddot)*p+2*ddlog3(xi,xidot)*pdot+dlog3(xi)*pddot;


    p_des_list(i,:) = p';
    p_des_prime_list(i,:) = p_des_prime';
    xi_des_list(i,:) = xi';
    xi_des_prime_list(i,:) = log3(R_des_prime)';
    F_err_list(i,:)  = Ferr;

    eul_des_list(i,:) = rotm2eul(R_des);
    eul_des_prime_list(i,:) = rotm2eul(R_des_prime);
    if drawCount>10
    handles = drawAxisHandleColor(X_traj{i},0.01,0.5,[0,0,0]); hold on;
    handles2 = drawAxisHandleColor(T_des_prime,0.01,0.5,[1,0,0]); hold on;    
    daspect([1,1,1])
     grid on;
     drawnow;
    drawCount = 0;
    end
    drawCount = drawCount+1;
end
xlabel("$x[m]$","Interpreter","latex")
ylabel("$y[m]$","Interpreter","latex")
zlabel("$z[m]$","Interpreter","latex")
drawAxis(X_traj{1},0.02,1)
drawAxis(X_traj{end},0.02,1)
xlim([0 0.02])
ylim([0 0.02])
view(-160,20)
 legend("","","","","","$T_d$","","","","$T_{d,adp}$","Interpreter","latex")


subplot(3,4,1)
plot(t_list,p_des_list(:,1),'k')
ylabel("$\tilde{p}_x[m]$","Interpreter","latex")
xlabel("Time$[s]$","Interpreter","latex")
yline(0,"k--")
xline(Tf/2,"k:")
yticks([0  0.005 0.01])
axis padded
xlim([0 5])
subplot(3,4,2)
plot(t_list,p_des_list(:,2),'k')
ylabel("$\tilde{p}_y[m]$","Interpreter","latex")
xlabel("Time$[s]$","Interpreter","latex")
yline(0,"k--")
xline(Tf/2,"k:")
yticks([0  0.005 0.01])
axis padded
xlim([0 5])
subplot(3,4,3)
plot(t_list,p_des_list(:,3),'k')
ylabel("$\tilde{p}_z[m]$","Interpreter","latex")
xlabel("Time$[s]$","Interpreter","latex")
yline(0,"k--")
xline(Tf/2,"k:")
yticks([0 0.01])
axis padded
xlim([0 5])

subplot(3,4,5)
plot(t_list,xi_des_list(:,1),'k')
ylabel("$\xi_1[rad]$","Interpreter","latex")
xlabel("Time$[s]$","Interpreter","latex")
yline(0,"k--")
xline(Tf/2,"k:")
yticks([0  0.005 0.01])
axis padded
xlim([0 5])
subplot(3,4,6)
plot(t_list,xi_des_list(:,2),'k')
ylabel("$\xi_2[rad]$","Interpreter","latex")
xlabel("Time$[s]$","Interpreter","latex")
yline(0,"k--")
xline(Tf/2,"k:")
yticks([0  0.005 0.01])
axis padded
xlim([0 5])
subplot(3,4,7)
plot(t_list,xi_des_list(:,3),'k')
ylabel("$\xi_3[rad]$","Interpreter","latex")
xlabel("Time$[s]$","Interpreter","latex")
yline(0,"k--")
xline(Tf/2,"k:")
yticks([0  0.005 0.01])
axis padded
xlim([0 5])
subplot(3,4,[9 10 11 12]);
axis off;
title("\quad\quad\quad\quad\quad(a) Position and orientation errors~~~~\quad\quad\quad\qquad\qquad\qquad\qquad(b) 3D trajectory plot","Interpreter","latex")
fig = figure(2);
exportgraphics(fig, 'myFigure.pdf', 'ContentType','vector');