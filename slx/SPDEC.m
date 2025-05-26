function qddot=SPDEC(q,qdot,T_d,V_d,V_dot_d,T,V,J,Jdot,Kp,Kd,dt)
Ttilde = TransInv(T)*Td;
lambda = log6(Ttilde);
V = J*qdot;
Vtilde = Adjoint(Ttilde)*V_d-V;
lambdadot = dlog6(lambda)*Vtilde;
Dinv = dlog6(lambda);
Ddot = ddexp6(lambda,lambdadot);
AdTilde = Adjoint(Ttilde);
D = dexp6(lambda);
Gamma1 = Kp*lambda+Kp*dt*Dinv*AdTilde*V_d+Kd*dt*Ddot*AdTilde*V_d+Kd*dt*D*AdTilde*ad(V_d)*AdTilde*V_d;
Gamma2 = Kd*dt*D*AdTilde*V_dotd-Kd*dt*D*Jdot-Kd*dt*Ddot*J-Kd*dt*D*AdTilde*ad(V_d)*J-Kp*dt*Dinv*J-Kd*Dinv*J);
qddot = (M+J'*D'*Kd*d*D*J)\(J'*D'*Gamma1+J'*D'*Gamma2*qdot-B*qdot-C*qdot);
end