function ret=dexp4(lambda)
xi1 = lambda(4);
xi2 = lambda(5);
xi3 = lambda(6);
eta1= lambda(1);
eta2 = lambda(2);
eta3 = lambda(3);
xi = [xi1,xi2,xi3]';
eta = [eta1,eta2,eta3]';
theta = norm([xi1,xi2,xi3]);
if theta<eps
    Omega_1 = 1/2;
    Omega_2 = 1/6;
    Gamma_1 = 1/6;
    Gamma_2 = 1/24;
    Gamma_3 = 1/60;
    Gamma_4 = 1/24;
    
else
    Omega_1 = (1-cos(theta))/theta^2;
    Omega_2 = (theta-sin(theta))/theta^3;
    Gamma_1 = (theta-sin(theta))/theta^3;
    Gamma_2 = (2*cos(theta)-2+theta^2)/(2*theta^4);
    
    Gamma_3 = (theta*cos(theta)+2*theta-3*sin(theta))/theta^5;
    Gamma_4 = (-6*cos(theta)-2*theta*sin(theta)-theta*theta+6)/(2*theta^4);
end
A = VecTose3([0;0;0;xi]);
B = VecTose3([eta;0;0;0]);

ret = [dexp3(xi),zeros(3,1);0,0,0,1]+1/2*B...
    +Gamma_1*(A*B+B*A)+...
    Gamma_2*(A*A*B+B*A*A)+...
    Gamma_3*(A*A*B*A)+...
    Gamma_4*(A*B*A);

end