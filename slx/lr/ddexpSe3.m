% function ddexp = ddexpse3(lambda, lambdot)
% % @Brief: Derivative of differential of exponential map on SE(3)
% % @Params: 
% %   lambda: An exponential coordinate (6 by 1 vector)
% %   lambdot: Its velocity  (6 by 1 vector)
% % @Returns:
% %   ddexp: The corresponding derivative of differential of the exponential
% %   map, i.e., d/dt(dexp_lambda) (6 by 6 matrix)
% %
% % @Example: 
% % clear; clc;
% % lambda = [1, 0, 0, 0, 0, pi/3]';
% % lambdot = [0, 1, 3, 1, -1, 2]';
% % ddexp = ddexpse3(lambda, lambdot)
% % Output:
% % ddexp =
% % 
% %    -0.6245   -0.7421   -0.2907   -0.9194   -1.1898    0.7368
% %     0.7421   -0.6245   -0.6212    0.8743   -1.2349    0.3274
% %     0.6212    0.2907         0   -0.1751    0.0031   -0.3155
% %          0         0         0   -0.6245   -0.7421   -0.2907
% %          0         0         0    0.7421   -0.6245   -0.6212
% %          0         0         0    0.6212    0.2907         0
% 
% eta = lambda(1:3);
% etadot = lambdot(1:3);
% xi = lambda(4:6);
% xidot = lambdot(4:6);
% 
% ddexp = zeros(6, 6);
% ddexp(1:3, 1:3) = ddexpSo3(xi, xidot);
% ddexp(4:6, 1:3) = zeros(3, 3);
% ddexp(4:6, 4:6) = ddexp(1:3, 1:3);
% 
% theta = norm(xi);
% if theta <1e-12
%     eta_c = VecToso3(eta);
%     xidot_c = VecToso3(xidot);
%     ddexp(1:3, 4:6) = 1/2*VecToso3(etadot) ...
%                     + 1/6*(eta_c*xidot_c + xidot_c*eta_c);  
% else
%     theta_2 = theta/2;
%     s = sin(theta_2)/theta_2;
%     c = cos(theta_2);
%     alpha = s*c;
%     beta = s*s;
%     theta_sqr = theta*theta;
% 
%     tmp1 = (1-alpha)/theta_sqr;
%     tmp4 = (xi'*eta)*(xi'*xidot)/theta_sqr;
% 
%     xi_c = VecToso3(xi);
%     xi_c2 = xi_c*xi_c;
%     xidot_c = VecToso3(xidot);
%     eta_c = VecToso3(eta);
%     etadot_c = VecToso3(etadot);
% 
%     xi_etadot_c = xi_c*etadot_c + etadot_c*xi_c;
%     eta_xidot_c = eta_c*xidot_c + xidot_c*eta_c;
%     xi_xidot_c = xi_c*xidot_c + xidot_c*xi_c;
%     xi_eta_c = xi_c*eta_c + eta_c*xi_c;
% 
%     ddexp(1:3, 4:6) = beta/2*(etadot_c-tmp4*xi_c)...
%         + tmp1*(xi_etadot_c  + eta_xidot_c + tmp4*xi_c)...
%         + (alpha-beta)/theta_sqr*(xi'*eta*xidot_c + (xidot'*eta + xi'*etadot - 4*tmp4)*xi_c + xi'*xidot*eta_c + tmp4*xi_c2)...
%         + 1/theta_sqr*(beta/2-3*tmp1)*(xi'*eta*xi_xidot_c + (xidot'*eta+xi'*etadot -5*tmp4)*xi_c2 + xi'*xidot*xi_eta_c);
% 
% end
% 
% end


function ret=ddexpSe3(lambda,lambdadot)

    eta = lambda(1:3);
    xi = lambda(4:6);
    etadot = lambdadot(1:3);
    xidot = lambdadot(4:6);
    A=VecToso3(xi);
    B=VecToso3(eta);
    Adot=VecToso3(xidot);
    Bdot=VecToso3(etadot);

%% 
    % [Gamma_1,Gamma_2,Gamma_3,Gamma_4]=Gamma(norm(xi),0);
    % [dGamma_1,dGamma_2,dGamma_3,dGamma_4]=Gamma(norm(xi),1);
    x=norm(xi);
    xn = x .^ (1:6);
    sx = sin(x);
    cx = cos(x);
    Gamma_1=(x - sx)/xn(3);
    Gamma_2=(2*cx + xn(2) - 2)/(2*xn(4));
    dGamma_1=-(2*x - 3*sx + x*cx)/xn(4);
    ddGamma_1=(6*x - 12*sx + xn(2)*sx+ 6*x*cx)/xn(5);
    ddGamma_2=(20*cx - xn(2)*cx + 8*x*sx+ 3*xn(2) - 20)/xn(6);
    dGamma_2=-(4*cx + x*sx + xn(2)- 4)/xn(5);
    Gamma_3= - dGamma_1/x;
    Gamma_4= dGamma_2*x+Gamma_2;
    dGamma_3 = -ddGamma_1/x+dGamma_1/xn(2);
    dGamma_4 =ddGamma_2*x+2*dGamma_2;


%%     
    AB=  A*B;
    BA=  B*A;
    AA = A*A;
    dAB=Adot*B+A*Bdot;
    dBA = Bdot*A+B*Adot;
    dAA=Adot*A+A*Adot;

    AB_BA = AB+BA;
    dAB_BA = (dAB)+(dBA);

    ABA = AB*A;
    dABA = dAB*A+AB*Adot;

    AABA = AA*BA;
    dAABA = dAA*BA+AA*dBA;

    AAB_BAA = A*AB+BA*A;
    dAAB_BAA = Adot*AB+A*dAB+dBA*A+BA*Adot;
    normxidot = xi'*xidot/norm(xi);
%% 
    ddexp3_=ddexpSo3(xi,xidot);    
    if norm(xi)<eps
        lim_Gamma_1 = 1/6;
        lim_Gamma_2 = 1/24;
        C = 1/2*Bdot...
                                + lim_Gamma_1*dAB_BA...
                                + lim_Gamma_2*dAAB_BAA;
        ret = [ddexp3_,C;zeros(3,3),ddexp3_];
        return;
    end


    C = 1/2*Bdot...
                            + dGamma_1*normxidot*(AB_BA)     + Gamma_1*dAB_BA...
                            + dGamma_2*normxidot*(AAB_BAA)   + Gamma_2*dAAB_BAA...
                            + dGamma_3*normxidot*(AABA)      + Gamma_3*dAABA...
                            + dGamma_4*normxidot*(ABA)       + Gamma_4*dABA;
    ret = [ddexp3_,C;zeros(3,3),ddexp3_];

end