function ddexpInv = ddexpInvse3(lambda, lambdot)
% @Brief: Derivative of inverse of differential of exponential map on SE(3)
% @Params: 
%   lambda: An exponential coordinate (6 by 1 vector)
%   lambdot: Its velocity  (6 by 1 vector)
% @Returns:
%   ddexpInv: The corresponding derivative of differential of the
%   exponential map, i.e., d/dt(dexp_lambda^{-1}) (6 by 6 matrix)
%
% @Example: 
% clear; clc;
% lambda = [1, 0, 0, 0, 0, pi/3]';
% lambdot = [0, 1, 3, 1, -1, 2]';
% ddexpInv = ddexpInvse3(lambda, lambdot)
% Output:
% ddexpInv =
% 
%    -0.3623    1.0000    0.5889   -0.5467    1.4151   -0.3238
%    -1.0000   -0.3623    0.4111   -1.5849   -0.7165    0.0889
%    -0.4111   -0.5889         0    0.6762    0.0889   -0.1698
%          0         0         0   -0.3623    1.0000    0.5889
%          0         0         0   -1.0000   -0.3623    0.4111
%          0         0         0   -0.4111   -0.5889         0



eta = lambda(1:3);
etadot = lambdot(1:3);
xi = lambda(4:6);
xidot = lambdot(4:6);

ddexpInv = zeros(6, 6);
ddexpInv(1:3, 1:3) = ddexpInvSo3(xi, xidot);
ddexpInv(4:6, 1:3) = zeros(3, 3);
ddexpInv(4:6, 4:6) = ddexpInv(1:3, 1:3);

theta = norm(xi);
if theta <1e-12
    eta_c = VecToso3(eta);
    xidot_c = VecToso3(xidot);
    ddexpInv(1:3, 4:6) = -1/2*VecToso3(etadot) ...
                    + 1/12*(eta_c*xidot_c + xidot_c*eta_c);  
else
    theta_2 = theta/2;
    s = sin(theta_2)/theta_2;
    c = cos(theta_2);
    
    beta = s*s;
    gamma = c/s;
    theta_sqr = theta*theta;
    
    tmp = (xi'*eta)*(xi'*xidot)/theta_sqr;
    
    xi_c = VecToso3(xi);
    xidot_c = VecToso3(xidot);
    eta_c = VecToso3(eta);
    etadot_c = VecToso3(etadot);
    
    xi_etadot_c = xi_c*etadot_c + etadot_c*xi_c;
    eta_xidot_c = eta_c*xidot_c + xidot_c*eta_c;
    xi_xidot_c = xi_c*xidot_c + xidot_c*xi_c;
    xi_eta_c = xi_c*eta_c + eta_c*xi_c;
    
    ddexpInv(1:3, 4:6) = -1/2*etadot_c ...
        + (1-gamma)/theta_sqr*(xi_etadot_c + eta_xidot_c)...
        + 1/theta_sqr*(1/beta + gamma -2)/theta_sqr*(xi'*eta*xi_xidot_c + (xidot'*eta+xi'*etadot-3*tmp)*xi_c*xi_c + xi'*xidot*xi_eta_c)...
        + 2/theta_sqr*(1-gamma/beta)/theta_sqr*tmp*xi_c*xi_c;

end

end