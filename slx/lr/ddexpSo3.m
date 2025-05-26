
function ddexp = ddexpSo3(xi, xidot)
% @Brief: Derivative of differential of exponential map on SO(3)
% @Params: 
%   xi: An exponential coordinate (3 by 1 vector)
%   xidot: Its velocity  (3 by 1 vector)
% @Returns:
%   ddexp: The corresponding derivative of differential of the exponential
%   map, i.e., d/dt(dexp_xi) (3 by 3 matrix)
%
% @Example: 
% clear; clc;
% xi = [0, 0, pi/3]';
% xidot = [1, -1, 2]';
% ddexp = ddexpso3(xi, xidot)
% Output:
% ddexp =
% 
%    -0.6245   -0.7421   -0.2907
%     0.7421   -0.6245   -0.6212
%     0.6212    0.2907         0

theta = norm(xi);
if theta < 1e-12
    ddexp = 1/2*VecToso3(xidot); % beta_2 = 1/2 when theta=0
    
else
    theta_2 = theta/2;
    s = sin(theta_2)/theta_2;
    c = cos(theta_2);
    
    alpha = s*c;
    beta = s*s;
    beta_2 = beta/2;
    theta_sqr = theta*theta;
    
    tmp = (1-alpha)/theta_sqr;
    xixidot = xi'*xidot;
    
    omg = VecToso3(xi);
    omgdot = VecToso3(xidot);
    omgomgdot = omg*omgdot + omgdot*omg;
    ddexp = beta_2 * omgdot ...
        + tmp * omgomgdot ...
        + (alpha-beta)/theta_sqr*xixidot*omg ...
        + (beta_2 - 3*tmp)/theta_sqr*xixidot*omg*omg;
    
end
end
% function ret=ddexpSo3(xi,xidot)
%     xi_=VecToso3(xi);
%     xidot_=VecToso3(xidot);
%     n_xi = norm(xi);
%     n_xi2 = n_xi^2;
%     n_xi3 = n_xi^3;
%     n_xi4 = n_xi^4;
%     n_xi5 = n_xi^5;
%     sx = sin(n_xi);
%     cx = cos(n_xi);
%     xiTxidot = xi'*xidot;
%     if norm(xi)<1e-7
%         ret= 1/2.0*(xidot_);
%         return;
%     end
%     ret = (-2/n_xi4+2*cx/n_xi4+sx/n_xi3)*xiTxidot*xi_+( 1/n_xi2-cx/n_xi2)*(xidot_)...
%                 +(-2/n_xi4 + 3/n_xi5*sx-cx/n_xi4)*xiTxidot*xi_*xi_+(1/n_xi2-sx/n_xi3)*(xidot_*xi_+xi_*xidot_);
% end