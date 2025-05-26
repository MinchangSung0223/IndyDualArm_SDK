function ddexpInv = ddexpInvso3(xi, xidot)
% @Brief: Derivative of inverse of differential of exponential map on SO(3)
% @Params: 
%   xi: An exponential coordinate (3 by 1 vector)
%   xidot: Its velocity  (3 by 1 vector)
% @Returns:
%   ddexpInv: The corresponding derivative of differential of the exponential
%   map, i.e., d/dt(dexp_xi^{-1}) (3 by 3 matrix)
%
% @Example: 
% clear; clc;
% xi = [0, 0, pi/3]';
% xidot = [1, -1, 2]';
% ddexpInv = ddexpInvso3(xi, xidot)
% Output:
% ddexpInv =
% 
%    -0.3623    1.0000    0.5889
%    -1.0000   -0.3623    0.4111
%    -0.4111   -0.5889         0

theta = norm(xi);


if norm(sin(theta)) < 1e-7
    if theta<1e-7
        ddexpInv = -1/2*VecToso3(xidot);
    else
        omg = VecToso3(xi);
        ddexpInv = -1/2*VecToso3(xidot)+1/12*omg*omg-1/360*xi'*xidot;
    end
    
else
    theta_2 = theta/2;
    s = sin(theta_2)/theta_2;
    c = cos(theta_2);
    
    beta = s*s;
    gamma = c/s;
    theta_sqr = theta*theta;
    xixidot = xi'*xidot;
    
    omg = VecToso3(xi);
    omgdot = VecToso3(xidot);
    omgomgdot = omg*omgdot + omgdot*omg;
    ddexpInv = -1/2 * omgdot ...
        + (1-gamma)/theta_sqr*omgomgdot ...
        + 1/theta_sqr*(1/beta+gamma-2)/theta_sqr*xixidot*omg*omg;
    
end
end