function dexpInv = dexpInvSo3(xi)
% @Brief: Inverse of differential of exponential map on so(3)
% @Params: 
%   xi: An exponential coordiant (3 by 1 vector)
% @Returns:
%   R: The corresponding inverse of differential of the exponential map
%   dexp_xi^{-1} (3 by 3 matrix)
%
% @Example: 
% clear; clc;
% xi = [0; 0; pi/3];
% dexp = dexpInvso3(xi)
% 
% Output:
% 
% dexp =
%     0.9069    0.5236         0
%    -0.5236    0.9069         0
%          0         0    1.0000


theta = norm(xi);
if abs(sin(norm(xi)))<1e-3
    
        
    if abs(norm(xi)-2*pi)<1e-3
        omg = VecToso3(xi);
        dexpInv = eye(3)- 1/2*omg+1/12*omg*omg;
    else
        dexpInv = eye(3);
    end
    
else
    theta_2 = theta/2;
    s = sin(theta_2)/theta_2;
    c = cos(theta_2);
    gamma = c/s;
    
    omg = VecToso3(xi);
    dexpInv = eye(3) - 1/2*omg + (1-gamma)/theta^2 * omg*omg;
end
end