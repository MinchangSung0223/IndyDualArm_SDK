function dexpInv = dexpInvse3(lambda)
% @Brief: Inverse of differential of exponential map on se(3)
% @Params: 
%   lambda: A exponential coordinate, lambda = [eta', xi'] (6 by 1 vector)
% @Returns:
%   dexpInv: The corresponding inverse of differential of the exponential
%   map, dexp_lambda^{-1} (6 by 6 matrix)
%
% @Example: 
% clear; clc;
% lambda = [1, 0, 0, 0, 0, pi/3]';
% dexpInv = dexpInvse3(lambda)
% 
% Output:
% dexpInv =
% 
%     0.9069    0.5236         0         0         0    0.0889
%    -0.5236    0.9069         0         0         0    0.5000
%          0         0    1.0000    0.0889   -0.5000         0
%          0         0         0    0.9069    0.5236         0
%          0         0         0   -0.5236    0.9069         0
%          0         0         0         0         0    1.0000

eta = lambda(1:3);
xi = lambda(4:6);

dexpInv = zeros(6, 6);
dexpInv(1:3, 1:3) = dexpInvSo3(xi);
dexpInv(1:3, 4:6) = ddexpInvSo3(xi, eta);
dexpInv(4:6, 1:3) = zeros(3, 3);
dexpInv(4:6, 4:6) = dexpInv(1:3, 1:3);
end