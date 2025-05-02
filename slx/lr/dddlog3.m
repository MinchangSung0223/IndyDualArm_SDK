% function ret = dddexpInvSo3(xi, xidot,xiddot)
%     dexpInvxi= dlog3(xi);
%     ret = -2*dexpInvxi*ddlog3(xi,xidot)*ddlog3(xi,xidot)-dexpInvxi*dddexp3(xi,xidot,xiddot)*dexpInvxi;
% end
function dddlog=dddlog3(xi,dxi,y,dy)
    if norm(xi)<eps
        dddlog = -1/2*VecToso3(dy)+1/12*(VecToso3(dxi)*VecToso3(y)+VecToso3(y)*VecToso3(dxi));
        return;
    end
    ceil_xi = VecToso3(xi);
    norm_xi = norm(xi);
    norm_xi2 = norm_xi*norm_xi;
    ceil_dxi = VecToso3(dxi);
    s = sin(norm_xi/2)/((norm_xi)/2);
    c  = cos(norm_xi/2);
    alpha = s*c;
    beta = s*s;
    ceil_dy = VecToso3(dy);
    ceil_y = VecToso3(y);
    ceil_dy_xi = (ceil_dy*ceil_xi+ceil_xi*ceil_dy);
    ceil_y_dxi = (ceil_y*ceil_dxi+ceil_dxi*ceil_y);
    ceil_xi_y =  (ceil_xi*ceil_y+ceil_y*ceil_xi);
    ceil_xi_dxi = (ceil_xi*ceil_dxi+ceil_dxi*ceil_xi);
    ceil_xi2 = ceil_xi*ceil_xi;

    zeta = xi'*y*xi'*dxi/norm_xi2;
    gamma = alpha/beta;
    Gamma1 = (1-alpha)/norm_xi2;
    Gamma2 = (alpha-beta)/norm_xi2;
    Gamma3 = (beta/2-3*Gamma1)/norm_xi2;
    Gamma4 = -Gamma2/beta;
    Gamma5 = (Gamma1+2*Gamma2)/(norm_xi2*beta);
    delta0 = dxi'*y + xi'*dy;
    delta1 = xi'*y*ceil_dxi + xi'*dxi*ceil_y + (delta0-4*zeta)*ceil_xi;
    delta2 = xi'*y*ceil_xi_dxi+ xi'*dxi*ceil_xi_y+(delta0-5*zeta)*ceil_xi2;
    delta3 = xi'*y*ceil_xi_dxi+ xi'*dxi*ceil_xi_y+(delta0-3*zeta)*ceil_xi2;
    dDxi = -1/2*VecToso3(dy) +2/norm_xi2*(1-gamma/beta)/norm_xi2*zeta*ceil_xi2...
        +Gamma4*(ceil_dy_xi+ceil_y_dxi)+Gamma5*delta3;
    dddlog = dDxi;
end