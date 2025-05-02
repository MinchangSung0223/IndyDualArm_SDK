% function ret=dddexp3(xi,xidot,xiddot)
%     xi_=vecToso3(xi);
%     xidot_=vecToso3(xidot);
%     xiddot_=vecToso3(xiddot);
%     n_xi = norm(xi);
%     n_xi2 = n_xi^2;
%     n_xi3 = n_xi^3;
%     n_xi4 = n_xi^4;
%     n_xi5 = n_xi^5;
%     n_xi6 = n_xi^6;
%     n_xi7 = n_xi^7;
%     sx = sin(n_xi);
%     cx = cos(n_xi);
%     xiTxidot=xi'*xidot;
%     xidotxidot_xiTxiddot=(xidot'*xidot+xi'*xiddot);
%     Omega_1_1 = (-2/n_xi4+2*cx/n_xi4+sx/n_xi3);
%     Omega_1_2 = (8/n_xi6-8/n_xi6*cx-5*sx/n_xi5+cx/n_xi4);
%     dOmega_1 = Omega_1_1*xiTxidot;
%     ddOmega_1 = Omega_1_2*(xiTxidot)^2+Omega_1_1*xidotxidot_xiTxiddot;
%     Omega_2_1 = (-2/n_xi4 + 3/n_xi5*sx-cx/n_xi4);
%     Omega_2_2 = (8/n_xi6-15*sx/n_xi7+7*cx/n_xi6+sx/n_xi5);
%     dOmega_2 = Omega_2_1*xiTxidot;
%     ddOmega_2 = Omega_2_2*(xiTxidot)^2+Omega_2_1*xidotxidot_xiTxiddot;
%     if abs(norm(xi))<1e-4
%         ret=1/2*(xiddot_)+1/3*(xidot_)*(xidot_);
%         return;
%     end
%     ret = ddOmega_1*xi_+2*dOmega_1*(xidot_)+(1/n_xi2-cx/n_xi2)*(xiddot_)...
%                 +ddOmega_2*xi_*xi_+2*dOmega_2*(xidot_*xi_+xi_*xidot_)+(1/n_xi2-sx/n_xi3)*(xiddot_*xi_+2*xidot_*xidot_+xi_*xiddot_);
% end
%
function dddexp=dddexp3(xi,dxi,y,dy)
    if norm(xi)<eps
        dddexp = 1/2*VecToso3(dy)+1/6*(VecToso3(dxi)*VecToso3(y)+VecToso3(y)*VecToso3(dxi));
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
    Gamma1 = (1-alpha)/norm_xi2;
    Gamma2 = (alpha-beta)/norm_xi2;
    Gamma3 = (beta/2-3*Gamma1)/norm_xi2;
    Gamma4 = -Gamma2/beta;
    Gamma5 = (Gamma1+2*Gamma2)/(norm_xi2*beta);
    delta0 = dxi'*y + xi'*dy;
    delta1 = xi'*y*ceil_dxi + xi'*dxi*ceil_y + (delta0-4*zeta)*ceil_xi;
    delta2 = xi'*y*ceil_xi_dxi+ xi'*dxi*ceil_xi_y+(delta0-5*zeta)*ceil_xi2;
    delta3 = xi'*y*ceil_xi_dxi+ xi'*dxi*ceil_xi_y+(delta0-3*zeta)*ceil_xi2;
    dCxi = beta/2*(ceil_dy-zeta*ceil_xi)+Gamma1*(ceil_dy_xi+ceil_y_dxi+zeta*ceil_xi)...
           +Gamma2*(delta1+zeta*ceil_xi2)+Gamma3*delta2;
    dddexp = dCxi;
end