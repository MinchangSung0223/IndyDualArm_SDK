function ret=dddexpSo3(xi,xidot,xiddot)
    xi_=VecToso3(xi);
    xidot_=VecToso3(xidot);
    xiddot_=VecToso3(xiddot);
    n_xi = norm(xi);
    n_xi2 = n_xi^2;
    n_xi3 = n_xi^3;
    n_xi4 = n_xi^4;
    n_xi5 = n_xi^5;
    n_xi6 = n_xi^6;
    n_xi7 = n_xi^7;
    sx = sin(n_xi);
    cx = cos(n_xi);
    xiTxidot=xi'*xidot;
    xidotxidot_xiTxiddot=(xidot'*xidot+xi'*xiddot);
    Omega_1_1 = (-2/n_xi4+2*cx/n_xi4+sx/n_xi3);
    Omega_1_2 = (8/n_xi6-8/n_xi6*cx-5*sx/n_xi5+cx/n_xi4);
    dOmega_1 = Omega_1_1*xiTxidot;
    ddOmega_1 = Omega_1_2*(xiTxidot)^2+Omega_1_1*xidotxidot_xiTxiddot;
    Omega_2_1 = (-2/n_xi4 + 3/n_xi5*sx-cx/n_xi4);
    Omega_2_2 = (8/n_xi6-15*sx/n_xi7+7*cx/n_xi6+sx/n_xi5);
    dOmega_2 = Omega_2_1*xiTxidot;
    ddOmega_2 = Omega_2_2*(xiTxidot)^2+Omega_2_1*xidotxidot_xiTxiddot;
    if abs(norm(xi))<1e-4
        ret=1/2*(xiddot_)+1/3*(xidot_)*(xidot_);
        return;
    end
    ret = ddOmega_1*xi_+2*dOmega_1*(xidot_)+(1/n_xi2-cx/n_xi2)*(xiddot_)...
                +ddOmega_2*xi_*xi_+2*dOmega_2*(xidot_*xi_+xi_*xidot_)+(1/n_xi2-sx/n_xi3)*(xiddot_*xi_+2*xidot_*xidot_+xi_*xiddot_);
end