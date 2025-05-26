function ret=dexpSo3(xi)
    xi_=VecToso3(xi);
    n_xi = norm(xi);
    n_xi2 = n_xi^2;
    n_xi3= n_xi^3;
    if abs(n_xi)<eps
        ret = eye(3);
        return;
    end
    ret = eye(3)+(1/n_xi2-cos(n_xi)/n_xi2)*xi_+(1/n_xi2-sin(n_xi)/n_xi3)*xi_*xi_;
end