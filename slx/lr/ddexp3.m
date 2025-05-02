function ddexp=ddexp3(xi,dxi)
    if norm(xi)<eps
        ddexp=1/2*VecToso3(dxi);
        return;
    end
    ceil_xi = VecToso3(xi);
    norm_xi = norm(xi);
    ceil_xi = VecToso3(xi);
    s = sin(norm_xi/2)/((norm_xi)/2);
    c  = cos(norm_xi/2);
    alpha = s*c;
    beta = s*s;
    eta = dxi;
    Cxi = beta/2*VecToso3(eta)+...
    (1-alpha)/norm_xi^2 * (VecToso3(eta)*VecToso3(xi)+VecToso3(xi)*VecToso3(eta))...
    +(alpha-beta)/norm_xi^2 *xi'*(eta)*VecToso3(xi)...
    -1/norm_xi^2*(3*(1-alpha)/norm_xi^2 - beta/2)*xi'*eta*ceil_xi*ceil_xi;
    ddexp = Cxi;
end