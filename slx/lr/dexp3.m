function dexp=dexp3(xi)
    if  norm(xi)<eps
        dexp = eye(3);
        return;
    end
    ceil_xi = VecToso3(xi);
    norm_xi = norm(xi);
    ceil_xi = VecToso3(xi);
    s = sin(norm_xi/2)/((norm_xi)/2);
    c  = cos(norm_xi/2);
    alpha = s*c;
    beta = s*s;
    dexp = eye(3) + beta/2*ceil_xi + (1-alpha)/norm_xi^2 *ceil_xi*ceil_xi;
end