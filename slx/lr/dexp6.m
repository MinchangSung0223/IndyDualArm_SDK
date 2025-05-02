function dexp=dexp6(lambda)
    eta = lambda(1:3);
    ceil_eta = VecToso3(eta);
    
    
    xi = lambda(4:6);
    norm_xi = norm(xi);
    ceil_xi = VecToso3(xi);
    s = sin(norm_xi/2)/(norm_xi/2);
    c  = cos(norm_xi/2);
    alpha = s*c;
    beta = s^2;

    Cxi = beta/2*VecToso3(eta)+(1-alpha)/norm_xi^2 * (VecToso3(eta)*VecToso3(xi)+VecToso3(xi)*VecToso3(eta))+(alpha-beta)/norm_xi^2 *xi'*(eta)*VecToso3(xi)-1/norm_xi^2*(3*(1-alpha)/norm_xi^2 - beta/2)*xi'*eta*ceil_xi*ceil_xi;
    if norm(xi)<eps
        Cxi = 1/2*VecToso3(eta);
    end
    dexp = [dexp3(xi) Cxi; zeros(3,3) , dexp3(xi)];
end