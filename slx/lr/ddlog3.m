function ddlog=ddlog3(xi,dxi)
    if norm(xi)<eps
        ddlog=-1/2*VecToso3(dxi);
        return;
    end
    norm_xi = norm(xi);
    norm_xi2 = norm_xi^2;
    ceil_xi = VecToso3(xi);
    s = sin(norm_xi/2)/((norm_xi)/2);
    c  = cos(norm_xi/2);
    alpha = s*c;
    beta = s*s;
    gamma = alpha/beta;
    D = -1/2*VecToso3(dxi) +(1-gamma)/norm_xi2*(skew_sum(dxi,xi))+1/norm_xi2*(1/beta+gamma-2)/norm_xi2*xi'*dxi*VecToso3(xi)*VecToso3(xi);
    ddlog = D;
end
function ret=skew_sum(a,b)
    ret = VecToso3(a)*VecToso3(b)+VecToso3(b)*VecToso3(a);
end