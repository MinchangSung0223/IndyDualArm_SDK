function dlog = dlog6(lambda)  
    eta = reshape(lambda(1:3),3,1);
    xi = reshape(lambda(4:6),3,1);
    
    norm_xi = norm(xi);
    ceil_xi = VecToso3(xi);
    s = sin(norm_xi/2)/((norm_xi)/2);
    c  = cos(norm_xi/2);
    alpha = s*c;
    beta = s*s;
    gamma = alpha/beta;    

    dlog_3 = dlog3(xi);
    O = zeros(3,3);
    norm_xixi = norm(xi)^2;
    D = -1/2*VecToso3(eta) + (1-gamma)/norm_xixi*(VecToso3(eta)*VecToso3(xi)+VecToso3(xi)*VecToso3(eta))+1/norm_xixi*(1/beta+gamma-2)/norm_xixi*xi'*eta*VecToso3(xi)*VecToso3(xi);
    if norm(xi)<eps
        D = -1/2*VecToso3(eta);
    end
    dlog = [dlog_3 D ; O dlog_3];
end