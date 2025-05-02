function dlog = dlog3(xi)
    if norm(xi)<eps
        dlog = eye(3);
        return;
    end
    % dlog = dexp^-1
    norm_xi = norm(xi);
    ceil_xi = VecToso3(xi);
    s = sin(norm_xi/2)/((norm_xi)/2);
    c  = cos(norm_xi/2);
    alpha = s*c;
    beta = s*s;
    gamma = alpha/beta;
    dlog = eye(3) - 1/2*VecToso3(xi)+ (1-gamma)/norm(xi)^2*VecToso3(xi)^2;
end