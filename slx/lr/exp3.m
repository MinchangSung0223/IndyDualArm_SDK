function R=exp3(xi)
    if  norm(xi)<eps
        R = eye(3);
        return;
    end
    norm_xi = norm(xi);
    ceil_xi = VecToso3(xi);
    s = sin(norm_xi/2)/(norm_xi/2);
    c  = cos(norm_xi/2);
    alpha = s*c;
    beta = s^2;
    R = eye(3) + alpha*ceil_xi + beta/2*ceil_xi*ceil_xi;
end