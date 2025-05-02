function [R,n]=exp3_n(xi)
    n=0;

    if  norm(xi)<eps
        R = eye(3);
        return;
    end
    if norm(xi)>pi
        n = fix(norm(xi)/pi);
    end
    xihat = xi/norm(xi);
    norm_xi = norm(xi)-n*pi;
    xi = xihat*norm_xi;
    ceil_xi = VecToso3(xi);
    s = sin(norm_xi/2)/(norm_xi/2);
    c  = cos(norm_xi/2);
    alpha = s*c;
    beta = s^2;
    R = eye(3) + alpha*ceil_xi + beta/2*ceil_xi*ceil_xi;
end