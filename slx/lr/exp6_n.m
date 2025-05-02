function [T,n]=exp6_n(lambda)
    xi = lambda(4:6);
    eta = lambda(1:3);
    [R,n] = exp3_n(xi);
    p = dexp3(xi)*eta;
    T = RpToTrans(R,p);
end