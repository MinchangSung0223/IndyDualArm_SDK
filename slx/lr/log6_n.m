function lambda = log6_n(T,n)
    [R,p] = TransToRp(T);
    xi = log3_n(T(1:3,1:3),n);
    eta = dlog3(xi)*p;
    lambda = [eta;xi];
end