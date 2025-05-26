function lambda = log6(T)
    se3mat = logm(T);
    lambda = [se3mat(1: 3, 4);se3mat(3, 2); se3mat(1, 3); se3mat(2, 1)];
end
