function lambda = log6(T)
    % [R,p] = TransToRp(T);
    % xi = log3(T(1:3,1:3));
    % eta = dlog3(xi)*p;
    % lambda = [eta;xi];

    lambda = se3ToVec(MatrixLog6(T));
end