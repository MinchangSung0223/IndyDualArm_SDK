function ddlog=ddlog6(lambda,lambda_dot)
    dlog6mat = dlog6(lambda);
    ddlog = dlog6mat*ddexp6(lambda,lambda_dot)*dlog6mat;
end
