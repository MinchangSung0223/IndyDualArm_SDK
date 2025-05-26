function ret=dddexpInvSe3(lambda,lambdadot,lambdaddot)
    dexpInvlambda= dexpInvSe3(lambda);
    ret = -2*dexpInvlambda*ddexpSe3(lambda,lambdadot)*ddexpInvSe3(lambda,lambdadot)-dexpInvlambda*dddexpSe3(lambda,lambdadot,lambdaddot)*dexpInvlambda;
end

