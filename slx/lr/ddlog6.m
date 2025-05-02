function ddlog=ddlog6(lambda,lambda_dot)
    xi = lambda(4:6);
    xi_dot = lambda_dot(4:6);
    eta = lambda(1:3);
    eta_dot= lambda_dot(1:3);
    
    D = ddlog3(xi,xi_dot);
    D_dot = dddlog3(xi,xi_dot,eta,eta_dot);
    ddlog = [D,D_dot; zeros(3,3),D];
end