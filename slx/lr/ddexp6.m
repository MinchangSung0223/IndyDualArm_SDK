function ddexp=ddexp6(lambda,lambda_dot)
    
    eta=lambda(1:3);
    xi=lambda(4:6);
    eta_dot=lambda_dot(1:3);
    xi_dot=lambda_dot(4:6);
    C = ddexp3(xi,xi_dot);
    C_dot = dddexp3(xi,xi_dot,eta,eta_dot);
    ddexp = [C C_dot ; zeros(3,3) C];
end