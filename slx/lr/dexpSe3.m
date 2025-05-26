function ret=dexpSe3(lambda)
    eta = lambda(1:3);
    xi = lambda(4:6);
    xi_=VecToso3(xi);
    eta_=VecToso3(eta);
    O = zeros(3,3);
    A = [xi_,O;O,xi_];
    B = [O,eta_;O,O];
    dexp3_ = dexpSo3(xi);
    if abs(sin(norm(xi)))<1e-7
        ret = [dexp3_,ddexpSo3(xi, eta);O,dexp3_];
        return;
    end
     % [Gamma_1,dGamma_1,ddGamma_1]=getGamma1(xi,zeros(3,1),zeros(3,1));
     % [Gamma_2,dGamma_2,ddGamma_2]=getGamma2(xi,zeros(3,1),zeros(3,1));
     % [Gamma_3,dGamma_3,ddGamma_3]=getGamma3(xi,zeros(3,1),zeros(3,1));
     % [Gamma_4,dGamma_4,ddGamma_4]=getGamma4(xi,zeros(3,1),zeros(3,1));
    [Gamma_1,Gamma_2,Gamma_3,Gamma_4]=Gamma(norm(xi),0);
    ret = [dexp3_,O;O,dexp3_]+1/2*B+Gamma_1*(B*A+A*B)+Gamma_2*(B*A*A+A*A*B)+Gamma_3*(A*A*B*A)+Gamma_4*(A*B*A);
    
    I = eye(6);
    ret2 = [dexp3_,O;O,dexp3_]+1/2*B-1/2*I+Gamma_1*(B*A+A*B-I)+Gamma_2*(B*A*A+A*A*B-I)+Gamma_3*(A*A*B*A-I)+Gamma_4*(A*B*A-I)+Gamma_1*I+Gamma_2*I+Gamma_3*I+Gamma_4*I+1/2*I;
    ret_inv = inv([dexp3_,O;O,dexp3_])+2*(I+B)-1/Gamma_1*(I+(A*B+B*A))-1/Gamma_2*(I+(B*A*A+A*A*B))-1/Gamma_3*(I+(A*A*B*A))-1/Gamma_4*(I+(A*B*A))+1/(1/2+Gamma_1+Gamma_2+Gamma_3+Gamma_4)*I
end