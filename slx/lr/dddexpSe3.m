function ret=dddexpSe3(lambda,lambdadot,lambdaddot)
    eta = lambda(1:3);
    xi = lambda(4:6);
    etadot = lambdadot(1:3);
    xidot = lambdadot(4:6);
    etaddot = lambdaddot(1:3);
    xiddot = lambdaddot(4:6);

    xi_=VecToso3(xi);
    eta_=VecToso3(eta);
    xidot_=VecToso3(xidot);
    etadot_=VecToso3(etadot);
    xiddot_=VecToso3(xiddot);
    etaddot_=VecToso3(etaddot);

    O = zeros(3,3);
    A = xi_;
    B = eta_;
    Adot = xidot_;
    Bdot = etadot_;
    Addot = xiddot_;
    Bddot = etaddot_;

    % [Gamma_1_,Gamma_2_,Gamma_3_,Gamma_4_]=Gamma(norm(xi),0);
    % [dGamma_1_,dGamma_2_,dGamma_3_,dGamma_4_]=Gamma(norm(xi),1);
    % [ddGamma_1_,ddGamma_2_,ddGamma_3_,ddGamma_4_]=Gamma(norm(xi),2);
        x=norm(xi);
        xn = x .^ (1:7);
        sx = sin(x);
        cx = cos(x);
        xcx = x*cx;
        xsx = x*sx;
        xn2cx = x*xcx;
        xn2sx = x*xsx;

        Gamma_1= (x - sx)/xn(3);
        dGamma_1=-(2*x - 3*sx + x*cx)/xn(4);
        ddGamma_1=(6*x - 12*sx + xn2sx+ 6*xcx)/xn(5);
        dddGamma_1= -(24*x - 60*sx - xn(3)*cx + 9*xn2sx + 36*xcx)/xn(6);

        Gamma_2=(2*cx + xn(2) - 2)/(2*xn(4));
        dGamma_2=-(4*cx + x*sx + xn(2)- 4)/xn(5);
        ddGamma_2=(20*cx - xn2cx + 8*xsx+ 3*xn(2) - 20)/xn(6);
        dddGamma_2=-(120*cx - 12*xn2cx - xn(3)*sx + 60*xsx + 12*xn(2) - 120)/xn(7);

        Gamma_3= - dGamma_1/x;
        dGamma_3 = -ddGamma_1/x+dGamma_1/xn(2);
        ddGamma_3 = -dddGamma_1/x+2*ddGamma_1/xn(2)-2*dGamma_1/xn(3);

        Gamma_4= dGamma_2*x+Gamma_2;        
        dGamma_4 =ddGamma_2*x+2*dGamma_2;
        ddGamma_4 =dddGamma_2*x+3*ddGamma_2;
        

        % Gamma_1 = 1/x^2 - sin(x)/x^3;
        % dGamma_1 = -(2*x - 3*sin(x) + x*cos(x))/x^4;
        % ddGamma_1 = (6*x - 12*sin(x) + x^2*sin(x) + 6*x*cos(x))/x^5;
        % dddGamma_1 = -(24*x - 60*sin(x) - x^3*cos(x) + 9*x^2*sin(x) + 36*x*cos(x))/x^6;
        % 
        % Gamma_2 = (2*cos(x) + x^2 - 2)/(2*x^4);
        % dGamma_2 = -(4*cos(x) + x*sin(x) + x^2 - 4)/x^5;
        % ddGamma_2 = (20*cos(x) - x^2*cos(x) + 8*x*sin(x) + 3*x^2 - 20)/x^6;
        % dddGamma_2 = -(120*cos(x) - 12*x^2*cos(x) - x^3*sin(x) + 60*x*sin(x) + 12*x^2 - 120)/x^7;
        % 
        % Gamma_3=(2*x - 3*sin(x) + x*cos(x))/x^5;
        % dGamma_3 = -ddGamma_1/x+dGamma_1/xn(2);
        % ddGamma_3 = -dddGamma_1/x+2*ddGamma_1/xn(2)-2*dGamma_1/xn(3);
        % 
        % Gamma_4=-(6*cos(x) + 2*x*sin(x) + x^2 - 6)/(2*x^4);
        % dGamma_4=(12*cos(x) - x^2*cos(x) + 6*x*sin(x) + x^2 - 12)/x^5;
        % ddGamma_4=-(60*cos(x) - 9*x^2*cos(x) - x^3*sin(x) + 36*x*sin(x) + 3*x^2 - 60)/x^6;

        AB=  A*B;
        BA=  B*A;
        AA = A*A;
        dAB=Adot*B+A*Bdot;
        ddAB = Addot*B+2*Adot*Bdot+A*Bddot;
        dBA = Bdot*A+B*Adot;
        ddBA = Bddot*A+2*Bdot*Adot+B*Addot;
        dAA=Adot*A+A*Adot;
        ddAA=Addot*A+2*Adot*Adot+A*Addot;

        AB_BA = AB+BA;
        dAB_BA = (dAB)+(dBA); 
        ddAB_BA = (ddAB)+(ddBA);  
    
        ABA = AB*A;
        dABA = dAB*A+AB*Adot;
        ddABA = ddAB*A+dAB*Adot+dAB*Adot+AB*Addot;
    
        AABA = AA*BA;
        dAABA = dAA*BA+AA*dBA;
        ddAABA = ddAA*BA+2*dAA*dBA+AA*ddBA;
    
        AAB_BAA = A*AB+BA*A;
        dAAB_BAA = Adot*AB+A*dAB+dBA*A+BA*Adot;
        ddAAB_BAA = Addot*AB+A*ddAB+ddBA*A+BA*Addot+2*(dBA*Adot+Adot*dAB);

        dnormxi = xi'*xidot/norm(xi);
        ddnormxi = -norm(xi)^(-3)*(xi'*xidot)^2 + 1/norm(xi)*(xidot'*xidot+xi'*xiddot);
    
        dddexp3_=dddexpSo3(xi,xidot,xiddot);
        if abs(norm(xi))<1e-2
            lim_Gamma_1 = 1/6;
            lim_Gamma_2 = 1/24;        
            lim_Gamma_3 = 1/60;
            lim_Gamma_4 = 1/24;
                        
            lim_dotGamma_1 =-(xi'*xidot)/60;
             
            lim_dotGamma_2  =-(xi'*xidot)/360;
             
            lim_dotGamma_3 =-(xi'*xidot)/630;
             
            lim_dotGamma_4 =-(xi'*xidot)/120;
            C = +1/2*Bddot...  
                                          +2*lim_dotGamma_1*dAB_BA     +lim_Gamma_1*ddAB_BA...
                                                                   +lim_Gamma_2*ddAAB_BAA...
                                                                     +lim_Gamma_4*ddABA;    
            ret= [dddexp3_,C;O,dddexp3_];
            return;
        end
        C = +1/2*Bddot...
                                + (ddGamma_1*dnormxi^2+dGamma_1*ddnormxi)*AB_BA   +2*dGamma_1*dnormxi*dAB_BA     +Gamma_1*ddAB_BA...
                                + (ddGamma_2*dnormxi^2+dGamma_2*ddnormxi)*AAB_BAA +2*dGamma_2*dnormxi*dAAB_BAA   +Gamma_2*ddAAB_BAA...
                                + (ddGamma_3*dnormxi^2+dGamma_3*ddnormxi)*AABA      +2*dGamma_3*dnormxi*dAABA    +Gamma_3*ddAABA...
                                + (ddGamma_4*dnormxi^2+dGamma_4*ddnormxi)*ABA       +2*dGamma_4*dnormxi*dABA     +Gamma_4*ddABA;    
        ret = [dddexp3_,C;O,dddexp3_];
        if norm(ret)>1000
            disp(ret);
        end
end

