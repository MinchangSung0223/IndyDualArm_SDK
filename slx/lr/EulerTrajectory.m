function [Xd,Vd,dVd,s,ds,dds,mu3,Jw12,Jw23,Jw13,Jw_]=EulerTrajectory(X0, XT, V0,VT,dV0,dVT ,Tf, N,t,order,lambda)
    Xd=eye(3);
    Vd=zeros(6,1);
    dVd=zeros(6,1);
    order_num=str_to_order(order);

    [R0,p0] = TransToRp(X0);
    [RT,pT] = TransToRp(XT);

    p0T = pT-p0;
    p0T = p0T+eps;

    [Rx0,Ry0,Rz0]=rotm2eulm(R0,order);
    [RxT,RyT,RzT]=rotm2eulm(RT,order);

     wx0T = MatrixLog3(Rx0'*RxT);
     wy0T = MatrixLog3(Ry0'*RyT);
     wz0T = MatrixLog3(Rz0'*RzT);

    vec_wx0T = so3ToVec(wx0T);
    vec_wy0T = so3ToVec(wy0T);
    vec_wz0T = so3ToVec(wz0T);
  


    w0 = V0(1:3);
    wT = VT(1:3);
    wdot0 = dV0(1:3);
    wdotT = dVT(1:3);
    
    v0 = V0(4:6);
    vT = VT(4:6);
    vdot0 = dV0(4:6);
    vdotT = dVT(4:6);
    
    
    Jw0 = [vec_wx0T, vec_wy0T, vec_wz0T];
    JwT = [vec_wx0T, vec_wy0T, vec_wz0T];
    R0_ = {Rx0,Ry0,Rz0};    
    RT_ = {RxT,RyT,RzT};
    Jw0_= [[1,0,0]', [0,1,0]', [0,0,1]'];
    JwT_= [[1,0,0]', [0,1,0]', [0,0,1]'];

    Jw0_(:,order_num(2)) =  damped_psuedo_inv(R0_{order_num(3)},0)* Jw0_(:,order_num(2));
    Jw0_(:,order_num(1)) =  damped_psuedo_inv(R0_{order_num(2)}*R0_{order_num(3)},0)* (Jw0_(:,order_num(1)));
    JwT_(:,order_num(2)) =  damped_psuedo_inv(RT_{order_num(3)},0)* JwT_(:,order_num(2));
    JwT_(:,order_num(1)) =  damped_psuedo_inv(RT_{order_num(2)}*RT_{order_num(3)},0)* (JwT_(:,order_num(1)));

    Jw0(:,order_num(2)) =  damped_psuedo_inv(R0_{order_num(3)},0)* Jw0(:,order_num(2));
    Jw0(:,order_num(1)) =  damped_psuedo_inv(R0_{order_num(2)}*R0_{order_num(3)},0)* (Jw0(:,order_num(1)));
    
    JwT(:,order_num(2)) =damped_psuedo_inv(RT_{order_num(3)},0) * JwT(:,order_num(2));
     JwT(:,order_num(1)) =  damped_psuedo_inv(RT_{order_num(2)}*RT_{order_num(3)},0)* (JwT(:,order_num(1)));


    mu30 = 1/sqrt(det(Jw0*Jw0'));
    mu3T = 1/sqrt(det(JwT*JwT'));
%     pinvJw0=pinv(Jw0);
%     pinvJwT=pinv(JwT);
    pinvJw0=pinv(Jw0);
    pinvJwT=pinv(JwT);

    ds0_w = pinvJw0*w0;
    ds0_w_ = pinv(Jw0_)*w0;

    ds0_v = [v0(1)/p0T(1),v0(2)/p0T(2),v0(3)/p0T(3)];
    dsT_w = pinvJwT*wT;
    dsT_w_ = pinv(JwT_)*wT;
    
    dsT_v = [vT(1)/p0T(1),vT(2)/p0T(2),vT(3)/p0T(3)];


    dJw0=dAngularJacobian(Jw0,ds0_w,order);
    dJwT=dAngularJacobian(JwT,dsT_w,order);

    dds0_w = pinvJw0*wdot0-pinvJw0*dJw0*ds0_w;
    dds0_v = [vdot0(1)/p0T(1),vdot0(2)/p0T(2),vdot0(3)/p0T(3)];
    ddsT_w = pinvJwT*wdotT-pinvJwT*dJwT*dsT_w;
    ddsT_v = [vdotT(1)/p0T(1),vdotT(2)/p0T(2),vdotT(3)/p0T(3)];

    ds0 = [ds0_w',ds0_v];
    dsT = [dsT_w',dsT_v];
    dds0 = [dds0_w',dds0_v];
    ddsT = [ddsT_w',ddsT_v];
    s=zeros(6,1);
    ds=zeros(6,1);
    dds=zeros(6,1);


     if  sqrt(det(Jw0_*Jw0_'))<0.001
         disp((mu3T/mu30))
     end  
     if  sqrt(det(JwT_*JwT_'))<0.001
         disp((mu3T/mu30))
     end      
    for i =1:1:6
        [s_,ds_,dds_]=QuinticTimeScalingKinematics(0,1,ds0(i),dsT(i),dds0(i),ddsT(i),Tf,t);
        s(i) = s_; ds(i) = ds_; dds(i) = dds_;
    end
    e_wx = MatrixExp3(wx0T*s(1));
    e_wy = MatrixExp3(wy0T*s(2));
    e_wz = MatrixExp3(wz0T*s(3));

    Rxs = Rx0*e_wx;
    Rys = Ry0*e_wy;
    Rzs = Rz0*e_wz;
    Rs_= {Rxs,Rys,Rzs};
    Rs = Rs_{order_num(1)}*Rs_{order_num(2)}*Rs_{order_num(3)};
    ps = [p0(1)+s(4)*p0T(1),p0(2)+s(5)*p0T(2),p0(3)+s(6)*p0T(3)]';
    Xd = RpToTrans(Rs,ps);
    Jw_vec = [vec_wx0T, vec_wy0T, vec_wz0T];
    Jw = [vec_wx0T, vec_wy0T, vec_wz0T];
    Jw_ = [[1,0,0]', [0,1,0]', [0,0,1]'];
    Jw_(:,order_num(2)) = damped_psuedo_inv(Rs_{order_num(3)},0)* Jw_(:,order_num(2));
    Jw_(:,order_num(1)) = damped_psuedo_inv(Rs_{order_num(2)}*Rs_{order_num(3)},0)* Jw_(:,order_num(1));

    Jw(:,order_num(2)) = damped_psuedo_inv(Rs_{order_num(3)},0)* Jw(:,order_num(2));
    Jw(:,order_num(1)) =  damped_psuedo_inv(Rs_{order_num(2)}*Rs_{order_num(3)},0)* (Jw(:,order_num(1)));

   

    ds_w = [ds(1),ds(2),ds(3)]';

    ws = Jw*ds_w;
    vs = [ds(4)*p0T(1),ds(5)*p0T(2),ds(6)*p0T(3)]';
    Vd = [ws;vs];
    Xd = RpToTrans(Rs,ps);
    dJw=dAngularJacobian(Jw,ds_w,order);
%     dJw_=dAngularJacobian(Jw_,ds_w,order);
%     val=eig(Jw*Jw')
     if order=="XZY"
         disp("")
     end
%      mu3 =sqrt(det((Rs_{order_num(2)}*Rs_{order_num(3)})'));
%      1/sqrt(det(Jw*Jw'))/max([mu30,mu3T]);
%     mu3=acos(dot(Jw(:,2),Jw(:,3))/norm(Jw(:,2))/norm(Jw(:,3)));
     
%     mu3=acos(dot(Jw(:,1),Jw(:,3))/norm(Jw(:,1))/norm(Jw(:,3)));
%      mu3=acos(dot(Jw(:,1),Jw(:,2))/norm(Jw(:,1))/norm(Jw(:,2)));
    Jw12=acos(dot(Jw_(:,1),Jw_(:,2))/norm(Jw_(:,1))/norm(Jw_(:,2)));
    Jw23=acos(dot(Jw_(:,2),Jw_(:,3))/norm(Jw_(:,2))/norm(Jw_(:,3)));
    Jw13=acos(dot(Jw_(:,1),Jw_(:,3))/norm(Jw_(:,1))/norm(Jw_(:,3)));
   
    dds_w = [dds(1),dds(2),dds(3)]';
    wdots=Jw*dds_w+dJw*ds_w;
    vdots=[dds(4)*p0T(1),dds(5)*p0T(2),dds(6)*p0T(3)]';
    dVd=[wdots;vdots];

     s(1) = vec_wx0T(1)*s(1);
     s(2) = vec_wy0T(2)*s(2);
     s(3) = vec_wz0T(3)*s(3);
    mu3=norm([ds(1),ds(2),ds(3)]);

end
function dJw=dAngularJacobian(Jw,ds,order)
    order_num=str_to_order(order);
    dJw=zeros(3,3);
    dJw(:,order_num(3)) = zeros(3,1);
    dJw(:,order_num(2)) = VecToso3(Jw(:,order_num(2)))*Jw(:,order_num(3))*ds(order_num(3));
    dJw(:,order_num(1)) = VecToso3(Jw(:,order_num(1)))*Jw(:,order_num(3))*ds(order_num(3))+VecToso3(Jw(:,order_num(1)))*Jw(:,order_num(2))*ds(order_num(2));
end



function order_num=str_to_order(order)
    order_num= [0,0,0];
    if order{1}(1)=='X'
        if order{1}(2)=='Y'
            order_num(1) = 1;
            order_num(2) = 2;
            order_num(3) = 3;
        else
            order_num(1) = 1;
            order_num(2) = 3;
            order_num(3) = 2;
        end
    elseif order{1}(1)=='Y'
        if order{1}(2)=='X'
            order_num(1) = 2;
            order_num(2) = 1;
            order_num(3) = 3;
        else
            order_num(1) = 2;
            order_num(2) = 3;
            order_num(3) = 1;
        end
    elseif order{1}(1)=='Z'
        if order{1}(2)=='X'
            order_num(1) = 3;
            order_num(2) = 1;
            order_num(3) = 2;
        else
            order_num(1) = 3;
            order_num(2) = 2;
            order_num(3) = 1;
        end
    end
end
function [Rx,Ry,Rz]=rotm2eulm(R,order)
    eul0=rotm2eul(R,cell2mat(order));
    roll = 0;
    pitch=0;
    yaw = 0;
    if order{1}(1)=='X'
        if order{1}(2)=='Y'
            roll = eul0(1);
            pitch = eul0(2);
            yaw = eul0(3);
        else
            roll = eul0(1);
            pitch = eul0(3);
            yaw = eul0(2);
        end
    elseif order{1}(1)=='Y'
        if order{1}(2)=='X'
            roll = eul0(2);
            pitch = eul0(1);
            yaw = eul0(3);
        else
            roll = eul0(3);
            pitch = eul0(1);
            yaw = eul0(2);
        end
    elseif order{1}(1)=='Z'
        if order{1}(2)=='X'
            roll = eul0(2);
            pitch = eul0(3);
            yaw = eul0(1);
        else
            roll = eul0(3);
            pitch = eul0(2);
            yaw = eul0(1);
        end
    end
    Rx = Rotx(roll);
    Ry = Roty(pitch);
    Rz = Rotz(yaw);
    

    function R=Rotx(val)
        R = [1 0 0 ;
             0 cos(val) -sin(val) ;
             0 sin(val) cos(val) ];
    end
    function R=Roty(val)
        R = [cos(val) 0 sin(val) ;
             0 1 0 ;
             -sin(val) 0 cos(val) ];
    end
    function R=Rotz(val)
        R = [cos(val) -sin(val) 0 ;
             sin(val) cos(val) 0 ;
             0 0 1 ];
    end    
end

function [s,ds,dds]=QuinticTimeScalingKinematics(s0,sT,ds0,dsT,dds0,ddsT,Tf,t)
    x(1) = s0;
    x(2) = ds0;
    x(3) = dds0/2.0;
    x(4) = -(10*s0 - 10*sT + 2*Tf*(3*ds0 + 2*dsT) + (Tf^2*(3*dds0 - ddsT))/2)/Tf^3;
    x(5) = (((3*dds0)/2 - ddsT)*Tf^2 + (8*ds0 + 7*dsT)*Tf + 15*s0 - 15*sT)/Tf^4;
    x(6) = -(6*s0 - 6*sT + (Tf^2*(dds0 - ddsT))/2 + 3*Tf*(ds0 + dsT))/Tf^5;
    s = x(1)+x(2)*t+x(3)*t^2+x(4)*t^3+x(5)*t^4+x(6)*t^5;
    ds = x(2)+2*x(3)*t+3*x(4)*t^2+4*x(5)*t^3+5*x(6)*t^4;
    dds = 2*x(3)+2*3*x(4)*t+3*4*x(5)*t^2+4*5*x(6)*t^3;
end
function dpinvR = damped_psuedo_inv(R,epsilon)
%               dpinvR = (MatrixExp3(MatrixLog3(R*inv(R'*R+epsilon*epsilon*eye(3)))))';
              dpinvR = R';
%              dpinvR = pinv(R*Rotz(epsilon)*Rotx(epsilon)*Roty(epsilon));
end
function ret=check_gimbal_lock(R,order)
    [Rx,Ry,Rz]=rotm2eulm(R,order);
    order_num=str_to_order(order);
    R_ = {Rx,Ry,Rz};    
    R_ = {Rx,Ry,Rz};
    Jw1 = MatrixLog3(R_(order_num(3)));
    Jw2= R_{order_num(3)}'*MatrixLog3(R_(order_num(2)));
    Jw3= (R_{order_num(2)}*R_{order_num(3)})'*MatrixLog3(R_(order_num(1)));
    if sum(cross(Jw1,Jw2)) < eps
        ret = -1;
        return;
    end
    if sum(cross(Jw2,Jw3)) < eps
        ret = -1;
        return;
    end
    if sum(cross(Jw3,Jw1)) < eps
        ret = -1;
        return;
    end
    ret =1;
end