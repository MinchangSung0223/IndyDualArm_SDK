function [theta,dtheta,ddtheta] = JointTrajectoryTime(thetastart, thetaend, Tf, method,t)
if t>Tf
    t=Tf;
end
if method == 3
    [s,ds,dds]= CubicTimeScalingKinematics(Tf, t);
else
    [s,ds,dds] = QuinticTimeScalingKinematics(Tf, t);
end
theta= thetastart + s * (thetaend - thetastart);
dtheta= ds * (thetaend - thetastart);
ddtheta= dds * (thetaend - thetastart);

end

function [s,ds,dds]= CubicTimeScalingKinematics(Tf, t)
    s = 3 * (t^ 2 / Tf^ 2)  - 2 * (t^ 3 / Tf^ 3) ;
    ds = 3 * (2*t / Tf^ 2)  - 2 * (3*t^ 2 / Tf^ 3) ;
    dds = 3 * (2 / Tf^ 2)  - 2 * (2*3*t / Tf^ 3) ;
end
function [s,ds,dds] = QuinticTimeScalingKinematics(Tf, t)
    s = 10 * (t / Tf) ^ 3 - 15 * (t / Tf) ^ 4 + 6 * (t / Tf) ^ 5;
    ds = 10 * (3*t^ 2 / Tf^ 3)  - 15 * (4*t^ 3 / Tf^ 4 )  + 6 * (5*t^ 4 / Tf ^ 5) ;
    dds = 10 * (2*3*t / Tf^ 3)  - 15 * (4*3*t^ 2 / Tf^ 4 )  + 6 * (5*4*t^ 3 / Tf ^ 5) ;
end