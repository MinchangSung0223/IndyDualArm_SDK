restoredefaultpath
addpath(genpath(pwd));
robot = importrobot("indy7_dualArm.urdf");
robot.DataFormat="column";
robot.Gravity=[0,0,-9.80665];