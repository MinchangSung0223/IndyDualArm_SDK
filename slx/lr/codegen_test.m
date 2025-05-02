addpath('../casadi-3.6.7-windows64-matlab2018b');
addpath("../casadi_lr");
import casadi.*;

% 함수 리스트
function_names = {
    'ad', 'AxisAng3', 'AxisAng6', 'dddexp3', 'dddlog3', ...
    'ddexp3', 'ddexp6', 'ddlog3', 'ddlog6', 'dexp3', ...
    'dexp6', 'dJacobianBody', 'dlog3', 'dlog6', 'exp3', ...
    'exp6', 'FKinBody', 'FKinSpace', 'JacobianBody', ...
    'JointTrajectoryTime', 'MatrixLog3', 'log3', 'log6', ...
    'MatrixExp3', 'MatrixExp6', 'MatrixLog6', 'NearZero', ...
    'RpToTrans', 'se3ToVec', 'so3ToVec', 'TransInv', ...
    'TransToRp', 'VecTose3', 'VecToso3', 'pRNE', ...
    'CoriolisMatrix', 'GravityForces',...
    'MassMatrix'
};
load kukakr510info.mat;
Mlist= reshape(Mlist,16,[]);
Glist= reshape(Glist,36,[]);
Slist= reshape(Slist,6,[]);
pRNEFunc = pRNE();
q = [0.0,-pi/2,pi/2,0.0,0.0,0.0]';
q2 = [0.0,-pi/2,pi/2,0.0,0.0,0.0]';
MassMatrixFunc = MassMatrix();

% pRNEFunc(q,zeros(6,1),zeros(6,1),q,zeros(6,1),zeros(6,1),[0,0,-9.81]',zeros(6,1),Mlist,Glist,Slist)
% 개별적으로 .c 파일 생성
for i = 1:length(function_names)
    func_name = function_names{i};
    cg = CodeGenerator([func_name '.c'], struct("mex", true));
    cg.add(feval(func_name)); % 함수 추가
    cg.generate(); % .c 파일 생성
end


% 각 파일을 mex 빌드
for i = 1:length(function_names)
    func_name = function_names{i};
    source_file = [func_name '.c']; % .c 파일 이름
    try
        % mex로 컴파일
        mex(source_file); % 필요한 경우 mex에 추가 옵션을 포함
        fprintf('%s 파일이 성공적으로 컴파일되었습니다.\n', source_file);
    catch ME
        fprintf('%s 파일 컴파일 중 오류가 발생했습니다: %s\n', source_file, ME.message);
    end
end
