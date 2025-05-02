% %% build_all.m  ─ Simulink-to-C 코드 생성 일괄 빌드 스크립트
% restoredefaultpath;          % MATLAB 경로 초기화
% addpath(genpath(pwd));       % 현재 폴더와 하위 폴더를 모두 포함
% 
% models = { ...
%     'FD', ...
%     'FK', ...
%     'ID', ...
%     'JSTraj', ...
%     'TSTraj',...
%     'HinfController'};
% 
% for k = 1:numel(models)
%     m = models{k};
%     mdlPath = fullfile(pwd, [m '.slx']);
% 
%     fprintf('\n### === Building %s ===\n', m);
%     load_system(mdlPath);    % 모델 메모리 로드
%     slbuild(m);              % 코드 생성
%     close_system(m, 0);      % 저장 안 하고 닫기(선택)
% end
%     fprintf('\n### === Done %s ===\n', m);
% 
% quit                          % MATLAB 종료

%% build_all.m  ─ Simulink-to-C 코드 생성 일괄 빌드 스크립트
restoredefaultpath;          % MATLAB 경로 초기화
addpath(genpath(pwd));       % 현재 폴더와 하위 폴더를 모두 포함

models = { 'TaskSpaceController'};

for k = 1:numel(models)
    m = models{k};
    mdlPath = fullfile(pwd, [m '.slx']);

    fprintf('\n### === Building %s ===\n', m);
    load_system(mdlPath);    % 모델 메모리 로드
    slbuild(m);              % 코드 생성
    close_system(m, 0);      % 저장 안 하고 닫기(선택)
end
    fprintf('\n### === Done %s ===\n', m);

quit                          % MATLAB 종료
