function handles = drawBoxHandle(T, boxsize, linewidth)
    % ----------------------------------------------------------
    % 1) 박스의 로컬 좌표 (8개 꼭짓점)
    %    - 박스 중심이 (0,0,0)에 오도록, 
    %      boxsize/2만큼 +/-로 설정
    % ----------------------------------------------------------
    cornersLocal = [
        -boxsize/2, -boxsize/2, -boxsize/2;  % 1
        -boxsize/2, -boxsize/2,  boxsize/2;  % 2
        -boxsize/2,  boxsize/2, -boxsize/2;  % 3
        -boxsize/2,  boxsize/2,  boxsize/2;  % 4
         boxsize/2, -boxsize/2, -boxsize/2;  % 5
         boxsize/2, -boxsize/2,  boxsize/2;  % 6
         boxsize/2,  boxsize/2, -boxsize/2;  % 7
         boxsize/2,  boxsize/2,  boxsize/2   % 8
    ];
    
    % ----------------------------------------------------------
    % 2) 동차좌표(4×1)로 만들고, 변환행렬 T를 곱함
    %    cornersLocal: 8×3 -> homCoord: 8×4 -> (T×homCoord')
    % ----------------------------------------------------------
    homCoord = [cornersLocal, ones(8,1)]';       % (4×8)
    transformed = T * homCoord;                  % (4×8)
    cornersWorld = transformed(1:3,:)';          % 8×3
    
    % ----------------------------------------------------------
    % 3) 박스를 이루는 12개 모서리를 정의
    %    각 모서리는 꼭짓점 인덱스 쌍
    %    예: 1 2는 cornersWorld에서 1번 꼭짓점과 2번 꼭짓점을 잇는 라인
    % ----------------------------------------------------------
    edges = [
        1 2; 1 3; 1 5;
        2 4; 2 6;
        3 4; 3 7;
        4 8;
        5 6; 5 7;
        6 8;
        7 8
    ];
    
    % ----------------------------------------------------------
    % 4) 선(Line)으로 그려주기
    %    - handles를 배열로 관리: handles(i) = line(...)
    %    - 색상은 검정('k')으로 지정, 필요시 변경 가능
    % ----------------------------------------------------------
    nEdges = size(edges,1);
    handles = gobjects(nEdges,1);  % 그래픽 핸들을 담을 배열
    for i = 1:nEdges
        idx = edges(i,:);
        xyz = cornersWorld(idx, :);   % 2×3
        handles(i) = line( ...
            xyz(:,1), xyz(:,2), xyz(:,3), ...
            'Color', 'm', ...
            'LineWidth', linewidth ...
        );
    end
end
