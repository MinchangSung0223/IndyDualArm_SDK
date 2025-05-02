function handles = drawAxisHandleColor(T, linelength, linewidth,color,type)
    if nargin<5
        type = '-';
    end
    % 변환 행렬을 이용하여 좌표 변환
    p0 = T * [0; 0; 0; 1];
    px = T * [linelength; 0; 0; 1];
    py = T * [0; linelength; 0; 1];
    pz = T * [0; 0; linelength; 1];

    % X 축 (빨강)
    handles.x = line([p0(1), px(1)], [p0(2), px(2)], [p0(3), px(3)], ...
                     "Color", color, "LineWidth", linewidth,"LineStyle",type);
    
    % Y 축 (초록)
    handles.y = line([p0(1), py(1)], [p0(2), py(2)], [p0(3), py(3)], ...
                     "Color", color, "LineWidth", linewidth,"LineStyle",type);
    
    % Z 축 (파랑)
    handles.z = line([p0(1), pz(1)], [p0(2), pz(2)], [p0(3), pz(3)], ...
                     "Color", color, "LineWidth", linewidth,"LineStyle",type);
    
    % 원점 (검은 점)
    handles.origin = plot3(p0(1), p0(2), p0(3), "k.", "MarkerSize", linewidth * 7);
end
