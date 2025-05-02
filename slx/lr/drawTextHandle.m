function handles = drawTextHandle(T, pos, fontsize)
    % 변환 행렬을 이용하여 좌표 변환
    p0 = T * [0; 0; 0; 1];
    px = T * [pos(1); 0; 0; 1];*
    py = T * [0; pos(2); 0; 1];
    pz = T * [0; 0; pos(3); 1];

    handles = text(px,py,pz, ...
                     "Color", color, "FontSize", fontsize);
   
end
