function deleteBoxHandles(handles)
    
    for i = 1:numel(handles)
        h = handles(i); % 개별 핸들 가져오기
        if isvalid(h) % 유효한 그래픽 객체인지 확인
            delete(h); % 삭제
        end
    end
end
