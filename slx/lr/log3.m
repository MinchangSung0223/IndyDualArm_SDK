function xi = log3(R)
        % 
        % A = R+R';
        % B = R-R';
        % normB = norm(B);
        % b = [B(3, 2)./normB; B(1, 3)./normB; B(2, 1)./normB];
        % [U,E,V]=eig(A);
        % xihat = U(:,end);
        % normxi = acos((trace(R)-1)/2);
        % if dot(xihat, b) < 0
        %     xi = -xihat*normxi;
        %     return;
        % else
        %     xi = xihat*normxi;
        %     return;
        % end
        % 
        % 
        xihat=so3ToVec(R-R');
        if norm(xihat)<eps
            xi = xihat;
            return 
        end
        xihat = xihat/norm(xihat);
        normxi = acos((trace(R)-1)/2);
        xi = normxi*xihat;

        

end

function sign_xi = find_sign_xi(R)
    % R은 3x3 회전 행렬
    sign_xi = zeros(3,1);

    % 1. 절댓값이 가장 큰 축을 찾음
    [~, max_idx] = max(abs([R(1,1), R(2,2), R(3,3)]));
    
    % 2. 기준 축의 부호를 1로 가정
    sign_xi(max_idx) = 1;
    
    % 3. 나머지 두 축의 부호를 R_ij를 이용하여 결정
    for i = 1:3
        if i ~= max_idx
            sign_xi(i) = -sign_xi(max_idx) * sign(R(i, max_idx));
        end
    end
end