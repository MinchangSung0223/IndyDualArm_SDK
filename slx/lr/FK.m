function T=FK(lambda_list,q)
    
    Ti = eye(4);
    [m,n]=size(lambda_list);
    lamda_z = [0,0,0,0,0,1]';
    ez = [0,0,1]';
    N = length(q);
    J = zeros(6,N);
    Jdot=zeros(6,N);
    q_dot = zeros(N,1);
    Ti = exp6(lambda_list(:,end));
    [Ri,pi] = TransToRp(Ti);
    J(:,end)=[Ri'*VecToso3(ez)*pi;Ri'*ez];
    for i =N:-1:1
        Ti = exp6(lambda_list(:,i))*exp6(lamda_z*q(i))*Ti;
 
        [Ri,pi] = TransToRp(Ti);
        if i>1
            J(:,i-1)=[Ri'*VecToso3(ez)*pi;Ri'*ez];
        end

    end
    T = Ti;
end