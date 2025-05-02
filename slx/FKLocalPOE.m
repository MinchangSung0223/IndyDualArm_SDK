function T=FKLocalPOE(lambda_list,q)
    Ti = eye(4);
    [m,n]=size(lambda_list);
    lamda_z = [0,0,0,0,0,1]';
    ez = [0,0,1]';
    N = length(q);
  
    Ti = exp6(lambda_list(:,end));
    [Ri,pi] = TransToRp(Ti);
    for i =N:-1:1
        Ti = exp6(lambda_list(:,i))*exp6(lamda_z*q(i))*Ti;
 
        [Ri,pi] = TransToRp(Ti);

    end
    T = Ti;
end