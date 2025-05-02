function dJs = dJacobianSpace(Js,q_dot)
    n = length(q_dot);
    dJs =zeros(6,n);
    for i=1:1:n
        Ji = Js(:,i);
        dJidt = zeros(6,1);
        for j=1:1:n
            Jj = Js(:,j);
            aJiaqj = zeros(6,1);
            if(i>j) 
                aJiaqj = ad(Ji)*Jj;
            end
            dJidt = dJidt+aJiaqj*q_dot(j);
        end
        dJs(:,i) = dJidt;
    end
end

