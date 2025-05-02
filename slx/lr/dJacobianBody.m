function dJb = dJacobianBody(Jb,q_dot)
    n = length(q_dot);
    dJb =zeros(6,n);
    for i=1:1:n
        Ji = Jb(:,i);
        dJidt = zeros(6,1);
        for j=1:1:n
            Jj = Jb(:,j);
            aJiaqj = zeros(6,1);
            if(i<j) 
                aJiaqj = ad(Ji)*Jj;
            end
            dJidt = dJidt+aJiaqj*q_dot(j);
        end
        dJb(:,i) = dJidt;
    end
end

