function C = CoriolisMatrix(thetalist,dthetalist,Mlist,Glist,Slist)
    n = length(thetalist);
    C=zeros(n,n);
    for i=1:1:n
        ei = zeros(n,1);
        ei(i) =1;
        C(:,i)=pRNE(thetalist,dthetalist,zeros(n,1),thetalist,ei,zeros(n,1),[0,0,0]',zeros(6,1),Mlist,Glist,Slist);
    end
end

