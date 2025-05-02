function Tn_list=getNomData(lambda_list,SampleNum,qr_list)
    Tn_list={};
    [m,n]=size(lambda_list);
    for k=1:1:SampleNum
        q=qr_list(k,:)';
        
        Tn =FK(lambda_list,q);
        % xi = log3(Tn(1:3,1:3)');
        Tn_list{k} =Tn;
    end
end