lambda_lr_list = [];
for i=length(lambda_rl_list):-1:1
    lambda_lr_list =[lambda_lr_list,-lambda_rl_list(:,i)];
end