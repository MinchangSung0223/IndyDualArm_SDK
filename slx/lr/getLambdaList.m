function [T_list,Llist]=getLambdaList(robot)
    robot.DataFormat='column';
    config= zeros(size(homeConfiguration(robot)));
    T_list={};
    for i = 1:1:length(robot.Bodies)
        T_list{i}=getTransform(robot,config,robot.BodyNames{i},robot.Base.Name);
    end
    Llist=[];
    for i=2:1:length(T_list)
        lambda = log6(TransInv(T_list{i-1})*T_list{i});
        Llist=[Llist,lambda];
    end
end