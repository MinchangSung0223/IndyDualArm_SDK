function [T,V,Vdot]=TaskWayPointTrajectory(waypoint_list,delay_list,t)
        N_delay = length(delay_list);
        time_list = zeros(N_delay+1,1);
        sum_delays = 0;
        idx = 0;
        for i =1:1:N_delay
            sum_delays = sum_delays+delay_list(i);
            time_list(i+1) = sum_delays;
            if(t>=time_list(i) && t< time_list(i+1))
                idx = i;
            end
        end
        if(t>=time_list(length(time_list)-1))
            idx = N_delay-1;
        end
        [T,V,Vdot]=LieScrewTrajectoryTime(waypoint_list(:,:,idx),waypoint_list(:,:,idx+1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),time_list(idx+1)-time_list(idx),t-time_list(idx));        
     
end