function ret = filter_strategy_detector( trackerW,record_crash,left_delta_time,right_delta_time)
% É¸Ñ¡ºÏÊÊµÄ¹ì¼£
%   
    ret=[];
    for index=1:size(record_crash,2)
        valid=singel_filter(trackerW,record_crash(1,index),left_delta_time,right_delta_time);
        if valid==1
            ret=[ret record_crash(1,index)];
        end
    end

end

function valid=singel_filter(trackerW, pair ,left_delta_time,right_delta_time)
    time_crash=pair.time_end;
    %time_detect=pair.time_start;
    
    
    index1=pair.id(1);
    index2=pair.id(2);
    states1=trackerW(index1).states(1:3,:);
    states2=trackerW(index2).states(1:3,:);
    timer1=trackerW(index1).start:trackerW(index1).end;
    timer2=trackerW(index2).start:trackerW(index2).end;
    time_both_start=max(timer1(1),timer2(1));
    time_both_end=min(timer1(end),timer2(end));
    velocity1=states1(1:3,2:end)-states1(1:3,1:end-1);
    velocity2=states2(1:3,2:end)-states2(1:3,1:end-1);
    if(time_both_start>=time_both_end)
        valid=0;
        return;
    end
    %-----------filter--------------
    filter_min_avg_dist=4;
    filter_min_dist=0.5;
    filter_min_speed=1.5;
    
    %------0. minium length to two sides-------
    if timer1(1)>time_crash-left_delta_time || timer1(end)<time_crash+right_delta_time
        valid=0;
        return;
    end
%     if timer2(1)>time_crash-left_delta_time || timer2(end)<time_crash+right_delta_time
%         valid=0;
%         return;
%     end
    
    %------1. too close trace -all--------------
    temp_states1=states1(1:3,find(timer1==time_both_start):find(timer1==time_both_end));
    temp_states2=states2(1:3,find(timer2==time_both_start):find(timer2==time_both_end));
    temp_diff=temp_states1-temp_states2;
    temp_sum=0; 
    for i=1:size(temp_diff,2)
        temp_dist=norm(temp_diff(1:3,i));
        if temp_dist<filter_min_dist
            valid=0;
            return;
        end
        temp_sum=temp_sum+temp_dist;
    end
    if temp_sum/(time_both_end-time_both_start+1)<filter_min_avg_dist
        valid=0;
        return;
    end
    
    
    %------2. too close trace - nearest point --------------
    temp_start_time=floor(max(time_both_start,time_crash-left_delta_time));
    temp_end_time=floor(min(time_both_end,time_crash+right_delta_time));
    temp_states1=states1(1:3,find(timer1==temp_start_time):find(timer1==temp_end_time));
    temp_states2=states2(1:3,find(timer2==temp_start_time):find(timer2==temp_end_time));
    temp_diff=temp_states1-temp_states2;
    temp_sum=0; 
    for i=1:size(temp_diff,2)
        temp_sum=temp_sum+temp_dist+norm(temp_diff(1:3,i));
    end
    if temp_sum/(time_both_end-time_both_start+1)<filter_min_avg_dist
        valid=0;
        return;
    end
    
    %------------3. minium speed-----------------------------
    for i=1:size(velocity1,2)
        if norm(velocity1(1:3,i))<filter_min_speed
            valid=0;
            return ;
        end
    end
    for i=1:size(velocity2,2)
        if norm(velocity2(1:3,i))<filter_min_speed
            valid=0;
            return ;
        end
    end
    
    valid=1;
    
    
    
    
    
end
