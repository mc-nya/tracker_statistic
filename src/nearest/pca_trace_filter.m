function ret = pca_trace_filter( trackerW,detect_set,delta_time )

    ret=[];
    for index=1:size(detect_set)
        valid=singel_filter(trackerW,detect_set(index,:),delta_time);
        if valid==1
            ret=[ret;detect_set(index,:)];
        end
    end


end

function valid=singel_filter(trackerW, pair ,delta_time)
    time_nearest=pair(1);
    index1=pair(2);
    index2=pair(3);
    states1=trackerW(index1).states(1:3,:);
    states2=trackerW(index2).states(1:3,:);
    timer1=trackerW(index1).start:trackerW(index1).end;
    timer2=trackerW(index2).start:trackerW(index2).end;
    time_both_start=max(timer1(1),timer2(1));
    time_both_end=min(timer1(end),timer2(end));
    velocity1=states1(1:3,2:end)-states1(1:3,1:end-1);
    velocity2=states2(1:3,2:end)-states2(1:3,1:end-1);
    

    %-----------filter--------------
    filter_min_avg_dist=4;
    filter_min_dist=0.05;
    filter_time_delta=delta_time;
    filter_min_speed=1.5;
    %------0. minium length to two sides-------
    if timer1(1)>time_nearest-delta_time || timer1(end)<time_nearest+delta_time
        valid=0;
        return;
    end
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
    temp_start_time=max(time_both_start,time_nearest-filter_time_delta);
    temp_end_time=min(time_both_end,time_nearest+filter_time_delta);
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