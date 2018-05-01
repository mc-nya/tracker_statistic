function valid= show_trace(trackerW, pair ,delta_time)
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
    filter_min_dist=1;
    filter_time_delta=10;
    filter_min_speed=1;
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
    
    
    %-------test code------------
    temp_states1=states1(1:3,find(timer1==time_nearest):find(timer1==time_nearest));
    temp_states2=states2(1:3,find(timer2==time_nearest):find(timer2==time_nearest));
    temp_diff=temp_states1-temp_states2;
    temp_dist=norm(temp_diff(1:3))
    
    %-------------draw----------------
    figure(1);
    hold off;
    if delta_time==0
        draw_states1=states1;
        draw_states2=states2;
    else
        draw_s_time1=max(timer1(1),time_nearest-delta_time);
        draw_s_time2=max(timer2(1),time_nearest-delta_time);
        draw_e_time1=min(timer1(end),time_nearest+delta_time);
        draw_e_time2=min(timer2(end),time_nearest+delta_time);
        draw_states1=states1(1:3,find(timer1==draw_s_time1):find(timer1==draw_e_time1));
        draw_states2=states2(1:3,find(timer2==draw_s_time2):find(timer2==draw_e_time2));
    end
        
    plot3(draw_states1(1,:),draw_states1(2,:),draw_states1(3,:),'r');
    hold on;
    plot3(draw_states2(1,:),draw_states2(2,:),draw_states2(3,:),'b');
    title(['time:' num2str(time_nearest) ' dist:' num2str(temp_dist)]);
    for i=1:10:size(draw_states1,2)
        plot3(draw_states1(1,i),draw_states1(2,i),draw_states1(3,i),'r+');
    end
    for i=1:10:size(draw_states2,2)
        plot3(draw_states2(1,i),draw_states2(2,i),draw_states2(3,i),'b+');
    end
    plot3(draw_states1(1,1),draw_states1(2,1),draw_states1(3,1),'ro');
    plot3(draw_states2(1,1),draw_states2(2,1),draw_states2(3,1),'bo');
    
    index1_nearest=find(timer1==time_nearest);
    index2_nearest=find(timer2==time_nearest);
    plot3(states1(1,index1_nearest),states1(2,index1_nearest),states1(3,index1_nearest),'rh');
    plot3(states2(1,index2_nearest),states2(2,index2_nearest),states2(3,index2_nearest),'bh');
    
    valid=1;
end

