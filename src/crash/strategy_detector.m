left_delta_time=6;
right_delta_time=4;
record_crash_washed=filter_strategy_detector(trackerW,record_crash,left_delta_time,right_delta_time);


for outer_index=1:size(record_crash_washed,2)
    pair=record_crash_washed(1,outer_index);
    time_crash=floor(pair.time_end);
    index1=pair.id(1);
    index2=pair.id(2);
    states1=trackerW(index1).states(1:3,:);
    states2=trackerW(index2).states(1:3,:);
    timer1=trackerW(index1).start:trackerW(index1).end;   
    timer2=trackerW(index2).start:trackerW(index2).end;   
        %cut time(left_delta_time---crash---right_delta_time)
    if left_delta_time~=0 || right_delta_time~=0
        start_time1=max(timer1(1),time_crash-left_delta_time);
        end_time1=min(timer1(end),time_crash+right_delta_time);
        states1=states1(1:3,find(timer1==start_time1):find(timer1==end_time1));
        timer1=timer1(find(timer1==start_time1):find(timer1==end_time1));   
        
        start_time2=max(timer2(1),time_crash-left_delta_time);
        end_time2=min(timer2(end),time_crash+right_delta_time);
        states2=states2(1:3,find(timer2==start_time1):find(timer2==end_time1));
        timer2=timer2(find(timer2==start_time1):find(timer2==end_time1));  
    end
    
    velocity1=states1(1:3,2:end)-states1(1:3,1:end-1);
    acc1=velocity1(1:3,2:end)-velocity1(1:3,1:end-1);
    acc1_d=acc1(1:3,2:end)-acc1(1:3,1:end-1);
    
    velocity2=states2(1:3,2:end)-states2(1:3,1:end-1);
    acc2=velocity2(1:3,2:end)-velocity2(1:3,1:end-1);
    acc2_d=acc2(1:3,2:end)-acc2(1:3,1:end-1);
    
    
    
    [ v1_norm,acc1_norm, acc1_on_v1_past, acc1_on_v1_past_norm, r1, v1_ang, acc1_ang] = calc_trace_attribute( states1 );
    
    [ v2_norm,acc2_norm, acc2_on_v2_past, acc2_on_v2_past_norm, r2, v2_ang, acc2_ang] = calc_trace_attribute( states2 );
    
    
end


clear pair time_crash index1 index2 states1 states2 timer1 timer2 end_time1 end_time2;
