left_delta_time=6;
right_delta_time=4;
record_crash_washed=filter_strategy_detector(trackerW,record_crash,left_delta_time,right_delta_time);
threshold_acc=1;
threshold_acc_d=1;
threshold_acc_norm=0.5;
threshold_acc_norm_d=0.5;
threshold_acc_tan=0.5;
threshold_acc_tan_d=0.5;

%==============step 1 根据加速度和加速度梯度对齐==============
for outer_index=1:size(record_crash_washed,2)
    pair=record_crash_washed(1,outer_index);
    time_crash=floor(pair.time_end);
    time_detect=pair.time_start;
    index1=pair.id(1);
    index2=pair.id(2);
    states1=trackerW(index1).states(1:3,:);
    states2=trackerW(index2).states(1:3,:);
    timer1=trackerW(index1).start:trackerW(index1).end;   
    timer2=trackerW(index2).start:trackerW(index2).end;   
        %cut time(time_start---crash---right_delta_time)
    if right_delta_time~=0
        start_time1=max(timer1(1),time_detect);
        end_time1=min(timer1(end),time_crash+right_delta_time);
        states1=states1(1:3,find(timer1==start_time1):find(timer1==end_time1));
        timer1=timer1(find(timer1==start_time1):find(timer1==end_time1));   
        
        start_time2=max(timer2(1),time_detect);
        end_time2=min(timer2(end),time_crash+right_delta_time);
        states2=states2(1:3,find(timer2==start_time2):find(timer2==end_time2));
        timer2=timer2(find(timer2==start_time2):find(timer2==end_time2));  
    end
    
    velocity1=states1(1:3,2:end)-states1(1:3,1:end-1);
    acc1=velocity1(1:3,2:end)-velocity1(1:3,1:end-1);
    acc1_d=acc1(1:3,2:end)-acc1(1:3,1:end-1);
    
    velocity2=states2(1:3,2:end)-states2(1:3,1:end-1);
    acc2=velocity2(1:3,2:end)-velocity2(1:3,1:end-1);
    acc2_d=acc2(1:3,2:end)-acc2(1:3,1:end-1);
    
%     for i=1:size(acc1_d,2)
%         if(norm(acc1(:,i))>threshold_acc) || (norm(acc1_d(:,i))>threshold_acc_d)
%             record_crash_washed(1,outer_index).time_strategy=timer1(i);
%             break;
%         end
%     end 
    [ v1_norm,acc1_norm, acc1_on_v1_past, acc1_on_v1_past_norm, r1, v1_ang, acc1_ang] = calc_trace_attribute( states1 );
    acc1_tan_d=acc1_on_v1_past(2:end)-acc1_on_v1_past(1:end-1);
    acc1_norm_d=acc1_on_v1_past_norm(2:end)-acc1_on_v1_past_norm(1:end-1);
    acc1_tan=acc1_on_v1_past;
    acc1_norm=acc1_on_v1_past_norm;
    for i=1:size(acc1_tan_d,2)
        if(norm(acc1_tan(:,i))>threshold_acc_tan) || (norm(acc1_tan_d(:,i))>threshold_acc_tan_d)
            record_crash_washed(1,outer_index).time_strategy=timer1(i);
            break;
        end
        if(norm(acc1(:,i))>threshold_acc) || (norm(acc1_d(:,i))>threshold_acc_d)
            record_crash_washed(1,outer_index).time_strategy=timer1(i);
            break;
        end
        if(norm(acc1_norm(:,i))>threshold_acc_norm) || (norm(acc1_norm_d(:,i))>threshold_acc_norm_d)
            record_crash_washed(1,outer_index).time_strategy=timer1(i);
            break;
        end
    end 
    %[ v2_norm,acc2_norm, acc2_on_v2_past, acc2_on_v2_past_norm, r2, v2_ang, acc2_ang] = calc_trace_attribute( states2 );
    
end

% ====================step2  筛选出找到策略的项，并保证与起始点不重合==============
temp_record_crash_washed=[];
for outer_index=1:size(record_crash_washed,2)
    if isempty(record_crash_washed(1,outer_index).time_strategy)~=1
        if record_crash_washed(1,outer_index).time_strategy~=record_crash_washed(1,outer_index).time_start
            if record_crash_washed(1,outer_index).time_strategy<record_crash_washed(1,outer_index).time_end
                temp_record_crash_washed=[temp_record_crash_washed record_crash_washed(1,outer_index)];
            end
        end
    end
end
record_crash_washed=temp_record_crash_washed;

%==============step3  对齐轨迹并可视化==============
time_left_longest=0;
time_right_longest=0;

for outer_index=1:size(record_crash_washed,2)
    pair=record_crash_washed(1,outer_index);
    time_crash=floor(pair.time_end);
    time_detect=pair.time_start;
    time_strategy=pair.time_strategy;
    time_left_longest=max(time_strategy-time_detect,time_left_longest);
    time_right_longest=max(time_crash-time_strategy,time_right_longest);
end

feature_angle_distribution=[];
figure(1);
hold on;
feature_time_dist=[];
for outer_index=1:size(record_crash_washed,2)
    pair=record_crash_washed(1,outer_index);
    time_crash=floor(pair.time_end);
    time_detect=pair.time_start;
    time_strategy=pair.time_strategy;
    time_pad=time_strategy-time_left_longest-1;
    feature_time_dist=[feature_time_dist; [time_strategy-time_detect time_crash-time_strategy]];
    
    index1=pair.id(1);
    index2=pair.id(2);
    states1=trackerW(index1).states(1:3,:);
    states2=trackerW(index2).states(1:3,:);
    timer1=trackerW(index1).start:trackerW(index1).end;   
    timer2=trackerW(index2).start:trackerW(index2).end;   
     %cut time(time_start---crash---right_delta_time)
    if right_delta_time~=0
        start_time1=max(timer1(1),time_detect);
        end_time1=min(timer1(end),time_crash+right_delta_time);
        states1=states1(1:3,find(timer1==start_time1):find(timer1==end_time1));
        timer1=timer1(find(timer1==start_time1):find(timer1==end_time1));   
        
        start_time2=max(timer2(1),time_detect);
        end_time2=min(timer2(end),time_crash+right_delta_time);
        states2=states2(1:3,find(timer2==start_time2):find(timer2==end_time2));
        timer2=timer2(find(timer2==start_time2):find(timer2==end_time2));  
    end
    timer1=timer1-time_pad;
    timer2=timer2-time_pad;
    time_crash=time_crash-time_pad;
    time_detect=time_detect-time_pad;
    time_strategy=time_strategy-time_pad;
   
    [ v1_norm,acc1_norm, acc1_on_v1_past, acc1_on_v1_past_norm, r1, v1_ang, acc1_ang] = calc_trace_attribute( states1 );
    
    [ v2_norm,acc2_norm, acc2_on_v2_past, acc2_on_v2_past_norm, r2, v2_ang, acc2_ang] = calc_trace_attribute( states2 );
    
    
    velocity1=states1(1:3,2:end)-states1(1:3,1:end-1);
    acc1=velocity1(1:3,2:end)-velocity1(1:3,1:end-1);
    acc1_d=acc1(1:3,2:end)-acc1(1:3,1:end-1);
    
    velocity2=states2(1:3,2:end)-states2(1:3,1:end-1);
    acc2=velocity2(1:3,2:end)-velocity2(1:3,1:end-1);
    acc2_d=acc2(1:3,2:end)-acc2(1:3,1:end-1);
    
    
    dur_time=pair.delta_time;
    v1=pair.velocity(1:3,1);
    p1_crash=states1(1:3,find(timer1==time_detect))+v1*dur_time;
    dist_s1_crash_point=[];
    for i=1:size(timer1,2);
        %==========简记变量初始化=========
        s1=states1(1:3,i);  
        %======计算和虚拟碰撞点的距离===========
        dist_s1_crash_point=[dist_s1_crash_point; norm(s1-p1_crash)];     
    end
   
    timer_both=max(timer1(1),timer2(1)):min(timer1(end)-2,timer2(end)-2);
    dist_s1_s2=[];
    angl_v1_v2=[];
    norm_v1_on_v2=[];
    norm_v2_on_v1=[];
    for i=1:size(timer_both,2)
        s1=states1(1:3,find(timer1==timer_both(i)));
        s2=states2(1:3,find(timer2==timer_both(i)));
        dist_s1_s2=[dist_s1_s2; norm(s1-s2)];
        
        if i~=size(timer_both,2)
            v1=velocity1(1:3,find(timer1==timer_both(i)));
            v2=velocity2(1:3,find(timer2==timer_both(i)));
            angl_v1_v2=[angl_v1_v2 acosd(dot(v1,v2)/norm(v1)/norm(v2))];
            norm_v1_on_v2=[norm_v1_on_v2 dot(v1,v2)/norm(v2)];
            norm_v2_on_v1=[norm_v2_on_v1 dot(v1,v2)/norm(v1)];
        end
    end
    temp=zeros(1,time_left_longest+1+time_right_longest+1);
    for i=1:size(angl_v1_v2,2)
        temp(1,timer1(i))=angl_v1_v2(i);
    end
    feature_angle_distribution=[feature_angle_distribution; temp];
    clear temp;
    plot(timer1(1:size(angl_v1_v2,2)),angl_v1_v2,'k');
    %line([time_detect time_detect],[0 max(max(v1_norm),max(v1_norm))],'color','r');
    line([time_strategy time_strategy],[0 max(max(angl_v1_v2),max(angl_v1_v2))],'color','g');
    line([time_crash time_crash],[0 max(max(angl_v1_v2),max(angl_v1_v2))],'color','b');
end
figure(2);
histogram(feature_time_dist(:,1));
figure(3);
histogram(feature_time_dist(:,2));

clear temp_record_crash_washed;
clear time_detect velocity1 velocity2 start_time1 start_time2 ans acc1 acc2 acc1_d acc2_d;
clear outer_index pair time_crash index1 index2 states1 states2 timer1 timer2 end_time1 end_time2;
clear i;
clear v1_norm acc1_norm  acc1_on_v1_past  acc1_on_v1_past_norm  r1  v1_ang  acc1_ang;
clear v2_norm acc2_norm  acc2_on_v2_past  acc2_on_v2_past_norm  r2  v2_ang  acc2_ang;
clear time_pad acc1_norm_d acc1_norm acc1_tan acc1_tan_d;
clear v1 v2 s1 s2 timer_both dur_time p1_crash dist_s1_s2 angl_v1_v2 norm_v1_on_v2 norm_v2_on_v1 dist_s1_crash_point;


