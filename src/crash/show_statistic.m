function [ valid ] = show_statistic( trackerW,pair, left_delta_time,right_delta_time )
    time_crash=round(pair.time_end);
    time_detect=pair.time_start;
    index1=pair.id(1);
    index2=pair.id(2);
    states1=trackerW(index1).states(1:3,:);
    states2=trackerW(index2).states(1:3,:);
    timer1=trackerW(index1).start:trackerW(index1).end;
    timer2=trackerW(index2).start:trackerW(index2).end;
    
    if left_delta_time~=0 || right_delta_time~=0
        start_time1=min(time_detect,max(timer1(1),time_crash-left_delta_time));
        end_time1=min(timer1(end),time_crash+right_delta_time);
        states1=states1(1:3,find(timer1==start_time1):find(timer1==end_time1));
        timer1=timer1(find(timer1==start_time1):find(timer1==end_time1));   
        
        start_time2=min(time_detect,max(timer2(1),time_crash-left_delta_time));
        end_time2=min(timer2(end),time_crash+right_delta_time);
        states2=states2(1:3,find(timer2==start_time1):find(timer2==end_time1));
        timer2=timer2(find(timer2==start_time1):find(timer2==end_time1));  
    end
    
    velocity1=states1(1:3,2:end)-states1(1:3,1:end-1);
    velocity2=states2(1:3,2:end)-states2(1:3,1:end-1);
    acc1=velocity1(1:3,2:end)-velocity1(1:3,1:end-1);
    acc2=velocity2(1:3,2:end)-velocity2(1:3,1:end-1);

    %find crash position
    dur_time=pair.delta_time;
    v1=pair.velocity(1:3,1);
    v2=pair.velocity(1:3,2);
    p1_crash=states1(1:3,find(timer1==time_detect))+v1*dur_time;
    p2_crash=states2(1:3,find(timer2==time_detect))+v2*dur_time;
    
    %filter by n, need n>4
    if dur_time<4
        valid=0;
        return ;
    end
    
    %filter by minium speed
    minium_speed=2;
    for i=1:size(velocity1,2)
        if norm(velocity1(1:3,i))<minium_speed
            valid=0;
            return ;
        end
    end
    for i=1:size(velocity2,2)
        if norm(velocity2(1:3,i))<minium_speed
            valid=0;
            return ;
        end
    end


    [ v1_norm,acc1_norm, acc1_on_v1_past, acc1_on_v1_past_norm, r1, v1_ang, acc1_ang] = calc_trace_attribute( states1 );
    
    [ v2_norm,acc2_norm, acc2_on_v2_past, acc2_on_v2_past_norm, r2, v2_ang, acc2_ang] = calc_trace_attribute( states2 );
    
    dist_s1_crash_point=[];
    for i=1:size(timer1,2);
        %==========简记变量初始化=========
        s1=states1(1:3,i);  
        %======计算和虚拟碰撞点的距离===========
        dist_s1_crash_point=[dist_s1_crash_point; norm(s1-p1_crash)];     
    end
    
    dist_s2_crash_point=[];
    for i=1:size(timer2,2)
        s2=states2(1:3,i);
        %======计算和虚拟碰撞点的距离===========
        dist_s2_crash_point=[dist_s2_crash_point; norm(s2-p2_crash)];
    end
    
    timer_both=max(timer1(1),timer2(1)):min(timer1(end),timer2(end));
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
    
    global feature_angle_distribution;
    tmp=zeros(left_delta_time*3+right_delta_time,20);
    for i=1:size(timer_both,2)-1
        tmp_time=timer_both(i)-time_crash+left_delta_time*2;
        tmp_ang=floor(angl_v1_v2(i)/10)+1;
        if tmp_time>0
            tmp(tmp_time,tmp_ang)=tmp(tmp_time,tmp_ang)+1;
        end
    end
    feature_angle_distribution=feature_angle_distribution+tmp;
    
    figure(2);
    
    %grid on;
    
    subplot(3,3,1);
    hold off;
    plot(timer1(1:end-2),v1_norm,'r');
    hold on;
    plot(timer2(1:end-2),v2_norm,'b');
    xlabel('time(frame)');
    ylabel('velocity(mm/frame)');
    title('速度大小');
    line([time_crash time_crash],[0 max(max(v1_norm),max(v2_norm))],'color','k');
    %legend('First','Second');
    
    subplot(3,3,2);
    hold off;
    plot(timer1(1:end-2),acc1_norm,'r');
    hold on;
    plot(timer2(1:end-2),acc2_norm,'b');
    xlabel('time(frame)');
    ylabel('acceleration(mm/frame^2)');
    title('加速度大小');
    line([time_crash time_crash],[0 max(max(acc1_norm),max(acc2_norm))],'color','k');

    subplot(3,3,3);
    hold off;
    plot(timer_both(1:end-1),norm_v1_on_v2,'r');
    hold on;
    plot(timer_both(1:end-1),norm_v2_on_v1,'b');
    xlabel('time(frame)');
    ylabel('speed(mm/f)');
    title('速度投影模长');
    line([time_crash time_crash],[min(min(norm_v1_on_v2),min(norm_v2_on_v1))-1 max(max(norm_v1_on_v2),max(norm_v2_on_v1))+1],'color','k');

    subplot(3,3,4);
    hold off;
    plot(timer1(2:end-2),v1_ang,'r')
    hold on;
    plot(timer2(2:end-2),v2_ang,'b');
    xlabel('time(frame)');
    ylabel('angle(degree)');
    title('速度与上一帧速度的角度');
    line([time_crash time_crash],[0 max(max(v1_ang),max(v2_ang))],'color','k');
    
    subplot(3,3,5);
    hold off;
    plot(timer_both(1:end-1),angl_v1_v2,'k')
    hold on;
    %plot(acc2_on_v2_1,'b');
    xlabel('time(frame)');
    ylabel('angle(degree)');
    title('果蝇速度夹角');
    line([time_crash time_crash],[0 max(angl_v1_v2)],'color','k');
    
        
    subplot(3,3,6);
    hold off;
    plot(timer_both,dist_s1_s2,'k')
    hold on;
    %plot(acc2_on_v2_1,'b');
    xlabel('time(frame)');
    ylabel('distance(mm)');
    title('果蝇间距离');
    line([time_crash time_crash],[0 max(dist_s1_s2)],'color','k');
    
    subplot(3,3,7);
    hold off;
    plot(timer1(1:end-2),acc1_on_v1_past,'r')
    hold on;
    plot(timer2(1:end-2),acc2_on_v2_past,'b');
    xlabel('time(frame)');
    ylabel('acc(mm/frame^2)');
    title('切向加速度');
    line([time_crash time_crash],[min(min(acc1_on_v1_past),min(acc2_on_v2_past))-1 max(max(acc1_on_v1_past),max(acc2_on_v2_past))+1],'color','k');

    %legend('First','Second');
    

    subplot(3,3,8);
    hold off;
    plot(timer1(1:end-2),acc1_on_v1_past_norm,'r')
    hold on;
    plot(timer2(1:end-2),acc2_on_v2_past_norm,'b');
    xlabel('time(frame)');
    ylabel('acc(mm/frame^2)');
    title('法向加速度');
    line([time_crash time_crash],[min(min(acc1_on_v1_past_norm),min(acc2_on_v2_past_norm))-1 max(max(acc1_on_v1_past_norm),max(acc2_on_v2_past_norm))+1],'color','k');
    
    subplot(3,3,9);
    hold off;
    plot(timer1(1:end),dist_s1_crash_point,'r');
    hold on;
    plot(timer2(1:end),dist_s2_crash_point,'b');
    xlabel('time(frame)');
    ylabel('distance(mm)');
    title('果蝇与碰撞点的距离');
    line([time_crash time_crash],[0 max(max(dist_s1_crash_point),max(dist_s2_crash_point))],'color','k');

    valid=1;
end

