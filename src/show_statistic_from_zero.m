function [ output_args ] = show_statistic_from_zero(  trackerW,record_crash, k  )
    id=record_crash(k).id;
    %calcute and copy trace data
    time_detect=record_crash(k).time_start;
    time_crash=round(record_crash(k).time_end);
    time_trace_end1=max(min(trackerW(id(1)).end,time_crash),trackerW(id(1)).end);
    time_trace_end2=max(min(trackerW(id(2)).end,time_crash),trackerW(id(2)).end);
    time_trace_start1=trackerW(id(1)).start;
    time_trace_start2=trackerW(id(2)).start;
    states1=trackerW(id(1)).smoothed_states(1:3,1:time_trace_end1-time_trace_start1+1);
    states2=trackerW(id(2)).smoothed_states(1:3,1:time_trace_end2-time_trace_start2+1);
    index_detect1=time_detect-time_trace_start1+1;
    index_detect2=time_detect-time_trace_start2+1;
    time_trace_length1=size(states1,2)-index_detect1+1;
    time_trace_length2=size(states2,2)-index_detect2+1;
    
    time_both_start=max(time_trace_start1,time_trace_start2);
    time_both_end=min(time_trace_end1,time_trace_end2);
    %filter dirty data
    %calc from crash detect time
    length=min(time_trace_length1,time_trace_length2);
    totaldist=0;
    for i=0:length-1
        dist=distance(states1(:,index_detect1+i),states2(:,index_detect2+i));
        totaldist=totaldist+dist;
    end
    totaldist=totaldist/length;
    if totaldist<5 || time_trace_length1<5 || time_trace_length2<5
        output_args=0;
        return ;
    end    
    
    velocity1=states1(1:3,2:end)-states1(1:3,1:end-1);
    velocity2=states2(1:3,2:end)-states2(1:3,1:end-1);
    acc1=velocity1(1:3,2:end)-velocity1(1:3,1:end-1);
    acc2=velocity2(1:3,2:end)-velocity2(1:3,1:end-1);

    %find crash position
    dur_time=record_crash(k).delta_time;
    v1_detect=record_crash(k).velocity(1:3,1);
    v2_detect=record_crash(k).velocity(1:3,2);
    p1_crash=states1(1:3,index_detect1)+v1_detect*dur_time;
    p2_crash=states2(1:3,index_detect2)+v2_detect*dur_time;
    
    %filter by n, need n>4
    if dur_time<4
        output_args=0;
        return ;
    end
    
    %filter by minium speed
    minium_speed=2;
    for i=index_detect1:size(velocity1,2)
        if norm(velocity1(1:3,i))<minium_speed
            output_args=0;
            return ;
        end
    end
    for i=index_detect2:size(velocity2,2)
        if norm(velocity2(1:3,i))<minium_speed
            output_args=0;
            return ;
        end
    end


    
    
    
    
    figure(4);
    %figure('visible','off')
    %start calculate
    v1_norm=[];
    acc1_norm=[];
    v1_on_v1_past=[];
    acc1_on_v1_past=[];
    v1_ang=[];
    acc1_ang=[];
    timer1=[];
    dist_s1_crash_point=[];
    
    for i=2:min(size(acc1,2),index_detect1+dur_time+10)
        %==========简记变量初始化=========
        s1=states1(1:3,i);
        s1_past=states1(1:3,i-1);
        v1_detect=velocity1(1:3,i);
        v1_past=velocity1(1:3,i-1);
        a1=acc1(1:3,i);
        a1_1=acc1(1:3,i-1);
        
        timer1=[timer1; i+time_trace_start1];
        %=====计算速度和加速度大小======
        v1_norm=[v1_norm; norm(velocity1(1:3,i))];
        acc1_norm=[acc1_norm; norm(acc1(1:3,i))];
        
        %=====计算速度在上一帧速度上投影大小====
        v1_on_v1_past=[v1_on_v1_past; dot(v1_detect,v1_past)/norm(v1_past)];
        %======计算加速度在在上一帧速度上投影大小====
        acc1_on_v1_past=[acc1_on_v1_past; dot(a1,v1_past)/norm(v1_past)];
        
        %======计算速度相对上一帧速度的变化角度==========
        v1_ang=[v1_ang; acosd(dot(v1_detect,v1_past)/norm(v1_detect)/norm(v1_past))];
        %======计算加速度相对上一帧加速度的变化角度==========
        acc1_ang=[acc1_ang; acosd(dot(a1,a1_1)/norm(a1)/norm(a1_1))];
        
        %======计算和虚拟碰撞点的距离===========
        dist_s1_crash_point=[dist_s1_crash_point; norm(s1-p1_crash)];
        
    end
    
    
    v2_norm=[];
    acc2_norm=[];
    v2_on_v2_1=[];
    acc2_on_v2_1=[];
    v2_ang=[];
    acc2_ang=[];
    timer2=[];
    dist_s2_crash_point=[];
    for i=2:min(size(acc2,2),index_detect2+dur_time+10)
        s2=states2(1:3,i);
        s2_1=states2(1:3,i-1);
        v2_detect=velocity2(1:3,i);
        v2_1=velocity2(1:3,i-1);
        a2=acc2(1:3,i);
        a2_1=acc2(1:3,i-1);
        
        timer2=[timer2; i+time_trace_start2];
        %=====计算速度和加速度大小======
        v2_norm=[v2_norm; norm(velocity2(1:3,i))];
        acc2_norm=[acc2_norm; norm(acc2(1:3,i))];
        %=====计算速度在上一帧速度上投影大小====
        v2_on_v2_1=[v2_on_v2_1; dot(v2_detect,v2_1)/norm(v2_1)];
        %======计算加速度在在上一帧速度上投影大小====
        acc2_on_v2_1=[acc2_on_v2_1; dot(a2,v2_1)/norm(v2_1)];

        %======计算速度相对上一帧速度的变化角度==========
        v2_ang=[v2_ang; acosd(dot(v2_detect,v2_1)/norm(v2_detect)/norm(v2_1))];
        %======计算加速度相对上一帧加速度的变化角度==========
        acc2_ang=[acc2_ang; acosd(dot(a2,a2_1)/norm(a2)/norm(a2_1))];
        
        %======计算和虚拟碰撞点的距离===========
        dist_s2_crash_point=[dist_s2_crash_point; norm(s2-p2_crash)];
    end
    
    
    dist_s1_s2=[];
    timer_both=[];
    
    for i=time_both_start:min(min(size(acc1,2)+time_trace_start1-1,size(acc2,2)+time_trace_start2-1),time_crash+10)
        timer_both=[timer_both; i];
        
        s1=states1(1:3,i-time_trace_start1+1);
        s2=states2(1:3,i-time_trace_start2+1);
        dist_s1_s2=[dist_s1_s2; norm(s1-s2)];
    end
    
    %minus offset, so that x start from 0
    timer1=timer1-min(time_trace_start1,time_trace_start2);
    timer2=timer2-min(time_trace_start1,time_trace_start2);
    timer_both=timer_both-min(time_trace_start1,time_trace_start2);
    time_crash=time_crash-min(time_trace_start1,time_trace_start2);
    time_detect=time_detect-min(time_trace_start1,time_trace_start2);
    
    
    %grid on;
    
    subplot(3,3,1);
    hold off;
    plot(timer1,v1_norm,'r');
    hold on;
    plot(timer2,v2_norm,'b');
    xlabel('time(frame)');
    ylabel('velocity(mm/frame)');
    title('速度大小');
    line([time_crash time_crash],[0 max(max(v1_norm),max(v2_norm))],'color','k');
    line([time_detect time_detect],[0 max(max(v1_norm),max(v2_norm))],'color','k');
    %legend('First','Second');
    
    subplot(3,3,2);
    hold off;
    plot(timer1,acc1_norm,'r');
    hold on;
    plot(timer2,acc2_norm,'b');
    xlabel('time(frame)');
    ylabel('acceleration(mm/frame^2)');
    title('加速度大小');
    line([time_crash time_crash],[0 max(max(acc1_norm),max(acc2_norm))],'color','k');
    line([time_detect time_detect],[0 max(max(acc1_norm),max(acc2_norm))],'color','k');
    %legend('First','Second');
    
    subplot(3,3,4);
    hold off;
    plot(timer1,v1_ang,'r')
    hold on;
    plot(timer2,v2_ang,'b');
    xlabel('time(frame)');
    ylabel('angle(degree)');
    title('速度与上一帧速度的角度');
    line([time_crash time_crash],[0 max(max(v1_ang),max(v2_ang))],'color','k');
    line([time_detect time_detect],[0 max(max(v1_ang),max(v2_ang))],'color','k');
    %legend('First','Second');
    
    subplot(3,3,5);
    hold off;
    plot(timer1,acc1_ang,'r')
    hold on;
    plot(timer2,acc2_ang,'b');
    xlabel('time(frame)');
    ylabel('angle(degree)');
    title('加速度与上一帧加速度的角度');
    line([time_crash time_crash],[0 max(max(acc1_ang),max(acc2_ang))],'color','k');
    line([time_detect time_detect],[0 max(max(acc1_ang),max(acc2_ang))],'color','k');
    %legend('First','Second');
    
    subplot(3,3,7);
    hold off;
    plot(timer1,v1_on_v1_past,'r')
    hold on;
    plot(timer2,v2_on_v2_1,'b');
    xlabel('time(frame)');
    ylabel('speed(mm/frame)');
    title('速度在上一帧速度上的投影长度');
    line([time_crash time_crash],[min(min(v1_on_v1_past),min(v2_on_v2_1))-1 max(max(v1_on_v1_past),max(v2_on_v2_1))+1],'color','k');
    line([time_detect time_detect],[min(min(v1_on_v1_past),min(v2_on_v2_1))-1 max(max(v1_on_v1_past),max(v2_on_v2_1))+1],'color','k');
    %legend('First','Second');
    
    subplot(3,3,8);
    hold off;
    plot(timer1,acc1_on_v1_past,'r')
    hold on;
    plot(timer2,acc2_on_v2_1,'b');
    xlabel('time(frame)');
    ylabel('acc(mm/frame^2)');
    title('加速度在上一帧速度上的投影长度');
    line([time_crash time_crash],[min(min(acc1_on_v1_past),min(acc2_on_v2_1))-1 max(max(acc1_on_v1_past),max(acc2_on_v2_1))+1],'color','k');
    line([time_detect time_detect],[min(min(acc1_on_v1_past),min(acc2_on_v2_1))-1 max(max(acc1_on_v1_past),max(acc2_on_v2_1))+1],'color','k');
    %legend('First','Second');
    
    subplot(3,3,3);
    hold off;
    plot(timer1,dist_s1_crash_point,'r')
    hold on;
    plot(timer2,dist_s2_crash_point,'b');
    xlabel('time(frame)');
    ylabel('distance(mm)');
    title('果蝇与碰撞点的距离');
    line([time_crash time_crash],[0 max(max(dist_s1_crash_point),max(dist_s2_crash_point))],'color','k');
    line([time_detect time_detect],[0 max(max(dist_s1_crash_point),max(dist_s2_crash_point))],'color','k');
    %legend('First','Second');
    
    subplot(3,3,6);
    hold off;
    plot(timer_both,dist_s1_s2,'k')
    hold on;
    %plot(acc2_on_v2_1,'b');
    xlabel('time(frame)');
    ylabel('distance(mm)');
    title('果蝇间距离');
    line([time_crash time_crash],[0 max(dist_s1_s2)],'color','k');
    line([time_detect time_detect],[0 max(dist_s1_s2)],'color','k');
    %legend('First','Second');
    
    
    output_args=1;
end


