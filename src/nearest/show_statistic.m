function valid = show_statistic(trackerW, pair ,delta_time)
    time_nearest=pair(1);
    index1=pair(2);
    index2=pair(3);
    states1=trackerW(index1).states(1:3,:);
    states2=trackerW(index2).states(1:3,:);
    timer1=trackerW(index1).start:trackerW(index1).end;
    timer2=trackerW(index2).start:trackerW(index2).end;
    time_both_start=max(timer1(1),timer2(1));
    time_both_end=min(timer1(end),timer2(end));
    
    if delta_time~=0
        delta_time=delta_time+2;
        start_time1=max(timer1(1),time_nearest-delta_time);
        start_time2=max(timer2(1),time_nearest-delta_time);
        end_time1=min(timer1(end),time_nearest+delta_time);
        end_time2=min(timer2(end),time_nearest+delta_time);
        states1=states1(1:3,find(timer1==start_time1):find(timer1==end_time1));
        states2=states2(1:3,find(timer2==start_time2):find(timer2==end_time2));
        timer1=timer1(find(timer1==start_time1):find(timer1==end_time1));
        timer2=timer2(find(timer2==start_time2):find(timer2==end_time2));
    end
    
    velocity1=states1(1:3,2:end)-states1(1:3,1:end-1);
    velocity2=states2(1:3,2:end)-states2(1:3,1:end-1);
    acc1=velocity1(1:3,2:end)-velocity1(1:3,1:end-1);
    acc2=velocity2(1:3,2:end)-velocity2(1:3,1:end-1);

    
    figure(2);
    %figure('visible','off')
    %start calculate
    v1_norm=[];
    acc1_norm=[];
    v1_on_v1_past=[];
    acc1_on_v1_past=[];
    v1_ang=[];
    acc1_ang=[];
    acc1_on_v1_past_norm=[];
    dist_s1_crash_point=[];
    
    r1=[];
    av1=[];
%     r1_verify=[];
%     for i=2:min(size(acc1,2)-1,dur_time+10)
%         s1=states1(1:3,i);
%         s1_past=states1(1:3,i-1);
%         s1_next=states1(1:3,i+1);
%         vec1=s1-s1_past;
%         vec2=s1_next-s1_past;
%         vec3=s1_next-s1;
% %         a=norm(vec1); b=norm(vec2); c=norm(vec3);
% %         p=(a+b+c)/2;
% %         rrr=(a*b*c)/(4*sqrt(p*(p-a)*(p-b)*(p-c)));
% %         r1_verify=[r1_verify rrr];
%         sin_alpha=norm(cross(vec1,vec2))/(norm(vec1)*norm(vec2));
%         av1=[av1; asin(sin_alpha)];
%         r1=[r1; norm(vec3)/(2*sin_alpha)];
%     end
    for i=1:size(acc1,2)
        v1=velocity1(1:3,i);
        v1_past=velocity1(1:3,i);%与加速度有关的用当前帧v
        a1=acc1(1:3,i);
        
        %=====计算速度和加速度大小======
        v1_norm=[v1_norm; norm(velocity1(1:3,i))];
        acc1_norm=[acc1_norm; norm(acc1(1:3,i))];
        
        %======计算加速度在在上一帧速度上投影大小====
        acc1_on_v1_past=[acc1_on_v1_past; dot(a1,v1_past)/norm(v1_past)];
        %======计算加速度法向量================
        acc1_on_v1_past_norm=[acc1_on_v1_past_norm; sqrt(norm(a1)^2-acc1_on_v1_past(end)^2)];
        %===========计算运动曲率半径===============
        r1=[r1 norm(v1)^2/acc1_on_v1_past_norm(end)];
        
        
    end
        
        
    for i=2:size(acc1,2)
        %==========简记变量初始化=========
        v1=velocity1(1:3,i);
        v1_past=velocity1(1:3,i-1);
        a1=acc1(1:3,i);
        a1_1=acc1(1:3,i-1);
        
        %======计算速度相对上一帧速度的变化角度==========
        v1_ang=[v1_ang; acosd(dot(v1,v1_past)/norm(v1)/norm(v1_past))];  
        %======计算加速度相对上一帧加速度的变化角度==========
        acc1_ang=[acc1_ang; acosd(dot(a1,a1_1)/norm(a1)/norm(a1_1))];
    end
    
    
    v2_norm=[];
    acc2_norm=[];
    v2_on_v2_past=[];
    acc2_on_v2_past=[];
    v2_ang=[];
    acc2_ang=[];
    acc2_on_v2_past_norm=[];
    dist_s2_crash_point=[];
    
    r2=[];
    av2=[];
%     for i=2:min(size(acc2,2)-1,dur_time+10)
%         s2=states2(1:3,i);
%         s2_past=states2(1:3,i-1);
%         s2_next=states2(1:3,i+1);
%         vec1=s2-s2_past;
%         vec2=s2_next-s2_past;
%         vec3=s2_next-s2;
%         sin_alpha=norm(cross(vec1,vec2))/(norm(vec1)*norm(vec2));
%         av2=[av2; asin(sin_alpha)];
%         r2=[r2; norm(vec3)/(2*sin_alpha)];
%     end
    for i=1:size(acc2,2)
        v2=velocity2(1:3,i);
        v2_past=velocity2(1:3,i);
        a2=acc2(1:3,i);   
        %=====计算速度和加速度大小======
        v2_norm=[v2_norm; norm(velocity2(1:3,i))];
        acc2_norm=[acc2_norm; norm(acc2(1:3,i))];

        %======计算加速度切向量===============
        acc2_on_v2_past=[acc2_on_v2_past; dot(a2,v2_past)/norm(v2_past)];
        %======计算加速度法向量================
        acc2_on_v2_past_norm=[acc2_on_v2_past_norm; sqrt(norm(a2)^2-(dot(a2,v2_past)/norm(v2_past))^2)];
        %===========计算运动曲率半径===============
        r2=[r2 norm(v2)^2/acc2_on_v2_past_norm(end)];
    end
    
    for i=2:size(acc2,2)
        v2=velocity2(1:3,i);
        v2_past=velocity2(1:3,i-1);
        a2=acc2(1:3,i);
        a2_past=acc2(1:3,i-1);
       
        %======计算速度相对上一帧速度的变化角度==========
        v2_ang=[v2_ang; acosd(dot(v2,v2_past)/norm(v2)/norm(v2_past))];
        %======计算加速度相对上一帧加速度的变化角度==========
        acc2_ang=[acc2_ang; acosd(dot(a2,a2_past)/norm(a2)/norm(a2_past))];
    end
    
    %-----calc distance----------
    timer_both=max(timer1(1),timer2(1)):min(timer1(end),timer2(end));
    dist_s1_s2=[];
    for i=1:size(timer_both,2)
        s1=states1(1:3,find(timer1==timer_both(i)));
        s2=states2(1:3,find(timer2==timer_both(i)));
        dist_s1_s2=[dist_s1_s2; norm(s1-s2)];
    end
    
    %---------------------start draw---------------
    figure(2);
    subplot(3,3,1);
    hold off;
    plot(timer1(1:end-2),v1_norm,'r');
    hold on;
    plot(timer2(1:end-2),v2_norm,'b');
    xlabel('time(frame)');
    ylabel('velocity(mm/frame)');
    title('速度大小');
    line([time_nearest time_nearest],[0 max(max(v1_norm),max(v2_norm))],'color','k');
    
    subplot(3,3,2);
    hold off;
    plot(timer1(1:end-2),acc1_norm,'r');
    hold on;
    plot(timer2(1:end-2),acc2_norm,'b');
    xlabel('time(frame)');
    ylabel('acceleration(mm/frame^2)');
    title('加速度大小');
    line([time_nearest time_nearest],[0 max(max(acc1_norm),max(acc2_norm))],'color','k');
    
    subplot(3,3,4);
    hold off;
    plot(timer1(2:end-2),v1_ang,'r')
    hold on;
    plot(timer2(2:end-2),v2_ang,'b');
    xlabel('time(frame)');
    ylabel('angle(degree)');
    title('速度与上一帧速度的角度');
    line([time_nearest time_nearest],[0 max(max(v1_ang),max(v2_ang))],'color','k');
    
    subplot(3,3,5);
    hold off;
    plot(timer1(2:end-2),acc1_ang,'r')
    hold on;
    plot(timer2(2:end-2),acc2_ang,'b');
    xlabel('time(frame)');
    ylabel('angle(degree)');
    title('加速度与上一帧加速度的角度');
    line([time_nearest time_nearest],[0 max(max(acc1_ang),max(acc2_ang))],'color','k');
    
    subplot(3,3,7);
    hold off;
    plot(timer1(1:end-2),acc1_on_v1_past,'r')
    hold on;
    plot(timer2(1:end-2),acc2_on_v2_past,'b');
    xlabel('time(frame)');
    ylabel('acc(mm/frame^2)');
    title('切向加速度');
    line([time_nearest time_nearest],[min(min(acc1_on_v1_past),min(acc2_on_v2_past))-1 max(max(acc1_on_v1_past),max(acc2_on_v2_past))+1],'color','k');
    
    subplot(3,3,8);
    hold off;
    plot(timer1(1:end-2),acc1_on_v1_past_norm,'r')
    hold on;
    plot(timer2(1:end-2),acc2_on_v2_past_norm,'b');
    xlabel('time(frame)');
    ylabel('acc(mm/frame^2)');
    title('法向加速度');
    line([time_nearest time_nearest],[min(min(acc1_on_v1_past_norm),min(acc2_on_v2_past_norm))-1 max(max(acc1_on_v1_past_norm),max(acc2_on_v2_past_norm))+1],'color','k');
    
    subplot(3,3,6);
    hold off;
    plot(timer_both,dist_s1_s2,'k')
    hold on;
    %plot(acc2_on_v2_1,'b');
    xlabel('time(frame)');
    ylabel('distance(mm)');
    title('果蝇间距离');
    line([time_nearest time_nearest],[0 max(dist_s1_s2)],'color','k');
    
    subplot(3,3,3);
    hold off;
    plot(timer1(1:end-2),r1,'r');
    hold on;
    plot(timer2(1:end-2),r2,'b');
    xlabel('time(frame)');
    ylabel('R(mm)');
    title('曲率半径');
    line([time_nearest time_nearest],[0 max(max(r1),max(r2))],'color','k');
    
end

