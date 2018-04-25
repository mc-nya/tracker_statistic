function [ output_args ] = show_statistic(  trackerW,record_crash, k  )
id=record_crash(k).id;
    t_start=record_crash(k).time_start;
    t_end=round(record_crash(k).time_end);
    t_end1=max(min(trackerW(id(1)).end,t_end),trackerW(id(1)).end);
    t_end2=max(min(trackerW(id(2)).end,t_end),trackerW(id(2)).end);
    tracker1_start=trackerW(id(1)).start;
    tracker2_start=trackerW(id(2)).start;
    states1=trackerW(id(1)).smoothed_states(1:3,t_start-tracker1_start+1:t_end1-tracker1_start+1);
    states2=trackerW(id(2)).smoothed_states(1:3,t_start-tracker2_start+1:t_end2-tracker2_start+1);
    
    length=min(size(states1,2),size(states2,2));
    totaldist=0;
    for i=1:length
        dist=distance(states1(:,i),states2(:,i));
        totaldist=totaldist+dist;
    end
    totaldist=totaldist/length;
    if totaldist<5 || size(states1,2)<5 || size(states2,2)<5
        output_args=0;
        return ;
    end    
    
    velocity1=states1(1:3,2:end)-states1(1:3,1:end-1);
    velocity2=states2(1:3,2:end)-states2(1:3,1:end-1);
    acc1=velocity1(1:3,2:end)-velocity1(1:3,1:end-1);
    acc2=velocity2(1:3,2:end)-velocity2(1:3,1:end-1);

    %find crash position
    dur_time=record_crash(k).delta_time;
    v1=record_crash(k).velocity(1:3,1);
    v2=record_crash(k).velocity(1:3,2);
    p1_crash=states1(1:3,1)+v1*dur_time;
    p2_crash=states2(1:3,1)+v2*dur_time;
    
    %filter by n, need n>4
    if dur_time<4
        output_args=0;
        return ;
    end
    
    %filter by minium speed
    minium_speed=2;
    for i=1:size(velocity1,2)
        if norm(velocity1(1:3,i))<minium_speed
            output_args=0;
            return ;
        end
    end
    for i=1:size(velocity2,2)
        if norm(velocity2(1:3,i))<minium_speed
            output_args=0;
            return ;
        end
    end


    
    
    
    
    figure(2);
    %figure('visible','off')
    %start calculate
    v1_norm=[];
    acc1_norm=[];
    v1_on_v1_past=[];
    acc1_on_v1_past=[];
    v1_ang=[];
    acc1_ang=[];
    
    dist_s1_crash_point=[];
    
    for i=2:min(size(acc1,2),dur_time+10)
        %==========��Ǳ�����ʼ��=========
        s1=states1(1:3,i);
        s1_past=states1(1:3,i-1);
        v1=velocity1(1:3,i);
        v1_past=velocity1(1:3,i-1);
        a1=acc1(1:3,i);
        a1_1=acc1(1:3,i-1);
        
        %=====�����ٶȺͼ��ٶȴ�С======
        v1_norm=[v1_norm; norm(velocity1(1:3,i))];
        acc1_norm=[acc1_norm; norm(acc1(1:3,i))];
        
        %=====�����ٶ�����һ֡�ٶ���ͶӰ��С====
        v1_on_v1_past=[v1_on_v1_past; dot(v1,v1_past)/norm(v1_past)];
        %======������ٶ�������һ֡�ٶ���ͶӰ��С====
        acc1_on_v1_past=[acc1_on_v1_past; dot(a1,v1_past)/norm(v1_past)];
        
        %======�����ٶ������һ֡�ٶȵı仯�Ƕ�==========
        v1_ang=[v1_ang; acosd(dot(v1,v1_past)/norm(v1)/norm(v1_past))];
        %======������ٶ������һ֡���ٶȵı仯�Ƕ�==========
        acc1_ang=[acc1_ang; acosd(dot(a1,a1_1)/norm(a1)/norm(a1_1))];
        
        %======�����������ײ��ľ���===========
        dist_s1_crash_point=[dist_s1_crash_point; norm(s1-p1_crash)];
        
    end
    
    
    v2_norm=[];
    acc2_norm=[];
    v2_on_v2_1=[];
    acc2_on_v2_1=[];
    v2_ang=[];
    acc2_ang=[];
    dist_s2_crash_point=[];
    for i=2:min(size(acc2,2),dur_time+10)
        s2=states2(1:3,i);
        s2_1=states2(1:3,i-1);
        v2=velocity2(1:3,i);
        v2_1=velocity2(1:3,i-1);
        a2=acc2(1:3,i);
        a2_1=acc2(1:3,i-1);
        %=====�����ٶȺͼ��ٶȴ�С======
        v2_norm=[v2_norm; norm(velocity2(1:3,i))];
        acc2_norm=[acc2_norm; norm(acc2(1:3,i))];
        %=====�����ٶ�����һ֡�ٶ���ͶӰ��С====
        v2_on_v2_1=[v2_on_v2_1; dot(v2,v2_1)/norm(v2_1)];
        %======������ٶ�������һ֡�ٶ���ͶӰ��С====
        acc2_on_v2_1=[acc2_on_v2_1; dot(a2,v2_1)/norm(v2_1)];

        %======�����ٶ������һ֡�ٶȵı仯�Ƕ�==========
        v2_ang=[v2_ang; acosd(dot(v2,v2_1)/norm(v2)/norm(v2_1))];
        %======������ٶ������һ֡���ٶȵı仯�Ƕ�==========
        acc2_ang=[acc2_ang; acosd(dot(a2,a2_1)/norm(a2)/norm(a2_1))];
        
        %======�����������ײ��ľ���===========
        dist_s2_crash_point=[dist_s2_crash_point; norm(s2-p2_crash)];
    end
    
    dist_s1_s2=[];
    for i=2:min(min(size(acc1,2),size(acc2,2)),dur_time+10)
        s1=states1(1:3,i);
        s2=states2(1:3,i);
        dist_s1_s2=[dist_s1_s2; norm(s1-s2)];
    end
    
    plot(0,0);
    %grid on;
    
    subplot(3,3,1);
    hold off;
    plot(v1_norm,'r');
    hold on;
    plot(v2_norm,'b');
    xlabel('time(frame)');
    ylabel('velocity(mm/frame)');
    title('�ٶȴ�С');
    line([t_end-t_start t_end-t_start],[0 max(max(v1_norm),max(v2_norm))],'color','k');
    %legend('First','Second');
    
    subplot(3,3,2);
    hold off;
    plot(acc1_norm,'r');
    hold on;
    plot(acc2_norm,'b');
    xlabel('time(frame)');
    ylabel('acceleration(mm/frame^2)');
    title('���ٶȴ�С');
    line([t_end-t_start t_end-t_start],[0 max(max(acc1_norm),max(acc2_norm))],'color','k');
    %legend('First','Second');
    
    subplot(3,3,4);
    hold off;
    plot(v1_ang,'r')
    hold on;
    plot(v2_ang,'b');
    xlabel('time(frame)');
    ylabel('angle(degree)');
    title('�ٶ�����һ֡�ٶȵĽǶ�');
    line([t_end-t_start t_end-t_start],[0 max(max(v1_ang),max(v2_ang))],'color','k');
    %legend('First','Second');
    
    subplot(3,3,5);
    hold off;
    plot(acc1_ang,'r')
    hold on;
    plot(acc2_ang,'b');
    xlabel('time(frame)');
    ylabel('angle(degree)');
    title('���ٶ�����һ֡���ٶȵĽǶ�');
    line([t_end-t_start t_end-t_start],[0 max(max(acc1_ang),max(acc2_ang))],'color','k');
    %legend('First','Second');
    
    subplot(3,3,7);
    hold off;
    plot(v1_on_v1_past,'r')
    hold on;
    plot(v2_on_v2_1,'b');
    xlabel('time(frame)');
    ylabel('speed(mm/frame)');
    title('�ٶ�����һ֡�ٶ��ϵ�ͶӰ����');
    line([t_end-t_start t_end-t_start],[min(min(v1_on_v1_past),min(v2_on_v2_1)) max(max(v1_on_v1_past),max(v2_on_v2_1))],'color','k');
    %legend('First','Second');
    
    subplot(3,3,8);
    hold off;
    plot(acc1_on_v1_past,'r')
    hold on;
    plot(acc2_on_v2_1,'b');
    xlabel('time(frame)');
    ylabel('acc(mm/frame^2)');
    title('���ٶ�����һ֡�ٶ��ϵ�ͶӰ����');
    line([t_end-t_start t_end-t_start],[min(min(acc1_on_v1_past),min(acc2_on_v2_1)) max(max(acc1_on_v1_past),max(acc2_on_v2_1))],'color','k');
    %legend('First','Second');
    
    subplot(3,3,3);
    hold off;
    plot(dist_s1_crash_point,'r')
    hold on;
    plot(dist_s2_crash_point,'b');
    xlabel('time(frame)');
    ylabel('distance(mm)');
    title('��Ӭ����ײ��ľ���');
    line([t_end-t_start t_end-t_start],[0 max(max(dist_s1_crash_point),max(dist_s2_crash_point))],'color','k');
    %legend('First','Second');
    
    subplot(3,3,6);
    hold off;
    plot(dist_s1_s2,'k')
    hold on;
    %plot(acc2_on_v2_1,'b');
    xlabel('time(frame)');
    ylabel('distance(mm)');
    title('��Ӭ�����');
    line([t_end-t_start t_end-t_start],[0 max(dist_s1_s2)],'color','k');
    %legend('First','Second');
    
    
    output_args=1;
end

