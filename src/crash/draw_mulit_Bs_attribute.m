for index1=1:size(trackerW,2)
    states1=trackerW(index1).states(1:3,:);
    velocity1=states1(1:3,2:end)-states1(1:3,1:end-1);
    timer1=trackerW(index1).start:trackerW(index1).end;   
    Bs=trackerW(index1).Bs;
    if size(Bs,1)<5 
        continue; 
    end;
    
    [ v1_norm,acc1_norm, acc1_on_v1_past, acc1_on_v1_past_norm, r1, v1_ang, acc1_ang] = calc_trace_attribute( states1 );
    
    figure(1);

    subplot(3,3,1);
    xlabel('time(frame)');
    ylabel('velocity(mm/frame)');
    title('速度大小');
    hold off;
    plot(timer1(1:end-2),v1_norm,'r');
    hold on;
    
    subplot(3,3,2);
    xlabel('time(frame)');
    ylabel('acceleration(mm/frame^2)');
    title('加速度大小');
    hold off;
    plot(timer1(1:end-2),acc1_norm,'r');
    hold on;

    subplot(3,3,3);
    hold off;
    plot(0,0,'r');
    hold on;
    xlabel('time(frame)');
    ylabel('speed(mm/f)');
    title('速度投影模长');
    
    subplot(3,3,4);
    hold off;
    plot(timer1(2:end-2),v1_ang,'r')
    hold on;
    xlabel('time(frame)');
    ylabel('angle(degree)');
    title('速度与上一帧速度的角度');
    
    subplot(3,3,5);
    xlabel('time(frame)');
    ylabel('angle(degree)');
    title('果蝇速度夹角');
    hold off;
    plot(0,0,'k')
    hold on;


    subplot(3,3,6);
    hold off;
    plot(0,0,'k')
    hold on;
    %plot(acc2_on_v2_1,'b');
    xlabel('time(frame)');
    ylabel('distance(mm)');
    title('果蝇间距离');
    
    
    subplot(3,3,7);
    hold off;
    plot(timer1(1:end-2),acc1_on_v1_past,'r')
    hold on;
    xlabel('time(frame)');
    ylabel('acc(mm/frame^2)');
    title('切向加速度');
   
    subplot(3,3,8);
    hold off;
    plot(timer1(1:end-2),acc1_on_v1_past_norm,'r')
    hold on;
    xlabel('time(frame)');
    ylabel('acc(mm/frame^2)');
    title('法向加速度');
      
    subplot(3,3,9);
    hold off;
    plot(0,0,'r');
    hold on;
    xlabel('time(frame)');
    ylabel('distance(mm)');
    title('果蝇与碰撞点的距离');

   
    for Bs_ind=1:size(Bs,1)
        %index2 mean the index of fly_b
        index2=Bs(Bs_ind).index_b;
        time_detect=Bs(Bs_ind).time_detect;
        time_crash=ceil(Bs(Bs_ind).time_end);
        time_strategy=Bs(Bs_ind).time_strategy;
        pos_crash_A=Bs(Bs_ind).pos_a;
              
        states2=trackerW(index2).states(1:3,:);
        velocity2=states2(1:3,2:end)-states2(1:3,1:end-1);
        timer2=trackerW(index2).start:trackerW(index2).end;   
        [ v2_norm,acc2_norm, acc2_on_v2_past, acc2_on_v2_past_norm, r2, v2_ang, acc2_ang] = calc_trace_attribute( states2 );
  
        dist_s2_crash_point=[];
        for i=1:size(timer2,2)
            s2=states2(1:3,i);
            %======计算和虚拟碰撞点的距离===========
            dist_s2_crash_point=[dist_s2_crash_point; norm(s2-pos_crash_A)];
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

            if i<size(timer_both,2)               
                v1=velocity1(1:3,find(timer1==timer_both(i)));            
                v2=velocity2(1:3,find(timer2==timer_both(i)));
                angl_v1_v2=[angl_v1_v2 acosd(dot(v1,v2)/norm(v1)/norm(v2))];
                norm_v1_on_v2=[norm_v1_on_v2 dot(v1,v2)/norm(v2)];
                norm_v2_on_v1=[norm_v2_on_v1 dot(v1,v2)/norm(v1)];
            end
        end
        
        subplot(3,3,1);
        plot(timer2(1:end-2),v2_norm,'b');
        line([time_detect time_detect],[0 max(max(v1_norm),max(v2_norm))],'color','k');
        plot(time_detect,max(max(v1_norm),max(v2_norm)),'b+');
        line([time_strategy time_strategy],[0 max(max(v1_norm),max(v2_norm))],'color','k');
        plot(time_strategy,max(max(v1_norm),max(v2_norm)),'rh');
        
        subplot(3,3,2);
        plot(timer2(1:end-2),acc2_norm,'b');
        line([time_detect time_detect],[min(min(norm_v1_on_v2),min(norm_v2_on_v1))-1 max(max(norm_v1_on_v2),max(norm_v2_on_v1))+1],'color','k');
        plot(time_detect,max(max(norm_v1_on_v2),max(norm_v2_on_v1))+1,'b+');
        line([time_strategy time_strategy],[min(min(norm_v1_on_v2),min(norm_v2_on_v1))-1 max(max(norm_v1_on_v2),max(norm_v2_on_v1))+1],'color','k');
        plot(time_strategy,max(max(norm_v1_on_v2),max(norm_v2_on_v1))+1,'rh');
        
        subplot(3,3,3);
        plot(timer_both(1:end-1),norm_v2_on_v1,'b');
        line([time_detect time_detect],[0 max(max(acc1_norm),max(acc2_norm))],'color','k');
        plot(time_detect,max(max(acc1_norm),max(acc2_norm)),'b+');
        line([time_strategy time_strategy],[0 max(max(acc1_norm),max(acc2_norm))],'color','k');
        plot(time_strategy,max(max(acc1_norm),max(acc2_norm)),'rh');
        
        
        subplot(3,3,4);
        plot(timer2(2:end-2),v2_ang,'b');
        line([time_detect time_detect],[0 max(max(v1_ang),max(v2_ang))],'color','k');
        plot(time_detect,max(max(v1_ang),max(v2_ang)),'b+');
        line([time_strategy time_strategy],[0 max(max(v1_ang),max(v2_ang))],'color','k');
        plot(time_strategy,max(max(v1_ang),max(v2_ang)),'rh');
        
        
        
        subplot(3,3,5);
        plot(timer_both(1:end-1),angl_v1_v2,'k');
        line([time_detect time_detect],[0 max(angl_v1_v2)],'color','k');
        plot(time_detect,max(angl_v1_v2),'b+');
        line([time_strategy time_strategy],[0 max(angl_v1_v2)],'color','k');
        plot(time_strategy,max(angl_v1_v2),'rh');
        
        subplot(3,3,6);
        plot(timer_both,dist_s1_s2,'k');
        line([time_detect time_detect],[0 max(dist_s1_s2)],'color','k');
        plot(time_detect,max(dist_s1_s2),'b+');
        line([time_strategy time_strategy],[0 max(dist_s1_s2)],'color','k');
        plot(time_strategy,max(dist_s1_s2),'rh');
        
        subplot(3,3,7);
        plot(timer2(1:end-2),acc2_on_v2_past,'b');
        line([time_detect time_detect],[min(min(acc1_on_v1_past),min(acc2_on_v2_past))-1 max(max(acc1_on_v1_past),max(acc2_on_v2_past))+1],'color','k');
        plot(time_detect,max(max(acc1_on_v1_past),max(acc2_on_v2_past))+1,'b+');
        line([time_strategy time_strategy],[min(min(acc1_on_v1_past),min(acc2_on_v2_past))-1 max(max(acc1_on_v1_past),max(acc2_on_v2_past))+1],'color','k');
        plot(time_strategy,max(max(acc1_on_v1_past),max(acc2_on_v2_past))+1,'rh');
        
        
        subplot(3,3,8);
        plot(timer2(1:end-2),acc2_on_v2_past_norm,'b');
        line([time_detect time_detect],[min(min(acc1_on_v1_past_norm),min(acc2_on_v2_past_norm))-1 max(max(acc1_on_v1_past_norm),max(acc2_on_v2_past_norm))+1],'color','k');
        plot(time_detect,max(max(acc1_on_v1_past_norm),max(acc2_on_v2_past_norm))+1,'b+');
        line([time_strategy time_strategy],[min(min(acc1_on_v1_past_norm),min(acc2_on_v2_past_norm))-1 max(max(acc1_on_v1_past_norm),max(acc2_on_v2_past_norm))+1],'color','k');
        plot(time_strategy,max(max(acc1_on_v1_past_norm),max(acc2_on_v2_past_norm))+1,'rh');
        
        
        subplot(3,3,9);
        plot(timer2(1:end),dist_s2_crash_point,'b');
        line([time_detect time_detect],[0 max(dist_s2_crash_point)],'color','k');
        plot(time_detect,max(dist_s2_crash_point),'b+');
        line([time_strategy time_strategy],[0 max(dist_s2_crash_point)],'color','k');
        plot(time_strategy,max(dist_s2_crash_point),'rh');
        
        
    end;
end;