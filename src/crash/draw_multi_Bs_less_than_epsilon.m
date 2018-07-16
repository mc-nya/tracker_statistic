% run pre_process_crash.m and strategy_detector_multi.m before running 
% this script

for index1=1:size(trackerW,2)
    states1=trackerW(index1).states(1:3,:);
    velocity1=states1(1:3,2:end)-states1(1:3,1:end-1);
    timer1=trackerW(index1).start:trackerW(index1).end;   
    Bs=trackerW(index1).Bs;
    if size(Bs,1)<5 
        continue; 
    end;
    B_count=0;
    [ v1_norm,acc1_norm, acc1_on_v1_past, acc1_on_v1_past_norm, r1, v1_ang, acc1_ang] = calc_trace_attribute( states1 );
    
    figure(1);
    title(['fly fruit #' num2str(index1)]);
    
    subplot(2,5,1);
    hold off;
    plot(timer1(1:end-2),v1_norm,'r');
    hold on;
    xlabel('time(frame)');
    ylabel('velocity(mm/frame)');
    title('速度大小');
    grid on;
    
    subplot(2,5,6);
    
    hold off;
    plot(timer1(1:end-2),acc1_norm,'r');
    hold on;
    xlabel('time(frame)');
    ylabel('acceleration(mm/frame^2)');
    title('加速度大小');
    grid on;

    subplot(2,5,4);
    hold off;
    plot(0,0,'r');
    hold on;
    grid on;
    xlabel('time(frame)');
    ylabel('speed(mm/f)');
    title('速度投影模长');
    
    subplot(2,5,7);
    hold off;
    acc1_on_v1_past_norm_d=acc1_on_v1_past_norm(2:end)-acc1_on_v1_past_norm(1:end-1);
    plot(timer1(1:end-3),acc1_on_v1_past_norm_d,'r')
    hold on;
    xlabel('time(frame)');
    ylabel('d_acc(mm/frame^3)');
    title('法向加速度导数');
    grid on;
    
    subplot(2,5,8);
    hold off;
    plot(timer1(2:end-2),v1_ang,'r')
    hold on;
    grid on;
    xlabel('time(frame)');
    ylabel('angle(degree)');
    title('速度与上一帧速度的角度');
    
    subplot(2,5,5);
    hold off;
    plot(0,0,'k')
    hold on;
    xlabel('time(frame)');
    ylabel('angle(degree)');
    title('果蝇速度夹角');
    grid on;


    subplot(2,5,9);
    hold off;
    plot(0,0,'k')
    hold on;
    %plot(acc2_on_v2_1,'b');
    xlabel('time(frame)');
    ylabel('distance(mm)');
    title('果蝇间距离');
    grid on;
    
    subplot(2,5,3);
    hold off;
    plot(timer1(1:end-2),acc1_on_v1_past,'r')
    hold on;
    xlabel('time(frame)');
    ylabel('acc(mm/frame^2)');
    title('切向加速度');
    grid on;
    
    subplot(2,5,2);
    hold off;
    plot(timer1(1:end-2),acc1_on_v1_past_norm,'r')
    hold on;
    xlabel('time(frame)');
    ylabel('acc(mm/frame^2)');
    title('法向加速度');
    grid on;
      
    subplot(2,5,10);
    hold off;
    plot(0,0,'r');
    hold on;
    xlabel('time(frame)');
    ylabel('distance(mm)');
    title('果蝇与碰撞点的距离');
    grid on;

   
    for Bs_ind=1:size(Bs,1)
        %index2 mean the index of fly_b
        index2=Bs(Bs_ind).index_b;
        time_detect=Bs(Bs_ind).time_detect;
        time_crash=ceil(Bs(Bs_ind).time_end);
        time_strategy=Bs(Bs_ind).time_strategy;
        pos_crash_A=Bs(Bs_ind).pos_a;
        
        if time_strategy~=-1
            continue;
        end;
        
        if time_detect+3>time_crash
            continue;
        end;
        
        
        states2=trackerW(index2).states(1:3,:);
        velocity2=states2(1:3,2:end)-states2(1:3,1:end-1);
        timer2=trackerW(index2).start:trackerW(index2).end;   
        if timer2(end)<time_crash || timer1(end)<time_crash
            continue;
        end;
        B_count=B_count+1;
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
        
        subplot(2,5,1);
        plot(timer2(1:end-2),v2_norm,'b');       
        plotSig(v1_norm,v2_norm,time_detect,time_crash,timer1,timer2,2)
        
        subplot(2,5,6);
        plot(timer2(1:end-2),acc2_norm,'b');
        plotSig(acc1_norm,acc2_norm,time_detect,time_crash,timer1,timer2,2)
        
        subplot(2,5,4);
        plot(timer_both(1:end-1),norm_v2_on_v1,'b');
        plotSig([],norm_v2_on_v1,time_detect,time_crash,timer_both,timer_both,2)
        
        subplot(2,5,7);
        acc2_on_v2_past_norm_d=acc2_on_v2_past_norm(2:end)-acc2_on_v2_past_norm(1:end-1);
        plot(timer2(1:end-3),acc2_on_v2_past_norm_d,'b');
        plotSig(acc1_on_v1_past_norm_d,acc2_on_v2_past_norm_d,time_detect,time_crash,timer1,timer2,3)
        
        subplot(2,5,8);
        plot(timer2(2:end-2),v2_ang,'b');
        plotSig(v1_ang,v2_ang,time_detect,time_crash,timer1,timer2,3)
  
        subplot(2,5,5);
        plot(timer_both(1:end-1),angl_v1_v2,'k');
        plotSig([],angl_v1_v2,time_detect,time_crash,timer_both,timer_both,2)
        
        subplot(2,5,9);
        plot(timer_both,dist_s1_s2,'k');
        plotSig([],dist_s1_s2,time_detect,time_crash,timer_both,timer_both,2)
        
        subplot(2,5,3);
        plot(timer2(1:end-2),acc2_on_v2_past,'b');
        plotSig(acc1_on_v1_past,acc2_on_v2_past,time_detect,time_crash,timer1,timer2,2)
        
        subplot(2,5,2);
        plot(timer2(1:end-2),acc2_on_v2_past_norm,'b');
        plotSig(acc1_on_v1_past_norm,acc2_on_v2_past_norm,time_detect,time_crash,timer1,timer2,2)
        
        subplot(2,5,10);
        plot(timer2(1:end),dist_s2_crash_point,'b');
        plotSig([],dist_s2_crash_point,time_detect,time_crash,timer1,timer2,2)

    end;
    if B_count~=0
        saveas(gca,['../../statistics/' num2str(index1) '.png']);
        saveas(gca,['../../statistics/' num2str(index1) '.fig']);
    end;
end;

function ret=plotSig(vectorA,vectorB,time_detect,time_crash,timerA,timerB,delta)
    if ~isempty(vectorA)
        plot(time_detect,vectorA(find(timerA==time_detect)),'bp');
        if timerA(end)-delta>=time_crash
            plot(time_crash,vectorA(find(timerA==time_crash)),'rp');
        end;
    end;
    if timerB(end)-delta>=time_detect
        plot(time_detect,vectorB(find(timerB==time_detect)),'bp');
    end;
    if timerB(end)-delta>=time_crash
        plot(time_crash,vectorB(find(timerB==time_crash)),'rp');
    end
end