for index1=1:size(trackerW,2)
    states1=trackerW(index1).states(1:3,:);
    timer1=trackerW(index1).start:trackerW(index1).end;   
    Bs=trackerW(index1).Bs;
    if size(Bs,1)<5 
        continue; 
    end;
    
    
    figure(1);

    hold off;
    plot3(states1(1,1),states1(2,1),states1(3,1),'hr');
    hold on;
    patch('XData',[states1(1,:) NaN],'YData',[states1(2,:) NaN],'ZData',[states1(3,:) NaN], ...,
               'facecolor','none','edgecolor','r'); 
   
    title('法向加速度小于0.525 切向加速度小于0.274');

    
    for Bs_ind=1:size(Bs,1)
        %index2 mean the index of fly_b
        index2=Bs(Bs_ind).index_b;
        time_detect=Bs(Bs_ind).time_detect;
        time_crash=ceil(Bs(Bs_ind).time_end);
        time_strategy=Bs(Bs_ind).time_strategy;
        pos_crash_A=Bs(Bs_ind).pos_a;
        
        if time_strategy~=-1
            continue
        end;
        states2=trackerW(index2).states(1:3,:);
        timer2=trackerW(index2).start:trackerW(index2).end;   
        
        
        %patch('XData',[states2(1,find(timer2==time_detect):end) NaN],'YData',[states2(2,find(timer2==time_detect):end) NaN], 'ZData',[states2(3,find(timer2==time_detect):end) NaN], ...,
        %       'facecolor','none','edgecolor','b');
        %plot3(states2(1,find(timer2==time_detect)),states2(2,find(timer2==time_detect)),states2(3,find(timer2==time_detect)),'hb');
        
        patch('XData',[states2(1,:) NaN],'YData',[states2(2,:) NaN], 'ZData',[states2(3,:) NaN], ...,
               'facecolor','none','edgecolor','b'); 
           
        plot3(states2(1,1),states2(2,1),states2(3,1),'hb');
        
        plot3(states1(1,find(timer1==time_detect)),states1(2,find(timer1==time_detect)),states1(3,find(timer1==time_detect)), ...,
            '+r');
        plot3(states2(1,find(timer2==time_detect)),states2(2,find(timer2==time_detect)),states2(3,find(timer2==time_detect)), ...,
            '+r');
%         plot3([states1(1,find(timer1==time_detect)) states2(1,find(timer2==time_detect))], ...,
%             [states1(2,find(timer1==time_detect)) states2(2,find(timer2==time_detect))], ...,
%             [states1(3,find(timer1==time_detect)) states2(3,find(timer2==time_detect))], ...,
%             '--','color',[0.8,0.8,0.8])
        plot3([states1(1,find(timer1==time_detect)) pos_crash_A(1)], ...,
            [states1(2,find(timer1==time_detect)) pos_crash_A(2)], ...,
            [states1(3,find(timer1==time_detect)) pos_crash_A(3)], ...,
            '--','color',[0.8,0.8,0.8])
        plot3([states2(1,find(timer2==time_detect)) pos_crash_A(1)], ...,
            [states2(2,find(timer2==time_detect)) pos_crash_A(2)], ...,
            [states2(3,find(timer2==time_detect)) pos_crash_A(3)], ...,
            '--','color',[0.8,0.8,0.8])
        plot3(pos_crash_A(1),pos_crash_A(2),pos_crash_A(3),'dr');
        
    end;
    saveas(gca,['../../trace/' num2str(index1) '.png']);
    saveas(gca,['../../trace/' num2str(index1) '.fig']);
end;
