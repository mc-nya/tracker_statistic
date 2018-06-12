for record_index=1:size(record_crash_washed,2)
    %   initialize specific collision pair 
    pair=record_crash_washed(1,record_index);
    time_collision=floor(pair.time_end);
    time_detect=pair.time_start;
    time_strategy=pair.time_strategy;
    index1=pair.id(1);
    index2=pair.id(2);
    states1=trackerW(index1).states(1:3,:);
    states2=trackerW(index2).states(1:3,:);
    %   each element in timer correspond to element in states, record the state's time 
    timer1=trackerW(index1).start:trackerW(index1).end;   
    timer2=trackerW(index2).start:trackerW(index2).end;
    %   time_min,max record A and B's time in both
    time_min=min(timer1(1),timer2(1));
    time_max=max(timer1(end),timer2(end));
    %   third: record fly C in the trace of A (at time t~t+n)
    third=record_crash_washed(1,record_index).fly_c;
    
    
    for i=1:size(third,1)
        index3=third(i,2);
        states3=trackerW(index3).states(1:3,:);
        timer3=trackerW(index3).start:trackerW(index3).end;   
        time_max=max(time_max,timer3(end));
    end
    
    figure;
    

    hold off;
    patch('XData',[states1(1,:) NaN],'YData',[states1(2,:) NaN],'ZData',[states1(3,:) NaN], ...,
               'facecolor','none','edgecolor','r'); 
    hold on;
    
    plot3(states1(1,1),states1(2,1),states1(3,1),'hr');
    
    patch('XData',[states2(1,:) NaN],'YData',[states2(2,:) NaN], 'ZData',[states2(3,:) NaN], ...,
        'CData',[color(find(timer_color==timer2(1)):find(timer_color==timer2(end))) NaN], ...,
               'facecolor','none','edgecolor','interp'); 
           
    plot3(states2(1,1),states2(2,1),states2(3,1),'hb');
   
    plot3(states2(1,find(timer3==time_detect)),states2(2,find(timer3==time_detect)),states2(3,find(timer3==time_detect)), ...,
            '+r');
        
    plot3(states1(1,find(timer1==time_detect)),states1(2,find(timer1==time_detect)),states1(3,find(timer1==time_detect)), ...,
            '+r');
    plot3(states1(1,find(timer1==time_strategy)),states1(2,find(timer1==time_strategy)),states1(3,find(timer1==time_strategy)), ...,
            'or');  
    plot3([states1(1,find(timer1==time_detect)) states2(1,find(timer3==time_detect))], ...,
            [states1(2,find(timer1==time_detect)) states2(2,find(timer3==time_detect))], ...,
            [states1(3,find(timer1==time_detect)) states2(3,find(timer3==time_detect))], ...,
            '--','color',[0.8,0.8,0.8])
    
    for i=1:size(third,1)
        
        i
        index3=third(i,2);
        time_detect=third(i,1);
        states3=trackerW(index3).states(1:3,:);
        timer3=trackerW(index3).start:trackerW(index3).end;   
        states3=states3(1:3,find(timer3==time_min):end);
        timer3=timer3(find(timer3==time_min):end);
        if size(states3,2)==0
            continue;
        end
        patch('XData',[states3(1,:) NaN],'YData',[states3(2,:) NaN], ...
               'ZData',[states3(3,:) NaN],'CData',[color(find(timer_color==timer3(1)):find(timer_color==timer3(end))) NaN], ... 
               'facecolor','none','edgecolor','interp'); 
        plot3(states3(1,1),states3(2,1),states3(3,1),'hk');
        plot3(states3(1,find(timer3==time_detect)),states3(2,find(timer3==time_detect)),states3(3,find(timer3==time_detect)), ...,
            '+','color',[1-color(find(timer_color==time_detect)) color(find(timer_color==time_detect)) 1-color(find(timer_color==time_detect))]);
        
        plot3(states1(1,find(timer1==time_detect)),states1(2,find(timer1==time_detect)),states1(3,find(timer1==time_detect)), ...,
            '+','color',[1-color(find(timer_color==time_detect)) color(find(timer_color==time_detect)) 1-color(find(timer_color==time_detect))]);
        
        plot3([states1(1,find(timer1==time_detect)) states3(1,find(timer3==time_detect))], ...,
            [states1(2,find(timer1==time_detect)) states3(2,find(timer3==time_detect))], ...,
            [states1(3,find(timer1==time_detect)) states3(3,find(timer3==time_detect))], ...,
            '--','color',[0.95,0.95,0.95])
    end
    t=0;
end