function [ output_args ] = show_trace( trackerW,pair, left_delta_time,right_delta_time )
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
    
    figure(1);
    hold off;
    plot3(states1(1,:),states1(2,:),states1(3,:),'r');
    hold on;
    plot3(states2(1,:),states2(2,:),states2(3,:),'b');
    %legend('First','Second');
    title('¹ûÓ¬·ÉĞĞ¹ì¼£');
    for i=1:size(timer1,2)
        if mod(timer1(i),10)==0
            plot3(states1(1,i),states1(2,i),states1(3,i),'r+');
        end      
    end
    
    for i=1:size(timer2,2)
        if mod(timer2(i),10)==0
            plot3(states2(1,i),states2(2,i),states2(3,i),'b+');
        end
    end
    plot3(states1(1,find(timer1==time_detect)),states1(2,find(timer1==time_detect)),states1(3,find(timer1==time_detect)),'ro');
    plot3(states2(1,find(timer2==time_detect)),states2(2,find(timer2==time_detect)),states2(3,find(timer2==time_detect)),'bo');
    
    %pring crash time
    if time_crash<=timer1(end)
        plot3(states1(1,find(timer1==time_crash)),states1(2,find(timer1==time_crash)),states1(3,find(timer1==time_crash)),'r*');
    end
    if time_crash<=timer2(end)
        plot3(states2(1,find(timer2==time_crash)),states2(2,find(timer2==time_crash)),states2(3,find(timer2==time_crash)),'b*');
    end
    %print crash position
    dur_time=pair.delta_time;
    v1=pair.velocity(1:3,1);
    v2=pair.velocity(1:3,2);
    p1=states1(1:3,find(timer1==time_detect))+v1*dur_time;
    p2=states2(1:3,find(timer2==time_detect))+v2*dur_time;

    plot3(p1(1),p1(2),p1(3),'rh');
    plot3(p2(1),p2(2),p2(3),'bh');
    
    grid on;
    output_args=1;

end

