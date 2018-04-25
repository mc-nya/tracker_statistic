function [ output_args ] = show_trace( trackerW,record_crash, k )
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
    if totaldist<5
        output_args=0;
        return;
    end
    figure(1);
    hold off;
    plot3(states1(1,:),states1(2,:),states1(3,:),'r');
    hold on;
    plot3(states2(1,:),states2(2,:),states2(3,:),'b');
    %legend('First','Second');
    title('¹ûÓ¬·ÉĞĞ¹ì¼£');
    for i=1:10:size(states1,2)
        plot3(states1(1,i),states1(2,i),states1(3,i),'r+');
    end
    for i=1:10:size(states2,2)
        plot3(states2(1,i),states2(2,i),states2(3,i),'b+');
    end
    plot3(states1(1,1),states1(2,1),states1(3,1),'ro');
    plot3(states2(1,1),states2(2,1),states2(3,1),'bo');
    
    %pring crash time
    if t_end-t_start+1<=size(states1,2)
        plot3(states1(1,t_end-t_start+1),states1(2,t_end-t_start+1),states1(3,t_end-t_start+1),'r*');
    end
    if t_end-t_start+1<=size(states2,2)
        plot3(states2(1,t_end-t_start+1),states2(2,t_end-t_start+1),states2(3,t_end-t_start+1),'g*');
    end
    %print carash position
    dur_time=record_crash(k).delta_time;
    v1=record_crash(k).velocity(1:3,1);
    v2=record_crash(k).velocity(1:3,2);
    p1=states1(1:3,1)+v1*dur_time;
    p2=states2(1:3,1)+v2*dur_time;

    plot3(p1(1),p1(2),p1(3),'rh');
    plot3(p2(1),p2(2),p2(3),'bh');
    
    grid on;
    output_args=1;

end

