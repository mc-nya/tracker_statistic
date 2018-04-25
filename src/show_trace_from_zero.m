function [ output_args ] = show_trace_from_zero( trackerW,record_crash, k )
    id=record_crash(k).id;
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
    
    figure(3);
    hold off;
    plot3(states1(1,:),states1(2,:),states1(3,:),'r');
    hold on;
    plot3(states2(1,:),states2(2,:),states2(3,:),'b');
    %legend('First','Second');
    title('¹ûÓ¬·ÉÐÐ¹ì¼£');
    for i=1:10:size(states1,2)
        plot3(states1(1,i),states1(2,i),states1(3,i),'r+');
    end
    for i=1:10:size(states2,2)
        plot3(states2(1,i),states2(2,i),states2(3,i),'b+');
    end
    %trace start
    plot3(states1(1,1),states1(2,1),states1(3,1),'rs');  
    plot3(states2(1,1),states2(2,1),states2(3,1),'bs');
   
    %detect point
    plot3(states1(1,index_detect1),states1(2,index_detect1),states1(3,index_detect1),'ro');
    plot3(states2(1,index_detect2),states2(2,index_detect2),states2(3,index_detect2),'bo');
    
    %pring crash time
    if time_crash-time_trace_start1+1<=size(states1,2)
        plot3(states1(1,time_crash-time_trace_start1+1),states1(2,time_crash-time_trace_start1+1),states1(3,time_crash-time_trace_start1+1),'r*');
    end
    if time_crash-time_trace_start2+1<=size(states2,2)
        plot3(states2(1,time_crash-time_trace_start2+1),states2(2,time_crash-time_trace_start2+1),states2(3,time_crash-time_trace_start2+1),'b*');
    end
    
    %print carash position
        
    dur_time=record_crash(k).delta_time;
    v1_detect=record_crash(k).velocity(1:3,1);
    v2_detect=record_crash(k).velocity(1:3,2);
    p1=states1(1:3,index_detect1)+v1_detect*dur_time;
    p2=states2(1:3,index_detect2)+v2_detect*dur_time;

    plot3(p1(1),p1(2),p1(3),'rh');
    plot3(p2(1),p2(2),p2(3),'bh');
    
    grid on;
    output_args=1;

end

