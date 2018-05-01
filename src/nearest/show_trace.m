function valid= show_trace(trackerW, pair )
    time_nearest=pair(1);
    index1=pair(2);
    index2=pair(3);
    states1=trackerW(index1).states(1:3,:);
    states2=trackerW(index2).states(1:3,:);
    timer1=trackerW(index1).start:trackerW(index1).end;
    timer2=trackerW(index2).start:trackerW(index2).end;
    
    %-------------draw----------------
    figure(1);
    hold off;
    plot3(states1(1,:),states1(2,:),states1(3,:),'r');
    hold on;
    plot3(states2(1,:),states2(2,:),states2(3,:),'b');
    title('¹ûÓ¬·ÉĞĞ¹ì¼£');
    for i=1:10:size(states1,2)
        plot3(states1(1,i),states1(2,i),states1(3,i),'r+');
    end
    for i=1:10:size(states2,2)
        plot3(states2(1,i),states2(2,i),states2(3,i),'b+');
    end
    plot3(states1(1,1),states1(2,1),states1(3,1),'ro');
    plot3(states2(1,1),states2(2,1),states2(3,1),'bo');
    
    
    
    valid=1;
end

