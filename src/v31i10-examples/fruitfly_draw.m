speedup=[];
speeddown=[];
temp=[];
for frame=-10:10
    open(['H:\\果蝇数据分析\\上升_角度分布\\statistic\\angle_' num2str(frame) '.fig']);
    speedup=get(gca,'children');
    speedup=[get(speedup(1),'xdata'); get(speedup(1),'ydata')];
    %speedup(2,:)=speedup(2,:)/sum(speedup(2,:));
    temp=[];
    for i=1:size(speedup,2)
        for j=1:speedup(2,i)
            temp=[temp i*10-10];
        end
    end
    speedup=circ_ang2rad(temp);
    open(['H:\\果蝇数据分析\\下降_角度分布\\statistic\\angle_' num2str(frame) '.fig']);
    speeddown=get(gca,'children');
    speeddown=[get(speeddown(1),'xdata'); get(speeddown(1),'ydata')];
    %speeddown(2,:)=speeddown(2,:)/sum(speeddown(2,:));
    temp=[];
    for i=1:size(speeddown,2)
        for j=1:speeddown(2,i)
            temp=[temp i*10-10];
        end
    end
    speeddown=circ_ang2rad(temp);
    
    figure;
    %hold on;
    circ_plot(speedup,'hist',[],40,true,true,'k','linewidth',2,'color','b');
    hold on;
    circ_plot(speeddown,'hist',[],40,true,true,'b','linewidth',2,'color','r');
    title(['frame: ' num2str(frame) ' (blue:speedup,red:speeddown)']);
    saveas(gca,['../../statistic/angle_' num2str(frame) '.png']);
    saveas(gca,['../../statistic/angle_' num2str(frame) '.fig']);
end