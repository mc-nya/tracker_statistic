count=0;
global feature_angle_distribution;
%feature_angle_distribution=zeros(delta_time*3+6,20);

for index=1:size(detect_set_washed,1)
    valid=show_trace(trackerW,detect_set_washed(index,:),12);
    if valid
        show_statistic(trackerW,detect_set_washed(index,:),12);
        count=count+1;
        %saveas(2,['../../statistic/' num2str(count) '.png']);
        %saveas(2,['../../statistic/' num2str(count) '.fig']);
        %saveas(1,['../../trace/' num2str(count) '.png']);
        %saveas(1,['../../trace/' num2str(count) '.fig']);
        valid=0;
    end

end

for index=delta_time-4:size(feature_angle_distribution,1)
    figure(3);
    hold off;
    bar(feature_angle_distribution(index,:));
    title(['frame:' num2str(index-delta_time*2) 'interval: 10degree']);
    %saveas(3,['../../statistic/angle_' num2str(index-delta_time*2) '.png']);
    %saveas(3,['../../statistic/angle_' num2str(index-delta_time*2) '.fig']);
end