save_crash=[];
count=1;
left_delta_time=12;
right_delta_time=5;
global feature_angle_distribution;
feature_angle_distribution=zeros(left_delta_time*3+right_delta_time,20);
for k = 1:size(record_crash_washed,2)
    
    show_trace(trackerW,record_crash_washed(1,k),left_delta_time,right_delta_time);
    t=show_statistic(trackerW,record_crash_washed(1,k),left_delta_time,right_delta_time);
    %show_trace_from_zero( trackerW,record_crash, k );
    %show_statistic_from_zero(trackerW,record_crash,k);
    if t==1
        saveas(2,['../statistic/' num2str(count) '.png']);
        saveas(2,['../statistic/' num2str(count) '.fig']);
        saveas(1,['../trace/' num2str(count) '.png']);
        saveas(1,['../trace/' num2str(count) '.fig']);
%         saveas(4,['../statistic_from_start/' num2str(count) '.png']);
%         saveas(4,['../statistic_from_start/' num2str(count) '.fig']);
%         saveas(3,['../trace_from_start/' num2str(count) '.png']);
%         saveas(3,['../trace_from_start/' num2str(count) '.fig']);

        t=0;
        %k
        count=count+1;
    end
    t=1;
end

for index=left_delta_time-4:size(feature_angle_distribution,1)
    figure(3);
    hold off;
    bar(feature_angle_distribution(index,:));
    title(['frame:' num2str(index-left_delta_time*2) 'interval: 10degree']);
    %saveas(3,['../../statistic/angle_' num2str(index-delta_time*2) '.png']);
    %saveas(3,['../../statistic/angle_' num2str(index-delta_time*2) '.fig']);
end
