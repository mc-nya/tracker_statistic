
for frame=1:size(feature_angle_distribution,2)
    feature=feature_angle_distribution(:,frame);
    feature=feature(feature~=0);
    feature=circ_ang2rad(feature);

    figure;
    %hold on;
    circ_plot(feature,'hist',[],40,true,true,'k','linewidth',2,'color','b');
    %hold on;
    title(['frame: ' num2str(frame) ' strategy frame: ' num2str(time_left_longest+1)]);
    saveas(gca,['../../statistic/angle_' num2str(frame) '.png']);
    saveas(gca,['../../statistic/angle_' num2str(frame) '.fig']);
    t=0;
end