count=0;
for index=1:size(detect_set_washed,1)
    valid=show_trace(trackerW,detect_set_washed(index,:),15);
    if valid
        show_statistic(trackerW,detect_set_washed(index,:),15);
        count=count+1;
        saveas(2,['../../statistic/' num2str(count) '.png']);
        saveas(2,['../../statistic/' num2str(count) '.fig']);
        saveas(1,['../../trace/' num2str(count) '.png']);
        saveas(1,['../../trace/' num2str(count) '.fig']);
        valid=0;
    end

end