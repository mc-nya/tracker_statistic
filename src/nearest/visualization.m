for index=1:size(detect_set,1)
    valid=show_trace(trackerW,detect_set(index,:));
    
    if valid
        valid=0;
    end
end