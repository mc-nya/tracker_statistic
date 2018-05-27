%-----wash the data----------------
k=[trackers(:).end]-[trackers(:).start];
trackerW=trackers(k>20);

%-----------sort the trace by time---------------
record_t=struct('id',{[]},'states',{[]});
record_crash=struct('id',{[]},'states',{[]});
for i=1:max([trackers(:).end])
    record_t(i).id=[];
    record_t(i).states=[];
    record_t(i).velocity=[];
end
for i = 1:size(trackerW,2)
    states=trackerW(i).states;
    velocity=states(1:3,2:end)-states(1:3,1:end-1);
    start=trackerW(i).start;
    for j=1:size(states,2)-1
        current=j+start-1;
        record_t(current).id=[record_t(current).id i];
        record_t(current).states=[record_t(current).states states(:,j)];
        record_t(current).velocity=[record_t(current).velocity velocity(:,j)];
    end
end

%-----------find crash pair---------------------
result_num=0;
for time=1:size(record_t,2)
    states=[record_t(time).states(:,:)];
    velocity=[record_t(time).velocity(:,:)];
   
    id=record_t(time).id;
    
    maxium_detection_distance=100;
    minium_crash_distance=5;
    for i=1:size(id,2)
        for j=1:size(id,2)
            if i==j
                continue;
            end
            %filter by maxium detection distance
            if (distance(states(1:3,i),states(1:3,j))<maxium_detection_distance)     
                delta_p=states(1:3,i)-states(1:3,j);
                delta_v=velocity(1:3,i)-velocity(1:3,j);                   
                delta_t=delta_p./-delta_v;
                min_dist=norm(cross(delta_p,delta_v))/norm(delta_v);
                min_dist_t=sqrt(norm(delta_p)^2-min_dist^2)/norm(delta_v);
                
                %filter by minium distance
                if min_dist<minium_crash_distance && min(delta_t)>0
                    result_num=result_num+1;
                    record_crash(result_num).time_start=time;
                    record_crash(result_num).time_end=time+min_dist_t;
                    record_crash(result_num).id=[id(i) id(j)];
                    record_crash(result_num).states=[states(:,i) states(:,j)];
                    record_crash(result_num).velocity=[velocity(:,i) velocity(:,j)];
                    record_crash(result_num).delta_time=min_dist_t;
                    record_crash(result_num).min_dist=min_dist;
                end
                
            end
        end
    end
end

clear time;
clear states velocity start k;
clear delta_p delta_v delta_t min_dist min_dist_t;
clear current i id j;