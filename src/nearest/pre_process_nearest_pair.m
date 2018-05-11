%-----wash the data----------------
k=[trackers(:).end]-[trackers(:).start];
trackerW=trackers(k>10);

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

%------------------detect-------------------
%parameters
max_distance_two_flies=7;
min_distance_other_flies=20;
%start
detect_set=[];
for time=1:size(record_t,2)-1
    current_ids=record_t(time).id;
    current_states=record_t(time).states(1:3,:);
    for current_fly=1:size(current_ids,2)
        dist_vec=current_states-repmat(current_states(:,current_fly),[1 size(current_states,2)]);
        dist=sum(dist_vec.^2,1);
        [result, index]=sort(dist,'ascend');
        if size(find(result<2),2)==1 && size(find(result<max_distance_two_flies),2)==2 && size(find(result<min_distance_other_flies),2)==2
            detect_set=[detect_set;time, current_ids(current_fly), current_ids(index(2))];
            %current_ids(current_fly) 
            %current_ids(index(2))
        end
    end
end    
%make pair unique
% for index=1:size(detect_set,1)
%     if(detect_set(index,2)>detect_set(index,3))
%         k=detect_set(index,3);
%         detect_set(index,3)=detect_set(index,2);
%         detect_set(index,2)=k;
%     end
% end
% detect_set=unique(detect_set,'rows');
