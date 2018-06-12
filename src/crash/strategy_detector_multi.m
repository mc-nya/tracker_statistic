left_delta_time=7;
right_delta_time=4;

threshold_acc_min=1000;
threshold_acc_d_min=1000;
threshold_acc_norm_min=0.77;
threshold_acc_norm_d_min=0.77;
threshold_acc_tan_min=1000;
threshold_acc_tan_d_min=1000;

threshold_acc_max=1000;
threshold_acc_d_max=1000;
threshold_acc_norm_max=2.5;
threshold_acc_norm_d_max=2.5;
threshold_acc_tan_max=1000;
threshold_acc_tan_d_max=1000;

threshold_avg_acc=inf;
threshold_avg_acc_norm=-inf;
threshold_avg_acc_tan=-inf;
minium_crash_distance=7;


for index1=1:size(trackerW,2)
    Bs=trackerW(index1).Bs;
    for Bs_ind=1:size(Bs,1);
        %index2 mean the index of fly_b
        index2=Bs(Bs_ind).index_b;
        time_detect=Bs(Bs_ind).time_detect;
        time_crash=ceil(Bs(Bs_ind).time_end);
        
        states1=trackerW(index1).states(1:3,:);
        states2=trackerW(index2).states(1:3,:);
        timer1=trackerW(index1).start:trackerW(index1).end;   
        timer2=trackerW(index2).start:trackerW(index2).end;   
        
        
        if right_delta_time~=0
            start_time1=max(timer1(1),time_detect);
            end_time1=min(timer1(end),time_crash+right_delta_time);
            states1=states1(1:3,find(timer1==start_time1):find(timer1==end_time1));
            timer1=timer1(find(timer1==start_time1):find(timer1==end_time1));   

            start_time2=max(timer2(1),time_detect);
            end_time2=min(timer2(end),time_crash+right_delta_time);
            states2=states2(1:3,find(timer2==start_time2):find(timer2==end_time2));
            timer2=timer2(find(timer2==start_time2):find(timer2==end_time2));  
        end
    
            
        velocity1=states1(1:3,2:end)-states1(1:3,1:end-1);
        
        
        [ v1_norm,acc1_norm, acc1_on_v1_past, acc1_on_v1_past_norm, r1, v1_ang, acc1_ang] = calc_trace_attribute( states1 );
        
        acc1=velocity1(1:3,2:end)-velocity1(1:3,1:end-1);
        acc1_d=acc1(1:3,2:end)-acc1(1:3,1:end-1);
        acc1_tan_d=acc1_on_v1_past(2:end)-acc1_on_v1_past(1:end-1);
        acc1_norm_d=acc1_on_v1_past_norm(2:end)-acc1_on_v1_past_norm(1:end-1);
        acc1_tan=acc1_on_v1_past;
        acc1_norm=acc1_on_v1_past_norm;
    
        
        index_post_strategy=-1;
        for i=1:size(acc1_tan_d,2)
            if(norm(acc1_tan(:,i))>threshold_acc_tan_min && norm(acc1_tan(:,i))<threshold_acc_tan_max) ...,
                    || (norm(acc1_tan_d(:,i))>threshold_acc_tan_d_min && norm(acc1_tan_d(:,i))<threshold_acc_tan_d_max)
                index_post_strategy=i;
                break;
            end
            if(norm(acc1(:,i))>threshold_acc_min && norm(acc1(:,i))<threshold_acc_max)  ...,
                    || (norm(acc1_d(:,i))>threshold_acc_d_min && norm(acc1_d(:,i))<threshold_acc_d_max)
                index_post_strategy=i;
                break;
            end
            if(norm(acc1_norm(:,i))>threshold_acc_norm_min && norm(acc1_norm(:,i))<threshold_acc_norm_max) ...,
                    || (norm(acc1_norm_d(:,i))>threshold_acc_norm_d_min && norm(acc1_norm_d(:,i))<threshold_acc_norm_d_max)
                index_post_strategy=i;
                break;
            end
        end 

        index_strategy=-1;
        for i=index_post_strategy:-1:1
            if norm(acc1_tan(:,i))<threshold_avg_acc_tan
                index_strategy=i;
                break;
            end
            if norm(acc1(:,i))<threshold_avg_acc
                index_strategy=i;
                break;
            end
            if norm(acc1_norm(:,i))<threshold_avg_acc_norm
                index_strategy=i;
                break;
            end
        end
        clear index_post_strategy;

        if index_strategy~=-1
            trackerW(index1).Bs(Bs_ind).time_strategy=timer1(index_strategy);
            trackerW(index1).Bs(Bs_ind).pos_strategy=states1(:,index_strategy);
            %record_crash_washed(1,outer_index).time_strategy=timer1(index_strategy);
        else
            trackerW(index1).Bs(Bs_ind).time_strategy=-1;
            trackerW(index1).Bs(Bs_ind).pos_strategy=[NaN NaN NaN];
        end
        clear index_strategy;
    end
        
end













clear temp_record_crash_washed;
clear time_detect velocity1 velocity2 start_time1 start_time2 ans acc1 acc2 acc1_d acc2_d;
clear outer_index pair time_crash index1 index2 states1 states2 timer1 timer2 end_time1 end_time2;
clear i;
clear v1_norm acc1_norm  acc1_on_v1_past  acc1_on_v1_past_norm  r1  v1_ang  acc1_ang;
clear v2_norm acc2_norm  acc2_on_v2_past  acc2_on_v2_past_norm  r2  v2_ang  acc2_ang;
clear time_pad acc1_norm_d acc1_norm acc1_tan acc1_tan_d;
clear v1 v2 s1 s2 timer_both dur_time p1_crash dist_s1_s2 angl_v1_v2 norm_v1_on_v2 norm_v2_on_v1 dist_s1_crash_point;

clear t delta_p delta_t delta_v dist_s2_crash_point feature frame h1 h2 id3 ids min_dist min_dist_t p2_crash;
clear states states1_c states3 velocity3 velocitys bin_str_crash_distribution;


        