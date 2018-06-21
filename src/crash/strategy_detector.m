left_delta_time=7;
right_delta_time=4;

threshold_acc_min=1000;
threshold_acc_d_min=1000;
threshold_acc_norm_min=1.51;
threshold_acc_norm_d_min=0.47;
threshold_acc_tan_min=1000;
threshold_acc_tan_d_min=1000;

threshold_acc_max=1000;
threshold_acc_d_max=1000;
threshold_acc_norm_max=3.0;
threshold_acc_norm_d_max=1.75;
threshold_acc_tan_max=1000;
threshold_acc_tan_d_max=1000;

threshold_avg_acc=-inf;
threshold_avg_acc_norm=0.22;
threshold_avg_acc_tan=-inf;
minium_crash_distance=7;

record_crash_washed=filter_strategy_detector(trackerW,record_crash,left_delta_time,right_delta_time);

feature_dt_detect=[];
%==============step 1 根据加速度和加速度梯度对齐==============
for outer_index=1:size(record_crash_washed,2)
    pair=record_crash_washed(1,outer_index);
    time_crash=floor(pair.time_end);
    time_detect=pair.time_start;
    index1=pair.id(1);
    index2=pair.id(2);
    states1=trackerW(index1).states(1:3,:);
    states2=trackerW(index2).states(1:3,:);
    timer1=trackerW(index1).start:trackerW(index1).end;   
    timer2=trackerW(index2).start:trackerW(index2).end;   
        %cut time(time_start---crash---right_delta_time)
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
    acc1=velocity1(1:3,2:end)-velocity1(1:3,1:end-1);
    acc1_d=acc1(1:3,2:end)-acc1(1:3,1:end-1);
    
    velocity2=states2(1:3,2:end)-states2(1:3,1:end-1);
    acc2=velocity2(1:3,2:end)-velocity2(1:3,1:end-1);
    acc2_d=acc2(1:3,2:end)-acc2(1:3,1:end-1);
    
    [ v1_norm,acc1_norm, acc1_on_v1_past, acc1_on_v1_past_norm, r1, v1_ang, acc1_ang] = calc_trace_attribute( states1 );
    
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
        if i<index_post_strategy && norm(acc1_norm(:,i))>norm(acc1_norm(:,i))
            index_strategy=i;
            break;
        end;
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
    
    
    
    
    if index_strategy~=-1
        record_crash_washed(1,outer_index).time_strategy=timer1(index_strategy);
        feature_dt_detect=[feature_dt_detect index_post_strategy-index_strategy];
    end
    clear index_post_strategy;
    clear index_strategy;
    %[ v2_norm,acc2_norm, acc2_on_v2_past, acc2_on_v2_past_norm, r2, v2_ang, acc2_ang] = calc_trace_attribute( states2 );
    
end
feature_dt_detect=feature_dt_detect(feature_dt_detect~=0);
histogram(feature_dt_detect);

median(feature_dt_detect)
% ====================step2  筛选出找到策略的项，并保证与起始点不重合==============
temp_record_crash_washed=[];
for outer_index=1:size(record_crash_washed,2)
    if isempty(record_crash_washed(1,outer_index).time_strategy)~=1
        if record_crash_washed(1,outer_index).time_strategy~=record_crash_washed(1,outer_index).time_start
            if record_crash_washed(1,outer_index).time_strategy<record_crash_washed(1,outer_index).time_end
                temp_record_crash_washed=[temp_record_crash_washed record_crash_washed(1,outer_index)];
            end
        end
    end
end
record_crash_washed=temp_record_crash_washed;



%==============step3  对齐轨迹并可视化==============
time_left_longest=0;
time_right_longest=0;

%计算对齐相关时间参数
for outer_index=1:size(record_crash_washed,2)
    pair=record_crash_washed(1,outer_index);
    time_crash=floor(pair.time_end);
    time_detect=pair.time_start;
    time_strategy=pair.time_strategy;
    time_left_longest=max(time_strategy-time_detect,time_left_longest);
    time_right_longest=max(time_crash-time_strategy+right_delta_time,time_right_longest);
end

feature_v1_norm=[];
feature_v2_norm=[];
feature_acc1_norm=[];
feature_acc1_tan=[];
feature_acc2_norm=[];
feature_acc2_tan=[];
feature_angle_distribution=[];
feature_dist_s1_crash_point=[];
feature_dist_s2_crash_point=[];
feature_str_crash_distribution=[];
feature_dist_s1_s2=[];


feature_time_dist=[];
for outer_index=1:size(record_crash_washed,2)
    pair=record_crash_washed(1,outer_index);
    time_crash=floor(pair.time_end);
    time_detect=pair.time_start;
    time_strategy=pair.time_strategy;
    
    feature_time_dist=[feature_time_dist; [time_strategy-time_detect time_crash-time_strategy]];
    
    index1=pair.id(1);
    index2=pair.id(2);
    states1=trackerW(index1).states(1:3,:);
    states2=trackerW(index2).states(1:3,:);
    timer1=trackerW(index1).start:trackerW(index1).end;   
    timer2=trackerW(index2).start:trackerW(index2).end;
    
    
    %----------------cut time(time_start---crash---right_delta_time)------
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
    
    time_pad=time_strategy-time_left_longest-1;
    %----------在A的轨迹上寻找可能碰撞的C---------------
    outer_index
    record_crash_washed(outer_index).fly_c=[];
    for time=timer1(1):timer1(end)
        ids=record_t(time).id;
        for i=1:size(ids,2)
            id3=ids(1,i);
            if id3==index1 || id3==index2 
                continue;
            end
            states=[record_t(time).states(:,:)];
            velocitys=[record_t(time).velocity(:,:)];
            states1_c=states(1:3,find(ids==index1));
            states3=states(1:3,i);
            velocity1=velocitys(1:3,find(ids==index1));
            velocity3=velocitys(1:3,i);
            if (distance(states1_c,states3)<maxium_detection_distance)
                delta_p=states1_c-states3;
                delta_v=velocity1-velocity3;
                delta_t=delta_p./-delta_v;
                min_dist=norm(cross(delta_p,delta_v))/norm(delta_v);
                min_dist_t=sqrt(norm(delta_p)^2-min_dist^2)/norm(delta_v);
                if min_dist<minium_crash_distance && min(delta_t)>0
                    temp=[time id3];
                    feature_str_crash_distribution=[feature_str_crash_distribution time-time_pad];
                    record_crash_washed(outer_index).fly_c=[record_crash_washed(outer_index).fly_c; temp];
                end
            end
        end
        
    end
    clear time;
    
    %----------------修改并对齐时间：time_left_longest~time_strategy~time_right_longest------------------
    time_pad=time_strategy-time_left_longest-1;
    timer1=timer1-time_pad;
    timer2=timer2-time_pad;
    time_crash=time_crash-time_pad;
    time_detect=time_detect-time_pad;
    time_strategy=time_strategy-time_pad;
   
    %---------------计算轨迹属性---------------------
    [ v1_norm,acc1_norm, acc1_on_v1_past, acc1_on_v1_past_norm, r1, v1_ang, acc1_ang] = calc_trace_attribute( states1 );
    
    [ v2_norm,acc2_norm, acc2_on_v2_past, acc2_on_v2_past_norm, r2, v2_ang, acc2_ang] = calc_trace_attribute( states2 );
    
    
    velocity1=states1(1:3,2:end)-states1(1:3,1:end-1);
    acc1=velocity1(1:3,2:end)-velocity1(1:3,1:end-1);
    acc1_d=acc1(1:3,2:end)-acc1(1:3,1:end-1);
    
    velocity2=states2(1:3,2:end)-states2(1:3,1:end-1);
    acc2=velocity2(1:3,2:end)-velocity2(1:3,1:end-1);
    acc2_d=acc2(1:3,2:end)-acc2(1:3,1:end-1);
    
    
    dur_time=pair.delta_time;
    v1=pair.velocity(1:3,1);
    v2=pair.velocity(1:3,2);
    p1_crash=states1(1:3,find(timer1==time_detect))+v1*dur_time;
    p2_crash=states2(1:3,find(timer2==time_detect))+v2*dur_time;
    dist_s1_crash_point=[];
    dist_s2_crash_point=[];
    for i=1:size(timer1,2);
        %==========简记变量初始化=========
        s1=states1(1:3,i);  
        %======计算和虚拟碰撞点的距离===========
        dist_s1_crash_point=[dist_s1_crash_point norm(s1-p1_crash)];     
    end
    for i=1:size(timer2,2);
        %==========简记变量初始化=========
        s2=states2(1:3,i);  
        %======计算和虚拟碰撞点的距离===========
        dist_s2_crash_point=[dist_s2_crash_point norm(s2-p2_crash)];     
    end
   
    timer_both=max(timer1(1),timer2(1)):min(timer1(end)-2,timer2(end)-2);
    dist_s1_s2=[];
    angl_v1_v2=[];
    norm_v1_on_v2=[];
    norm_v2_on_v1=[];
    for i=1:size(timer_both,2)
        s1=states1(1:3,find(timer1==timer_both(i)));
        s2=states2(1:3,find(timer2==timer_both(i)));
        dist_s1_s2=[dist_s1_s2 norm(s1-s2)];
        
        if i~=size(timer_both,2)
            v1=velocity1(1:3,find(timer1==timer_both(i)));
            v2=velocity2(1:3,find(timer2==timer_both(i)));
            angl_v1_v2=[angl_v1_v2 acosd(dot(v1,v2)/norm(v1)/norm(v2))];
            norm_v1_on_v2=[norm_v1_on_v2 dot(v1,v2)/norm(v2)];
            norm_v2_on_v1=[norm_v2_on_v1 dot(v1,v2)/norm(v1)];
        end
    end
    
    %-----记录速度大小-----
    temp=zeros(1,time_left_longest+1+time_right_longest); 
    for i=1:size(v1_norm,2)
        temp(1,timer1(i))=v1_norm(i);
    end
    feature_v1_norm=[feature_v1_norm; temp];
    
    temp=zeros(1,time_left_longest+1+time_right_longest); 
    for i=1:size(v2_norm,2)
        temp(1,timer2(i))=v2_norm(i);
    end
    feature_v2_norm=[feature_v2_norm; temp];
    
    %-----记录法向加速度大小------
    temp=zeros(1,time_left_longest+1+time_right_longest); 
    for i=1:size(acc1_on_v1_past_norm,2)
        temp(1,timer1(i))=acc1_on_v1_past_norm(i);
    end
    feature_acc1_norm=[feature_acc1_norm; temp];
    
    temp=zeros(1,time_left_longest+1+time_right_longest); 
    for i=1:size(acc2_on_v2_past_norm,2)
        temp(1,timer2(i))=acc2_on_v2_past_norm(i);
    end
    feature_acc2_norm=[feature_acc2_norm; temp];
    
    %-----记录切向加速度大小------
    temp=zeros(1,time_left_longest+1+time_right_longest); 
    for i=1:size(acc1_on_v1_past,2)
        temp(1,timer1(i))=acc1_on_v1_past(i);
    end
    feature_acc1_tan=[feature_acc1_tan; temp];
    
    temp=zeros(1,time_left_longest+1+time_right_longest); 
    for i=1:size(acc2_on_v2_past,2)
        temp(1,timer2(i))=acc2_on_v2_past(i);
    end
    feature_acc2_tan=[feature_acc2_tan; temp];
    
    
    %-----记录相互距离------
    temp=zeros(1,time_left_longest+1+time_right_longest); 
    for i=1:size(dist_s1_s2,2)
        temp(1,timer_both(i))=dist_s1_s2(i);
    end
    feature_dist_s1_s2=[feature_dist_s1_s2; temp];
        
    %------记录AB分别和碰撞点距离-----
    temp=zeros(1,time_left_longest+1+time_right_longest); 
    for i=1:size(dist_s1_crash_point,2)
        temp(1,timer1(i))=dist_s1_crash_point(i);
    end
    feature_dist_s1_crash_point=[feature_dist_s1_crash_point; temp];
    
    temp=zeros(1,time_left_longest+1+time_right_longest); 
    for i=1:size(dist_s2_crash_point,2)
        temp(1,timer2(i))=dist_s2_crash_point(i);
    end
    feature_dist_s2_crash_point=[feature_dist_s2_crash_point; temp];
    
    %-----记录互相角度变化-----
    temp=zeros(1,time_left_longest+1+time_right_longest); 
    for i=1:size(angl_v1_v2,2)
        temp(1,timer_both(i))=angl_v1_v2(i);
        
    end
    feature_angle_distribution=[feature_angle_distribution; temp];
    clear temp;
end
% %==========画图::叠加画速度分布==========
% figure;
% for i=1:size(feature_v1_norm,2)
%     hold off;
%     draw1=feature_v1_norm(:,i);
%     draw2=feature_v2_norm(:,i);
%     h1=histogram(draw1(draw1>0),'Normalization','probability','edgecolor','b');
%     hold on;
%     h2=histogram(draw2(draw2>0),'Normalization','probability','edgecolor','r');    
%     h1.BinWidth=0.2;
%     h2.BinWidth=0.2;
%     title(['速度分布图:' num2str(i) ' A决策点:' num2str(time_left_longest) ' A总数:' num2str(size(draw1(draw1>0),1)) ' B总数' num2str(size(draw2(draw2>0),1))]);
%     xlabel('velocity(mm/f)');
%     ylabel('probability');
%     legend('A','B');
%     
%     saveas(gca,['../../statistic/velocity_distribution_' num2str(i) '.png']);
%     saveas(gca,['../../statistic/velocity_distribution_' num2str(i) '.fig']);
%     clear draw1 draw2 nbins;
% end
% 
% %==========画图::叠加画法向加速度分布==========
% figure;
% for i=1:size(feature_acc1_norm,2)
%     hold off;
%     draw1=feature_acc1_norm(:,i);
%     draw2=feature_acc2_norm(:,i);
%     h1=histogram(draw1(draw1>0),'Normalization','probability','edgecolor','b');
%     hold on;
%     h2=histogram(draw2(draw2>0),'Normalization','probability','edgecolor','r');
%     h1.BinWidth=0.2;
%     h2.BinWidth=0.2;
%     title(['法向加速度分布图:' num2str(i) ' A决策点:' num2str(time_left_longest) ' A总数:' num2str(size(draw1(draw1>0),1)) ' B总数' num2str(size(draw2(draw2>0),1))]);
%     xlabel('centripetal acceleration(mm/f^2)');
%     ylabel('probability');
%     legend('A','B');
%     saveas(gca,['../../statistic/acc_norm_distribution_' num2str(i) '.png']);
%     saveas(gca,['../../statistic/acc_norm_distribution_' num2str(i) '.fig']);
%     clear draw1 draw2 nbins;
% end
% 
% %==========画图::叠加画切向加速度分布==========
% figure;
% for i=1:size(feature_acc1_tan,2)
%     hold off;
%     draw1=feature_acc1_tan(:,i);
%     draw2=feature_acc2_tan(:,i);
%     h1=histogram(draw1(draw1>0),'Normalization','probability','edgecolor','b');
%     hold on;
%     h2=histogram(draw2(draw2>0),'Normalization','probability','edgecolor','r');
%     h1.BinWidth=0.2;
%     h2.BinWidth=0.2;
%     title(['切向加速度分布图:' num2str(i) ' A决策点:' num2str(time_left_longest) ' A总数:' num2str(size(draw1(draw1>0),1)) ' B总数' num2str(size(draw2(draw2>0),1))]);
%     xlabel('tangential acceleration(mm/f^2)');
%     ylabel('probability');
%     legend('A','B');
%     saveas(gca,['../../statistic/acc_tan_distribution_' num2str(i) '.png']);
%     saveas(gca,['../../statistic/acc_tan_distribution_' num2str(i) '.fig']);
%     clear draw1 draw2;
% end
% 
% %==========画图::叠加画和碰撞点距离分布==========
% figure;
% for i=1:size(feature_dist_s1_crash_point,2)
%     hold off;
%     draw1=feature_dist_s1_crash_point(:,i);
%     draw2=feature_dist_s2_crash_point(:,i);
%     h1=histogram(draw1(draw1>0),'Normalization','probability','edgecolor','b');
%     hold on;
%     h2=histogram(draw2(draw2>0),'Normalization','probability','edgecolor','r');
%     h1.BinWidth=5;
%     h2.BinWidth=5;
%     title(['和碰撞点距离分布图:' num2str(i) ' A决策点:' num2str(time_left_longest) ' A总数:' num2str(size(draw1(draw1>0),1)) ' B总数' num2str(size(draw2(draw2>0),1))]);
%     xlabel('distance(mm)');
%     ylabel('probability');
%     legend('A','B');
%     saveas(gca,['../../statistic/dist_crash_point_distribution_' num2str(i) '.png']);
%     saveas(gca,['../../statistic/dist_crash_point_distribution_' num2str(i) '.fig']);
%     clear draw1 draw2;
% end
% 
% %==========画图::画两者间距离分布==========
% figure;
% for i=1:size(feature_dist_s1_s2,2)
%     hold off;
%     draw1=feature_dist_s1_s2(:,i);
%     h1=histogram(draw1(draw1>0),'Normalization','probability','edgecolor','b');
%     h1.BinWidth=5;
%     xlabel('distance(mm)');
%     ylabel('probability');
%     title(['果蝇间距离分布图:' num2str(i) ' A决策点:' num2str(time_left_longest) ' 总数:' num2str(size(draw1(draw1>0),1))]);
%     saveas(gca,['../../statistic/dist_AB_distribution_' num2str(i) '.png']);
%     saveas(gca,['../../statistic/dist_AB_distribution_' num2str(i) '.fig']);
%     clear draw1;
% end
%     
% 
% %==========画图::画两者间角度分布==========
% figure;
% for i=1:size(feature_angle_distribution,2)
%     hold off;
%     draw1=feature_angle_distribution(:,i);
%     h1=histogram(draw1(draw1>0),'Normalization','probability','edgecolor','b');
%     h1.BinWidth=5;
%     title(['果蝇间速度夹角分布图:' num2str(i) ' A决策点:' num2str(time_left_longest) ' 总数:' num2str(size(draw1(draw1>0),1))]);
%     xlabel('angle(degree)');
%     ylabel('probability');
%     saveas(gca,['../../statistic/angle_distribution_' num2str(i) '.png']);
%     saveas(gca,['../../statistic/angle_distribution_' num2str(i) '.fig']);
%     clear draw1;
% end

%==========画图::决策点到t+n时间分布-正则化=============
figure(2);
temp_time_dist=zeros(1,time_right_longest+1);
regularize_divider=sum(feature_v1_norm>0);
for i=1:size(feature_time_dist(:,2))
    temp_time_dist(feature_time_dist(i,2)+1)=temp_time_dist(feature_time_dist(i,2)+1)+1;
end
for i=1:size(temp_time_dist,2)-3
    temp_time_dist(i)=temp_time_dist(i)/regularize_divider(i+time_strategy+1);
end
hold off;
bar(temp_time_dist(1:end-1));
hold on;
clear temp_time_dist regularize_divider;
title('决策点到t+n时间分布-正则化');
xlabel('time(frame)');
ylabel('normalized count');

% figure(3);
% histogram(feature_time_dist(:,2));


%saveas(gca,['../../statistic/strategy_t+n.png']);
%saveas(gca,['../../statistic/strategy_t+n.fig']);


%==========画图::t到决策点时间分布-正则化=============
figure(3);

temp_time_dist=zeros(1,time_left_longest+1);
regularize_divider=sum(feature_v1_norm>0);
for i=1:size(feature_time_dist(:,1))
    temp_time_dist(feature_time_dist(i,1))=temp_time_dist(feature_time_dist(i,1))+1;
end
for i=1:size(temp_time_dist,2)-3
    temp_time_dist(i)=temp_time_dist(i)/regularize_divider(time_strategy-i);
end
hold off;
bar(temp_time_dist(1:end-1));
hold on;
title('t到决策点时间分布-正则化 ');
xlabel('time(frame)');
ylabel('normalized count');
clear temp_time_dist regularize_divider;


%histogram(feature_time_dist(:,1));


%saveas(gca,['../../statistic/t_strategy.png']);
%saveas(gca,['../../statistic/t_strategy.fig']);



% ==================画图::第三只果蝇数量分布-正则化=============
figure(4);
bin_str_crash_distribution=zeros(1,time_left_longest+1+time_right_longest);
regularize_divider=sum(feature_v1_norm>0);
for i=1:size(feature_str_crash_distribution,2)
    feature_str_crash_distribution(1,i)
    size(bin_str_crash_distribution,2)
    bin_str_crash_distribution(feature_str_crash_distribution(1,i))=bin_str_crash_distribution(feature_str_crash_distribution(1,i))+1;
end
for i=1:size(bin_str_crash_distribution,2)-1
    if regularize_divider(i)<15
        bin_str_crash_distribution(i)=0;
    else
        bin_str_crash_distribution(i)=bin_str_crash_distribution(i)/regularize_divider(i);
    end
end
hold off;
bar(bin_str_crash_distribution);
hold on;
%histogram(feature_str_crash_distribution);
title(['第三只碰撞时间分布 决策点' num2str(time_left_longest+1)]);
xlabel('time(frame)');
ylabel('normalized count');

%saveas(gca,['../../statistic/third.png']);
%saveas(gca,['../../statistic/third.fig']);

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
