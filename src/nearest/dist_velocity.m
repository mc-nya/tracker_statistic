delta_time=8;
detect_set_washed=dist_velocity_filter(trackerW,detect_set,delta_time);
feature_vector=[];
feature_v1_norm=[];
feature_acc1_norm=[];
feature_acc1_on_v1_past=[];
feature_acc1_on_v1_past_norm=[];
feature_v1_ang=[];
feature_dist_velocity=[];
for outer_index=1:size(detect_set_washed)
    pair=detect_set_washed(outer_index,:);
    
    time_nearest=pair(1);
    index1=pair(2);
    index2=pair(3);
    states1=trackerW(index1).states(1:3,:);
    states2=trackerW(index2).states(1:3,:);
    timer1=trackerW(index1).start:trackerW(index1).end;
    timer2=trackerW(index2).start:trackerW(index2).end;
    time_both_start=max(timer1(1),timer2(1));
    time_both_end=min(timer1(end),timer2(end));
    
    if delta_time~=0
        
        start_time1=max(timer1(1),time_nearest-delta_time);
        start_time2=max(timer2(1),time_nearest-delta_time);
        end_time1=min(timer1(end),time_nearest+2);
        end_time2=min(timer2(end),time_nearest+2);
        states1=states1(1:3,find(timer1==start_time1):find(timer1==end_time1));
        states2=states2(1:3,find(timer2==start_time2):find(timer2==end_time2));
        timer1=timer1(find(timer1==start_time1):find(timer1==end_time1));
        timer2=timer2(find(timer2==start_time2):find(timer2==end_time2));
    end
    
    velocity1=states1(1:3,2:end)-states1(1:3,1:end-1);
    velocity2=states2(1:3,2:end)-states2(1:3,1:end-1);
    acc1=velocity1(1:3,2:end)-velocity1(1:3,1:end-1);
    acc2=velocity2(1:3,2:end)-velocity2(1:3,1:end-1);
    timer1=timer1(1:end-2);
    timer2=timer2(1:end-2);
    
    v1_norm=[];
    acc1_norm=[];
    acc1_on_v1_past=[];
    v1_ang=[];
    acc1_ang=[];
    acc1_on_v1_past_norm=[];
    
    r1=[];
    av1=[];
    for i=1:size(acc1,2)
        v1=velocity1(1:3,i);
        v1_past=velocity1(1:3,i);%与加速度有关的用当前帧v
        a1=acc1(1:3,i);       
        %=====计算速度和加速度大小======
        v1_norm=[v1_norm, norm(velocity1(1:3,i))];
        acc1_norm=[acc1_norm, norm(acc1(1:3,i))];       
        %======切向加速度====
        acc1_on_v1_past=[acc1_on_v1_past, dot(a1,v1_past)/norm(v1_past)];
        %======法向加速度================
        acc1_on_v1_past_norm=[acc1_on_v1_past_norm, sqrt(norm(a1)^2-acc1_on_v1_past(end)^2)];
        %===========计算运动曲率半径===============
        r1=[r1 norm(v1)^2/acc1_on_v1_past_norm(end)];     
    end      
    for i=2:size(acc1,2)
        %==========简记变量初始化=========
        v1=velocity1(1:3,i);
        v1_past=velocity1(1:3,i-1);
        a1=acc1(1:3,i);
        a1_1=acc1(1:3,i-1);
        
        %======计算速度相对上一帧速度的变化角度==========
        v1_ang=[v1_ang acosd(dot(v1,v1_past)/norm(v1)/norm(v1_past))];  
        %======计算加速度相对上一帧加速度的变化角度==========
        acc1_ang=[acc1_ang acosd(dot(a1,a1_1)/norm(a1)/norm(a1_1))];
    end
    %-----calc distance----------
    timer_both=max(timer1(1),timer2(1)):min(timer1(end),timer2(end));
    dist_s1_s2=[];
    for i=1:size(timer_both,2)
        s1=states1(1:3,find(timer1==timer_both(i)));
        s2=states2(1:3,find(timer2==timer_both(i)));
        dist_s1_s2=[dist_s1_s2; norm(s1-s2)];
    end
    
    feature_dist_velocity=[feature_dist_velocity; [dist_s1_s2 acc1_on_v1_past_norm(find(timer1==timer_both(1)):find(timer1==timer_both(end)))']];
    
    feature_v1_norm=[feature_v1_norm; v1_norm];
    feature_acc1_norm=[feature_acc1_norm; acc1_norm];
    feature_acc1_on_v1_past=[feature_acc1_on_v1_past; acc1_on_v1_past];
    feature_acc1_on_v1_past_norm=[feature_acc1_on_v1_past_norm; acc1_on_v1_past_norm];
    feature_v1_ang=[feature_v1_ang; v1_ang];
    %feature_vector=[feature_vector; v1_norm];

end
feature_vector=feature_v1_norm;
portion=0.90;

[pc,score,latent,tsquare,explained] =pca(feature_vector);
percentage_list=cumsum(latent)./sum(latent);
dimesion=find(percentage_list>portion);
tran=pc(:,1:dimesion(1));
feature_vector= bsxfun(@minus,feature_vector,mean(feature_vector,1));
feature_vector= feature_vector*tran;
median_vector=median(feature_vector,1);
figure(1);

hold on;

for i=1:size(feature_vector)
    
    plot3(feature_vector(i,1),feature_vector(i,2),feature_vector(i,3),'r+');
end
plot3(median_vector(1),median_vector(2),median_vector(3),'bh');

diff_vec=feature_vector-repmat(median_vector,size(feature_vector,1),1);
dist_vec=[];
for i=1:size(diff_vec)
    dist_vec=[dist_vec norm(diff_vec(i,:))];
end
dist_median=median(dist_vec);
selected_item=find(dist_vec<dist_median);
%selected_item=intersect(find(feature_vector(:,1)<0),find(feature_vector(:,2)>0));

%----------------using pca result, revise detect_set-----------
feature_vector=feature_vector(selected_item,:);
detect_set_washed=detect_set_washed(selected_item,:);
feature_v1_norm=feature_v1_norm(selected_item,:);
feature_acc1_norm=feature_acc1_norm(selected_item,:);
feature_acc1_on_v1_past=feature_acc1_on_v1_past(selected_item,:);
feature_acc1_on_v1_past_norm=feature_acc1_on_v1_past_norm(selected_item,:);
%feature_v1_ang=feature_v1_ang(selected_item,:);


feature_v1_norm=[];
feature_acc1_norm=[];
feature_acc1_on_v1_past=[];
feature_acc1_on_v1_past_norm=[];
feature_v1_ang=[];
feature_dist_velocity=[];
for outer_index=1:size(detect_set_washed)
    pair=detect_set_washed(outer_index,:);
    
    time_nearest=pair(1);
    index1=pair(2);
    index2=pair(3);
    states1=trackerW(index1).states(1:3,:);
    states2=trackerW(index2).states(1:3,:);
    timer1=trackerW(index1).start:trackerW(index1).end;
    timer2=trackerW(index2).start:trackerW(index2).end;
    time_both_start=max(timer1(1),timer2(1));
    time_both_end=min(timer1(end),timer2(end));
    
    if delta_time~=0
        
        start_time1=max(timer1(1),time_nearest-delta_time);
        start_time2=max(timer2(1),time_nearest-delta_time);
        end_time1=min(timer1(end),time_nearest+2);
        end_time2=min(timer2(end),time_nearest+2);
        states1=states1(1:3,find(timer1==start_time1):find(timer1==end_time1));
        states2=states2(1:3,find(timer2==start_time2):find(timer2==end_time2));
        timer1=timer1(find(timer1==start_time1):find(timer1==end_time1));
        timer2=timer2(find(timer2==start_time2):find(timer2==end_time2));
    end
    
    velocity1=states1(1:3,2:end)-states1(1:3,1:end-1);
    velocity2=states2(1:3,2:end)-states2(1:3,1:end-1);
    acc1=velocity1(1:3,2:end)-velocity1(1:3,1:end-1);
    acc2=velocity2(1:3,2:end)-velocity2(1:3,1:end-1);
    timer1=timer1(1:end-2);
    timer2=timer2(1:end-2);
    
    v1_norm=[];
    acc1_norm=[];
    acc1_on_v1_past=[];
    v1_ang=[];
    acc1_ang=[];
    acc1_on_v1_past_norm=[];
    
    r1=[];
    av1=[];
    for i=1:size(acc1,2)
        v1=velocity1(1:3,i);
        v1_past=velocity1(1:3,i);%与加速度有关的用当前帧v
        a1=acc1(1:3,i);       
        %=====计算速度和加速度大小======
        v1_norm=[v1_norm, norm(velocity1(1:3,i))];
        acc1_norm=[acc1_norm, norm(acc1(1:3,i))];       
        %======切向加速度====
        acc1_on_v1_past=[acc1_on_v1_past, dot(a1,v1_past)/norm(v1_past)];
        %======法向加速度================
        acc1_on_v1_past_norm=[acc1_on_v1_past_norm, sqrt(norm(a1)^2-acc1_on_v1_past(end)^2)];
        %===========计算运动曲率半径===============
        r1=[r1 norm(v1)^2/acc1_on_v1_past_norm(end)];     
    end      
    for i=2:size(acc1,2)
        %==========简记变量初始化=========
        v1=velocity1(1:3,i);
        v1_past=velocity1(1:3,i-1);
        a1=acc1(1:3,i);
        a1_1=acc1(1:3,i-1);
        
        %======计算速度相对上一帧速度的变化角度==========
        v1_ang=[v1_ang acosd(dot(v1,v1_past)/norm(v1)/norm(v1_past))];  
        %======计算加速度相对上一帧加速度的变化角度==========
        acc1_ang=[acc1_ang acosd(dot(a1,a1_1)/norm(a1)/norm(a1_1))];
    end
    %-----calc distance----------
    timer_both=max(timer1(1),timer2(1)):min(timer1(end),timer2(end));
    dist_s1_s2=[];
    for i=1:size(timer_both,2)
        s1=states1(1:3,find(timer1==timer_both(i)));
        s2=states2(1:3,find(timer2==timer_both(i)));
        dist_s1_s2=[dist_s1_s2; norm(s1-s2)];
    end
    
    feature_dist_velocity=[feature_dist_velocity; [dist_s1_s2 v1_norm(find(timer1==timer_both(1)):find(timer1==timer_both(end)))']];
    
    feature_v1_norm=[feature_v1_norm; v1_norm];
    feature_acc1_norm=[feature_acc1_norm; acc1_norm];
    feature_acc1_on_v1_past=[feature_acc1_on_v1_past; acc1_on_v1_past];
    feature_acc1_on_v1_past_norm=[feature_acc1_on_v1_past_norm; acc1_on_v1_past_norm];
    feature_v1_ang=[feature_v1_ang; v1_ang];
    %feature_vector=[feature_vector; v1_norm];

end





figure(2);

hold on;
for i=1:size(feature_vector)
    
    plot3(feature_vector(i,1),feature_vector(i,2),feature_vector(i,3),'r+');
end
plot3(median_vector(1),median_vector(2),median_vector(3),'bh');

figure(3);
feature_vector=feature_v1_norm;
hold on;
for i=1:size(feature_vector)
    %plot(feature_vector(i,:),'k');
    for j=1:size(feature_vector,2)
        plot(j,feature_vector(i,j),'b.');
    end
end
plot(mean(feature_vector),'r+');
plot(median(feature_vector),'r*');
plot(std(feature_vector)+mean(feature_vector),'r')
plot(-std(feature_vector)+mean(feature_vector),'r')

figure(4);
hold on;
plot(feature_dist_velocity(:,1),feature_dist_velocity(:,2),'k.');


