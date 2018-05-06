delta_time=12;
detect_set_washed=trace_filter(trackerW,detect_set,delta_time);
analysis_vector=[];
for outer_index=1:size(detect_set_washed)
    pair=detect_set_washed(outer_index,:);
    
    time_nearest=pair(1);
    index1=pair(2);    
    states1=trackerW(index1).states(1:3,:);
    timer1=trackerW(index1).start:trackerW(index1).end;   

    
    if delta_time~=0
        start_time1=max(timer1(1),time_nearest-delta_time);
        end_time1=min(timer1(end),time_nearest+delta_time);
        states1=states1(1:3,find(timer1==start_time1):find(timer1==end_time1));
        timer1=timer1(find(timer1==start_time1):find(timer1==end_time1));    
    end
    
    velocity1=states1(1:3,2:end)-states1(1:3,1:end-1);
    acc1=velocity1(1:3,2:end)-velocity1(1:3,1:end-1);
    
    v1_norm=[];
    acc1_norm=[];
    v1_on_v1_past=[];
    acc1_on_v1_past=[];
    v1_ang=[];
    acc1_ang=[];
    acc1_on_v1_past_norm=[];
    dist_s1_crash_point=[];
    
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
    
    analysis_vector=[analysis_vector; v1_norm];
end

[pc,score,latent,tsquare,explained] =pca(analysis_vector);
percentage=cumsum(latent)./sum(latent);
tran=pc(:,1:7);
analysis_vector= bsxfun(@minus,analysis_vector,mean(analysis_vector,1));
analysis_vector= analysis_vector*tran;
figure(1)
hold off;
plot3(0,0,0);
hold on;
for i=1:size(feature_after_PCA)
    
    plot3(analysis_vector(i,1),analysis_vector(i,2),analysis_vector(i,3),'r+');
end
    
[pc,score,latent,tsquare,explained] =pca(analysis_vector);
percentage=cumsum(latent)./sum(latent);
tran=pc(:,1:3);
analysis_vector= bsxfun(@minus,analysis_vector,mean(analysis_vector,1));
analysis_vector= analysis_vector*tran;
figure(1)
hold off;
plot3(0,0,0);
hold on;
for i=1:size(feature_after_PCA)
    
    plot3(analysis_vector(i,1),analysis_vector(i,2),analysis_vector(i,3),'r+');
end

