function [ v_norm,acc_norm, acc_on_v_past, acc_on_v_past_norm, r, v_ang, acc_ang] = calc_trace_attribute( states )


    velocity=states(1:3,2:end)-states(1:3,1:end-1);
    acc=velocity(1:3,2:end)-velocity(1:3,1:end-1);
    
    v_norm=[];
    acc_norm=[];
    acc_on_v_past=[];
    v_ang=[];
    acc_ang=[];
    acc_on_v_past_norm=[];
    
    r=[];

    for i=1:size(acc,2)
        v=velocity(1:3,i);
        v_past=velocity(1:3,i);%与加速度有关的用当前帧v
        a=acc(1:3,i);       
        %=====计算速度和加速度大小======
        v_norm=[v_norm, norm(velocity(1:3,i))];
        acc_norm=[acc_norm, norm(acc(1:3,i))];       
        %======切向加速度====
        acc_on_v_past=[acc_on_v_past, dot(a,v_past)/norm(v_past)];
        %======法向加速度================
        acc_on_v_past_norm=[acc_on_v_past_norm, sqrt(norm(a)^2-acc_on_v_past(end)^2)];
        %===========计算运动曲率半径===============
        r=[r norm(v)^2/acc_on_v_past_norm(end)];     
    end      
    for i=2:size(acc,2)
        %==========简记变量初始化=========
        v=velocity(1:3,i);
        v_past=velocity(1:3,i-1);
        a=acc(1:3,i);
        a_past=acc(1:3,i-1);
        %======计算速度相对上一帧速度的变化角度==========
        v_ang=[v_ang acosd(dot(v,v_past)/norm(v)/norm(v_past))];  
        %======计算加速度相对上一帧加速度的变化角度==========
        acc_ang=[acc_ang acosd(dot(a,a_past)/norm(a)/norm(a_past))];
    end

end

