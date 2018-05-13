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
        v_past=velocity(1:3,i);%����ٶ��йص��õ�ǰ֡v
        a=acc(1:3,i);       
        %=====�����ٶȺͼ��ٶȴ�С======
        v_norm=[v_norm, norm(velocity(1:3,i))];
        acc_norm=[acc_norm, norm(acc(1:3,i))];       
        %======������ٶ�====
        acc_on_v_past=[acc_on_v_past, dot(a,v_past)/norm(v_past)];
        %======������ٶ�================
        acc_on_v_past_norm=[acc_on_v_past_norm, sqrt(norm(a)^2-acc_on_v_past(end)^2)];
        %===========�����˶����ʰ뾶===============
        r=[r norm(v)^2/acc_on_v_past_norm(end)];     
    end      
    for i=2:size(acc,2)
        %==========��Ǳ�����ʼ��=========
        v=velocity(1:3,i);
        v_past=velocity(1:3,i-1);
        a=acc(1:3,i);
        a_past=acc(1:3,i-1);
        %======�����ٶ������һ֡�ٶȵı仯�Ƕ�==========
        v_ang=[v_ang acosd(dot(v,v_past)/norm(v)/norm(v_past))];  
        %======������ٶ������һ֡���ٶȵı仯�Ƕ�==========
        acc_ang=[acc_ang acosd(dot(a,a_past)/norm(a)/norm(a_past))];
    end

end

