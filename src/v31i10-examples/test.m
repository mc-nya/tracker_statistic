close all;
clear all;
clc;


load CV99_neighbor_combine_diff_R_50_single_obj_t1_max;

k = 0;
res = [];
avg_theta_vel_cover = [];
avg_theta_vel_change = [];
avg_theta_acc_cover = [];
avg_theta_acc_change = [];
for i = 1 : size(theta_acc_change, 1)
    idx = find(theta_acc_change(i, :) > 0);
    if length(idx) > 0
        k = k + 1;
        avg_theta_vel_cover(k) = mean(theta_vel_cover(i, idx));
        alpha_deg = theta_vel_cover(i, idx);
        alpha_rad = circ_ang2rad(alpha_deg);
        figure(1)
        subplot(1,2,1)
        circ_plot(alpha_rad,'pretty','bo',true,'linewidth',2,'color','r');
        subplot(1,2,2)
        circ_plot(alpha_rad,'hist',[],20,true,true,'linewidth',2,'color','r')
        saveas(gcf, ['CV99_R_50_max/CV_99_R_50_Obj_', num2str(i), '.jpg']);
        hold off;
        close all;
%         tic;
    end
%     if length(idx) > 0
%         k = k + 1;
%         avg_theta_vel_cover(k) = mean(theta_vel_cover(i, idx));
%         avg_theta_vel_change(k) = mean(theta_vel_change(i, idx));
%         avg_theta_acc_cover(k) = mean(theta_acc_cover(i, idx));
%         avg_theta_acc_change(k) = mean(theta_acc_change(i, idx));
%     end
end


hist(avg_theta_vel_cover)

