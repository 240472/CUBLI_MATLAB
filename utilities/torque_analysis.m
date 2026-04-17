clear all;
close all;
clc;


% Check filename, trim_time and indeces from which are data taken
load("work\torque_analysis_005.mat")

trim_time_begin = 31;
trim_time_end = 640; 
transient_time = 0.15;
% KV = 330;



% Reading data
time = data{1}.Values.Time;
Ts = time(2) - time(1);

wheel_speed_rpm = data{3}.Values.Data(:,1); 
meased_Iq_A = data{4}.Values.Data(:,1);
target_Iq_A = data{9}.Values.Data(:,1);
applied_torque_Nm = data{7}.Values.Data; 
target_torque_Nm = data{8}.Values.Data; 

rot_accels_rads2 = data{13}.Values.Data; 

% Trimming data
data_filter_idx = (meased_Iq_A > 1e-9);

wheel_speed_rpm = wheel_speed_rpm .* data_filter_idx;
target_Iq_A = target_Iq_A .* data_filter_idx;
applied_torque_Nm = applied_torque_Nm .* data_filter_idx;
target_torque_Nm = target_torque_Nm .* data_filter_idx;

rot_accels_rads2 = rot_accels_rads2 .* data_filter_idx;

trim_idx_begin = find((time >= trim_time_begin), 1);
trim_idx_end = find((time >= trim_time_end), 1);

time = time(trim_idx_begin:trim_idx_end);
time = time - time(1);

wheel_speed_rpm = wheel_speed_rpm(trim_idx_begin:trim_idx_end);
meased_Iq_A = meased_Iq_A(trim_idx_begin:trim_idx_end);
target_Iq_A = target_Iq_A(trim_idx_begin:trim_idx_end);
applied_torque_Nm = applied_torque_Nm(trim_idx_begin:trim_idx_end);
target_torque_Nm = target_torque_Nm(trim_idx_begin:trim_idx_end);

rot_accels_rads2 = rot_accels_rads2(trim_idx_begin:trim_idx_end);



% % Finding mean for each torque segment
target_Iq_A_segment_idx_begin = find(diff(target_Iq_A) > 0) + 1;
target_Iq_A_segment_idx_end = find(diff(target_Iq_A) < 0) + 1;

torque_segments = [target_Iq_A_segment_idx_begin, target_Iq_A_segment_idx_end, target_torque_Nm(target_Iq_A_segment_idx_begin)];
torque_segments = torque_segments(torque_segments(:,2)-torque_segments(:,1) > 10,:);

for i = 1:size(torque_segments,1)

    begin_idx = torque_segments(i,1) + floor(transient_time/Ts);
    end_idx = torque_segments(i,2);

    torque_segments(i,4) = mean(applied_torque_Nm(begin_idx:end_idx));
end



% Linear approximation

torque_linear_fit = polyfit(torque_segments(:,3), torque_segments(:,4), 1);
torque_fit_y = polyval(torque_linear_fit,torque_segments(:,3));

torque_inverse_linear_fit = polyfit(torque_segments(:,4), torque_segments(:,3), 1);
torque_inverse_fit_x = polyval(torque_inverse_linear_fit,torque_segments(:,3));

torque_fit_better_y = polyval(torque_linear_fit,torque_inverse_fit_x);




subplot(5,1,1)
plot(time, [target_Iq_A, meased_Iq_A])
subplot(5,1,2)
plot(time, wheel_speed_rpm)
subplot(5,1,3)
plot(time, [target_torque_Nm, applied_torque_Nm])
subplot(5,1,4)
plot(torque_segments(:,3), [torque_segments(:,4), torque_fit_y])
subplot(5,1,5)
plot(torque_inverse_fit_x, torque_fit_better_y)


fprintf("Torque slope is %.4f SOMETHING.\n",torque_linear_fit(1));
fprintf("Torque inverse slope is %.4f SOMETHING.\n",torque_inverse_linear_fit(1));