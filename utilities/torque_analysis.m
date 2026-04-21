clear all;
close all;
clc;

%file_names = ["work\torque_analysis_005_negative_v3.mat", "work\torque_analysis_005_v3.mat"];
% Check filename, trim_time and indeces from which are data taken
load("work\torque_analysis_005_negative.mat")

file_names = ["work\torque_analysis_005_correction_v2.mat"];

merged_torque_segments = [];

for k = 1:size(file_names,1)

load(file_names(k));

trim_time_begin = 40;
trim_time_end = 615; 
transient_time = 0.15;
% KV = 330;

% Reading data
time = data{1}.Values.Time;
Ts = time(2) - time(1);
for j = 1:data.numElements
    switch data{j}.Name
        case "wheel_speed [rpm]"
            wheel_speed_rpm = data{j}.Values.Data(:,1);
        case "meased_Iq [A]"
            meased_Iq_A = data{j}.Values.Data(:,1);
        case "target_Iq [A]"
            target_Iq_A = data{j}.Values.Data(:,1);
        case "applied_torque [Nm]"
            applied_torque_Nm = data{j}.Values.Data; 
        case "target_torque [Nm]"
            target_torque_Nm = data{j}.Values.Data; 
        case "rot_accel [rad/s^2]"
            rot_accels_rads2 = data{j}.Values.Data;
        case "next_state"
            state = data{j}.Values.Data;    
    end
end

% Trimming data
data_filter_idx = (abs(meased_Iq_A) > 1e-9);

%state = state .* StateEnum(data_filter_idx);
wheel_speed_rpm = wheel_speed_rpm .* data_filter_idx;
target_Iq_A = target_Iq_A .* data_filter_idx;
applied_torque_Nm = applied_torque_Nm .* data_filter_idx;
target_torque_Nm = target_torque_Nm .* data_filter_idx;

rot_accels_rads2 = rot_accels_rads2 .* data_filter_idx;

trim_idx_begin = find((time >= trim_time_begin), 1);
trim_idx_end = find((time >= trim_time_end), 1);

time = time(trim_idx_begin:trim_idx_end);
time = time - time(1);

state = state(trim_idx_begin:trim_idx_end);
wheel_speed_rpm = wheel_speed_rpm(trim_idx_begin:trim_idx_end);
meased_Iq_A = meased_Iq_A(trim_idx_begin:trim_idx_end);
target_Iq_A = target_Iq_A(trim_idx_begin:trim_idx_end);
applied_torque_Nm = applied_torque_Nm(trim_idx_begin:trim_idx_end);
target_torque_Nm = target_torque_Nm(trim_idx_begin:trim_idx_end);

rot_accels_rads2 = rot_accels_rads2(trim_idx_begin:trim_idx_end);


% % Finding mean for each torque segment
target_Iq_A_segment_idx_begin = find(state == StateEnum.StartingMotors) + 1;
target_Iq_A_segment_idx_end = find(state == StateEnum.StoppingMotors) + 1;

torque_segments = [target_Iq_A_segment_idx_begin, target_Iq_A_segment_idx_end, target_torque_Nm(target_Iq_A_segment_idx_begin + 5)];
torque_segments = torque_segments(torque_segments(:,2)-torque_segments(:,1) > 10,:);

for i = 1:size(torque_segments,1)

    begin_idx = torque_segments(i,1) + floor(transient_time/Ts);
    end_idx = torque_segments(i,2);

    torque_segments(i,4) = mean(applied_torque_Nm(begin_idx:end_idx));
end

if k == 1   
    merged_torque_segments = [merged_torque_segments; flip(torque_segments)];
else
    merged_torque_segments = [merged_torque_segments; torque_segments];
end

figure(k)
subplot(5,1,1)
plot(time, [target_Iq_A, meased_Iq_A])
subplot(5,1,2)
plot(time, wheel_speed_rpm)
subplot(5,1,3)
plot(time, [target_torque_Nm, applied_torque_Nm])

end

% Linear approximation

% Fit f: commanded -> measured
f = polyfit(merged_torque_segments(:,3), merged_torque_segments(:,4), 3);

% Fit f⁻¹: measured -> commanded  (this is what you apply in real-time)
f_inv = polyfit(merged_torque_segments(:,4), merged_torque_segments(:,3), 3);

% Apply f⁻¹ then f — result should be close to original y values
y_check = polyval(f, polyval(f_inv, merged_torque_segments(:,3)));

% Plot to verify
figure;
plot(merged_torque_segments(:,3), merged_torque_segments(:,3), 'k--'); % ideal y=x
hold on;
plot(merged_torque_segments(:,3), y_check, 'r-');
xlabel('Desired torque');
ylabel('Reconstructed torque');
legend('Ideal', 'f(f^{-1}(y))');
grid on;

f(4) = 0;
f_inv(4) = 0;

figure
x_lin = linspace(-0.05,0.05,1000);
plot(x_lin, [polyval(f,polyval(f_inv,x_lin)); polyval(f,x_lin)])
legend("f_corrected", "f")

% subplot(5,1,4)
% plot(merged_torque_segments(:,3), [merged_torque_segments(:,4), torque_fit_y])
% subplot(5,1,5)
% plot(torque_inverse_fit_x, torque_fit_better_y)
% 
% 
% fprintf("Torque slope is %.4f SOMETHING.\n",torque_linear_fit(1));
% fprintf("Torque inverse slope is %.4f SOMETHING.\n",torque_inverse_linear_fit(1));