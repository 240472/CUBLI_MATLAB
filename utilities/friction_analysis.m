clear all;
close all;
clc;


% Check filename, trim_time and indeces from which are data taken
load("work\friction_analysis_positive_speeds.mat")
trim_time = 24;
transient_time = 1.5;
KV = 330;

hok = [71.78, 19.84];


% Reading data
time = data{1}.Values.Time;
Ts = time(2) - time(1);

target_speed_rpm = data{7}.Values.Data; 
wheel_speed_rpm = data{3}.Values.Data(:,1); 
meased_Iq_A = data{4}.Values.Data(:,1);


% Trimming data
target_speed_rpm = target_speed_rpm .* (wheel_speed_rpm ~= 0);

trim_idx = find((time >= trim_time), 1);

time = time(trim_idx:end);
time = time - time(1);

target_speed_rpm = target_speed_rpm(trim_idx:end);
wheel_speed_rpm = wheel_speed_rpm(trim_idx:end);

meased_Iq_A = meased_Iq_A(trim_idx:end);
meased_Iq_T = meased_Iq_A * (1/((2*pi*KV)/60));


% Finding mean for each speed segment
speed_change_idx = find(diff(target_speed_rpm) ~= 0) + 1;
speed_segments = [speed_change_idx, target_speed_rpm(speed_change_idx)];

for i = 1:size(speed_segments,1) - 1 

    begin_idx = speed_change_idx(i) + floor(transient_time/Ts);
    end_idx = speed_change_idx(i + 1);

    speed_segments(i,3) = mean(meased_Iq_T(begin_idx:end_idx));
end

% Linear approximation
speed_segments = speed_segments(1:end - 1, :);
speed_segments(:,2) = speed_segments(:,2) * (2*pi/60);

friction_linear_fit = polyfit(speed_segments(:,2), speed_segments(:,3), 1);
friction_fit_y = polyval(friction_linear_fit,speed_segments(:,2));

subplot(3,1,1)
plot(time, [target_speed_rpm, wheel_speed_rpm])
subplot(3,1,2)
plot(time, meased_Iq_T)
subplot(3,1,3)
plot(speed_segments(:,2), [speed_segments(:,3), friction_fit_y])

fprintf("Friction slope is %.4f μN·m·s.\n",friction_linear_fit(1)*1e6);