

simulink_Ts = 0.01; %must be shortest
sensor_Ts = 0.01;
kalman_Ts = 0.01;

ICM42_calib_time = 5; %time to sense gyro offset and g...don't move with anything!!!
avg_order = 4;

%% Initial parameters for fusion algorithms
% (1,1) P_initial      = 1x1 initial error covariance matrix
% (1,2) SIGMA_g        = gyrsocope noise for process noise covariance matrix static
% (1,3) SIGMA_g        = gyrsocope noise for process noise covariance matrix dynamic
% (1,4) epsilon_g      = threshold for switching between (1,2) & (1,3)
% (1,5) stdBiasAcc_in  = 1x1 (m/s^2) initial value for Accelerometer bias covariance matrix
% (1,6) stdBiasMag_in  = 1x1 (a.u.) initial value for magnetometer bias covariance matrix
% (1,7) stdAcc         = accelerometer noise for measurement noise covariance matrix static
% (1,8) stdAcc         = accelerometer noise for measurement noise covariance matrix dynamic
% (1,9) epsilon_a      = threshold for switching between (1,7) & (1,8)
% (1,10) stdMag        = magnetometer noise for measurement noise covariance matrix static
% (1,11) stdMag        = magnetometer noise for measurement noise covariance matrix dynamic
% (1,12) epsilon_m     = threshold for switching between (1,10) & (1,11)

P_initial = 1e6;
SIGMA_static = 1e6;
SIGMA_dynamic = deg2rad(0.0386);
epsilon_g = 100*pi/180;
stdBiasAcc_in = 0.01;  
stdBiasMag_in = 1e3;  
stdAcc_static = 0.008;  
stdAcc_dynamic = 1e6; 
epsilon_a = 0.3;       
stdMag_static = 1;  
stdMag_dynamic = 1e6; 
epsilon_m = 0.1;    

params_1period = [SIGMA_static, SIGMA_dynamic, epsilon_g, stdBiasAcc_in, stdAcc_static, stdAcc_dynamic, epsilon_a];
params_batch = [P_initial, SIGMA_static, SIGMA_dynamic, epsilon_g, stdBiasAcc_in, stdBiasMag_in, stdAcc_static, stdAcc_dynamic, epsilon_a, stdMag_static, stdMag_dynamic, epsilon_m];
ICM42_params = params_1period;


init_rot_quat = [0 0 0 1]';
init_acc_bias = [0 0 0]';
init_P = P_initial;

ICM42_gyr_offset = [-0.1319, -0.6878, 0.1737];  %init value, after rest period measured by sensors
ICM42_g = 9.8121;                               %init value, after rest period measured by sensors
