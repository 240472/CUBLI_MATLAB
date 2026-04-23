
% clear mex; clear classes; clear functions;
% rehash toolboxcache;
% sl_refresh_customizations;


stateInit = StateEnum.Initialization;

simulink_Ts = 0.01; %must be the shortest
sensor_Ts = 0.01;
kalman_Ts = 0.01;

ICM42_calib_time = 10; %time to sense gyro offset and g...don't move with anything!!!
init_time = 10;
calibration_time = 10;
avg_order = 2;

KV = 330;                   % motor constant RPM/V
friction_slope = 20e-6;
friction_offset = 0.0001;

torque_inv_function = [0.162097095719061	0.067726653685203	0.495960303984765	0];



Iwg = 315e-6;              %moment of inertia of WHEEL to its center of gravity 3.75e-3 3.75e-4
Isg = 125e-6;              %moment of inertia of STRUCTURE to its center of gravity
mw = 0.15;                  %weight of WHEEL
ms = 0.6;                   %weight of STRUCTURE
d = 0.15*sqrt(2)/2;         %distance from spinning axis to center of gravity 
g = 9.81;

Iw0 = Iwg + mw * d^2;       %parallel axis theorem (Steinerova věta)
Is0 = Isg + ms * d^2;

mc = mw + ms;               %weight of CUBE

Ic0 = Is0 + Iw0 - Iwg;      %moment of inertia of CUBE (without Iwg)


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

P_initial = 10;
SIGMA_static = 10*deg2rad(0.0386);
SIGMA_dynamic = deg2rad(0.0386); %deg2rad(0.0386)
epsilon_g = 300*pi/180;
stdBiasAcc_in = 5e-2;  %5e-2
stdBiasMag_in = 1e3;  
stdAcc_static = 0.008; %0.008  
stdAcc_dynamic = 0.08; 
epsilon_a = 0.05;       
stdMag_static = 1;  
stdMag_dynamic = 1e6; 
epsilon_m = 0.1;    

SUPER_PARAMS = [deg2rad(0.2), deg2rad(0.2), 300*pi/180, 0, 0.1, 1e24, 0.02];

params_1period = [SIGMA_static, SIGMA_dynamic, epsilon_g, stdBiasAcc_in, stdAcc_static, stdAcc_dynamic, epsilon_a];
params_batch = [P_initial, SIGMA_static, SIGMA_dynamic, epsilon_g, stdBiasAcc_in, stdBiasMag_in, stdAcc_static, stdAcc_dynamic, epsilon_a, stdMag_static, stdMag_dynamic, epsilon_m];
ICM42_params = SUPER_PARAMS;


init_rot_quat = [0 0 0 1]';
init_acc_bias = [0 0 0]';
init_P = P_initial;

ICM42_gyr_offset = [-0.1319, -0.6878, 0.1737];  %init value, after rest period measured by sensors
ICM42_g = 9.8121;                               %init value, after rest period measured by sensors



% P_initial = 1e3;
% SIGMA_static = 1e3;
% SIGMA_dynamic = 0.1; %deg2rad(0.0386)
% epsilon_g = 100*pi/180;
% stdBiasAcc_in = 5e-1;  
% stdBiasMag_in = 1e3;  
% stdAcc_static = 5e-2; %0.008  
% stdAcc_dynamic = 5e2; 
% epsilon_a = 0.05;       
% stdMag_static = 1;  
% stdMag_dynamic = 1e6; 
% epsilon_m = 0.1;    
