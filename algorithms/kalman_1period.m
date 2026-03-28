function [rot_quat, acc_bias, P]  = kalman_1period(data, IMU_Fs, Param, g, rot_quat, acc_bias, P)
%#codegen

% This function implements  the sensor fusion algorithm proposed in
% "Quaternion-Based Extended Kalman Filter for Determining Orientation by Inertial and Magnetic Sensing"
% by Angelo M. Sabatini, IEEE TRANSACTIONS ON BIOMEDICAL ENGINEERING, VOL. 53, NO. 7, JULY 2006
% &
% Estimating Three-Dimensional Orientation of Human Body Parts by Inertial/Magnetic Sensing
% by Angelo Maria Sabatini, Sensors 2011, 11, 1489-1525; doi:10.3390/s110201489

%%% This code originally implemented by
% Marco Caruso (Politecnico di Torino), marco.caruso@studenti.polito.it
% Date: 21/11/2019

% The ORIGINAL code was modified by Milad Nazarahari
% (nazaraha@ualberta.ca), during Ph.D. studies under supervision of Dr. H. Rouhani
% (hrouhani@ualberta.ca), University of Alberta, December 2020 as a part of
% a comparative/benchmarking study.

% The MATLAB code performs the operations of the ORIGINAL code except for:
% reference acceleration/magnetic field (Earth's magnetic field)
% was obtained from sensor readout during motionless perioed at the begining of the experiment
% & implementing adaptive parameter tuning (hard-switch) & accelerometer and
% magnetometer bias were added as states

%  --------------- INPUT ---------------
%%%% "data" including
% acc            = 1x3 (m/s^2)
% gyr            = 1x3 (rad/s)
% Fs             = sampling frequency of the IMU (Hz)
% QS             = 1x2 (s) motionless period at the begining of the trial

%%%% Params
% (1,1) SIGMA_g        = gyrsocope noise for process noise covariance matrix static
% (1,2) SIGMA_g        = gyrsocope noise for process noise covariance matrix dynamic
% (1,3) epsilon_g      = threshold for switching between (1,1) & (1,2)
% (1,4) stdBiasAcc_in  = 1x1 (m/s^2) initial value for Accelerometer bias covariance matrix
% (1,5) stdAcc         = accelerometer noise for measurement noise covariance matrix static
% (1,6) stdAcc         = accelerometer noise for measurement noise covariance matrix dynamic
% (1,7) epsilon_a      = threshold for switching between (1,5) & (1,6)

% --------------- OUTPUT ---------------
% rot_quat             = Nx4 [qx qy qz qw], the scalar part is at the END of the quaternion


%-------------NEEDS TO BE INITIALIAZED-----------------

% rot_quat = zeros(len, 4);
% P = Param(1)*eye(7); % initial error covariance matrix
% g = norm(mean(accel(QS(1)*IMU_Fs:QS(2)*IMU_Fs, :))); % gravitational acceleration


accel = data(1:3);
gyro = deg2rad(data(4:6));
% len = size(accel,1);
dt = 1/IMU_Fs;

quaternion = [rot_quat' acc_bias']';

% PROCESS NOISE
SIGMA_a = dt*Param(4)^2*eye(3);



    % Updating Gyro noise adaptively (Q)
    if norm(gyro) <= Param(3)
        SIGMA_g = Param(1)^2*eye(3);
    else
        SIGMA_g = Param(2)^2*eye(3);
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%% PREDICTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    wx = gyro(1); wy = gyro(2); wz = gyro(3);
    omega = 0.5*[0    wz -wy wx
        -wz    0     wx wy
        wy -wx    0    wz
        -wx -wy -wz  0];%skew symmetric

    Phi = [expm(omega*dt) zeros(4,3);
           zeros(3,4)     eye(3)     ];
    
    % Project the state ahead
    quaternion_ = Phi*quaternion;
    
    CSI = [[0 -quaternion(3) quaternion(2);
        quaternion(3) 0 -quaternion(1)
        -quaternion(2) quaternion(1) 0]+quaternion(4)*eye(3);...
        -quaternion(1:3)'];
    
    Q = [(dt/2)^2*CSI*SIGMA_g*CSI' zeros(4,3);
        zeros(3,4) SIGMA_a      ];
    
    % Compute the a priori covariance matrix
    P_= Phi*P*Phi'+Q;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%% UPDATE STEP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    q1 = quaternion_(1); q2 = quaternion_(2); q3 = quaternion_(3); q4 = quaternion_(4);
    
    % Linearize the measurement equation: Jacobian
    F = [2*g*[q3 -q4 q1 -q2
        q4 q3 q2 q1
        -q1 -q2 q3 q4],eye(3)];
        
    
    % Adapt the measurement covariance matrix
    C = [q1^2-q2^2-q3^2+q4^2 2*(q1*q2+q3*q4) 2*(q1*q3-q2*q4)
        2*(q1*q2-q3*q4) -q1^2+q2^2-q3^2+q4^2 2*(q2*q3+q1*q4)
        2*(q1*q3+q2*q4) 2*(q2*q3-q1*q4) -q1^2-q2^2+q3^2+q4^2];
    
    % Accelerometer
    if norm(accel - C*[0;0;g] - quaternion_(5:7)) < Param(7) % (Eq 70 in 2011 paper)
        std_acc = Param(5);
    else
        std_acc = Param(6);
    end

    
    % Measurement covariance
    R = std_acc^2*eye(3);
    
    % Compute the Kalman Gain
    K = P_* F' * (F * P_ * F' + R)^-1;
    
    % Compute the a posteriori state estimate
    z = accel'; % measurement vector
    z_predict = C*[0; 0; g] + quaternion_(5:end);
    quaternion = quaternion_ + K*(z - z_predict);
    
    % Normalize the quaternion
    quaternion(1:4) = quaternion(1:4)/norm(quaternion(1:4));
    
    % Compute the a posteriori covariance matrix
    P = P_ - K*F*P_;
    
    rot_quat = quaternion(1:4);
    acc_bias = quaternion(5:7);
    
end


