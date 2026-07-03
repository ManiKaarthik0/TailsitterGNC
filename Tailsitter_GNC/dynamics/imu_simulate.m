function [z_gyro, z_accel, z_mag] = imu_simulate(X_true, dt)
% IMU_SIMULATE  Simulate noisy IMU from true 16-state vector
%
% X_true follows rigid_body.m order:
% X = [x,y,z, psi,phi,theta, vx,vy,vz, wx,wy,wz, T1,T2, d1,d2]
%      1,2,3,  4,  5,  6,    7, 8, 9,  10,11,12, 13,14, 15,16

    g = 9.81;

    % Unpack — matching rigid_body.m indices exactly
    psi   = X_true(4);
    phi   = X_true(5);
    theta = X_true(6);
    wx    = X_true(10);
    wy    = X_true(11);
    wz    = X_true(12);

    %% Gyro noise parameters
    gyro_noise_std = 0.005;
    gyro_bias      = [0.003; -0.002; 0.001];
    
    mag_noise_std = deg2rad(1);
    z_mag = X_true(4) + mag_noise_std*randn;   % heading measurement (true psi + noise)
    
    %% Accel noise parameters
    accel_noise_std = 0.02;

    %% Gyro measurement
    z_gyro = [wx; wy; wz] + gyro_bias + gyro_noise_std * randn(3,1);

    %% Accelerometer — gravity rotated to body frame
    ax =  g * sin(theta);
    ay = -g * sin(phi) * cos(theta);
    az = -g * cos(phi) * cos(theta);

    z_accel = [ax; ay; az] + accel_noise_std * randn(3,1);
end