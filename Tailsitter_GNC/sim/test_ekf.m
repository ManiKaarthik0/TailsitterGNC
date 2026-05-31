% TEST_EKF  Validate EKF on synthetic trajectory before Simulink integration
addpath('/Personal Projects/Tech/Tailsitter GNC/Tailsitter_GNC/dynamics/', '/Personal Projects/Tech/Tailsitter GNC/Tailsitter_GNC/params/');
run('params.m');

dt = 0.01;    % 100 Hz
T  = 10;
N  = T/dt;

%% Noise covariances
Q = diag([1e-4, 1e-4, 1e-4, 1e-5, 1e-5, 1e-5]);   % process
R_gyro  = diag([0.005^2, 0.005^2, 0.005^2]);        % gyro
R_accel = diag([0.02^2,  0.02^2,  0.02^2]);          % accel

%% Initial conditions
X_est = zeros(6,1);   % start with zero attitude estimate
P_est = diag([0.1, 0.1, 0.1, 0.01, 0.01, 0.01]);

%% Storage
X_true_log = zeros(6, N);
X_est_log  = zeros(6, N);

%% Synthetic true trajectory — small sinusoidal roll perturbation
for k = 1:N
    t = k * dt;

    % Simulate a gentle roll oscillation
    phi_true   = 0.1 * sin(2*t);
    theta_true = 0.1;
    psi_true   = 0.0;
    wx_true    = 0.2 * cos(2*t);
    wy_true    = 0.0;
    wz_true    = 0.0;

    X_true = zeros(16,1);
    X_true(5)  = phi_true;
    X_true(6)  = theta_true;
    X_true(4)  = psi_true;
    X_true(10) = wx_true;
    X_true(11) = wy_true;
    X_true(12) = wz_true;

    % Simulate IMU
    [z_gyro, z_accel] = imu_simulate(X_true, dt);

    % Run EKF
    [X_est, P_est] = ekf_attitude(X_est, P_est, z_gyro, z_accel, dt, Q, R_gyro, R_accel);

    % Log
    X_true_log(:,k) = [psi_true; phi_true; theta_true; wx_true; wy_true; wz_true];
    X_est_log(:,k)  = X_est;
end

t_vec = (1:N)*dt;

%% Plot
figure;
labels = {'\psi [rad]','\phi [rad]','\theta [rad]','wx [rad/s]','wy [rad/s]','wz [rad/s]'};
for i = 1:6
    subplot(3,2,i);
    plot(t_vec, X_true_log(i,:), 'b', 'DisplayName', 'true');
    hold on;
    plot(t_vec, X_est_log(i,:),  'r--', 'DisplayName', 'EKF estimate');
    ylabel(labels{i}); grid on;
    if i == 1; legend; end
end
sgtitle('EKF attitude estimation vs true states');

%% Error
err = X_true_log - X_est_log;
fprintf('RMS errors:\n');
fprintf('  phi:   %.4f rad (%.2f deg)\n', rms(err(1,:)), rad2deg(rms(err(1,:))));
fprintf('  theta: %.4f rad (%.2f deg)\n', rms(err(2,:)), rad2deg(rms(err(2,:))));
fprintf('  psi:   %.4f rad (%.2f deg)\n', rms(err(3,:)), rad2deg(rms(err(3,:))));
fprintf('  wx:    %.4f rad/s\n', rms(err(4,:)));
fprintf('  wy:    %.4f rad/s\n', rms(err(5,:)));
fprintf('  wz:    %.4f rad/s\n', rms(err(6,:)));