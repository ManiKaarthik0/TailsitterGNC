function [X_est, P_est] = ekf_attitude(X_est, P_est, z_gyro, z_accel, dt, Q, R_gyro, R_accel)
% 6-STATE ATTITUDE EKF
%
% X_est  = [psi, phi, theta, wx, wy, wz]   matches rigid_body.m order
% P_est  = 6x6 covariance matrix
% z_gyro = [wx, wy, wz] gyro measurement
% z_accel= [ax, ay, az] accelerometer measurement

    psi   = X_est(1);
    phi   = X_est(2);
    theta = X_est(3);
    wx    = X_est(4);
    wy    = X_est(5);
    wz    = X_est(6);

    g = 9.81;

    %% ── PREDICT ──────────────────────────────────────────
    % ZXY kinematics — same as rigid_body.m
    cth = cos(theta); sth = sin(theta);
    cph = cos(phi);   tph = tan(phi);
    inv_cph = 1.0 / sqrt(cph^2 + 1e-6);

    dphi   =  wx*cth + wz*sth;
    dpsi   = (-wx*sth + wz*cth) * inv_cph;
    dtheta =  wy + wx*sth*tph - wz*cth*tph;

    % Propagate state — order [psi, phi, theta, wx, wy, wz]
    X_pred = X_est + dt * [dpsi; dphi; dtheta; 0; 0; 0];

    %% Jacobian F = ∂f/∂X   rows: [psi, phi, theta, wx, wy, wz]
    F = eye(6);

    % Row 1: dpsi/d(...)
    F(1,2) = dt * ((-wx*sth + wz*cth) * sin(phi) * inv_cph^2);  % dpsi/dphi
    F(1,3) = dt * ((-wx*cth - wz*sth) * inv_cph);                % dpsi/dtheta
    F(1,4) = dt * (-sth * inv_cph);                               % dpsi/dwx
    F(1,6) = dt * ( cth * inv_cph);                               % dpsi/dwz

    % Row 2: dphi/d(...)
    F(2,3) = dt * (-wx*sth + wz*cth);   % dphi/dtheta
    F(2,4) = dt * cth;                   % dphi/dwx
    F(2,6) = dt * sth;                   % dphi/dwz

    % Row 3: dtheta/d(...)
    F(3,2) = dt * (wx*sth*(1/cph^2) - wz*cth*(1/cph^2));  % dtheta/dphi — fixed
    F(3,3) = 1 + dt * (wx*cth*tph + wz*sth*tph);               % dtheta/dtheta
    F(3,4) = dt * (sth*tph);                                % dtheta/dwx
    F(3,5) = dt * 1;                                        % dtheta/dwy
    F(3,6) = dt * (-cth*tph);                               % dtheta/dwz

    % Rows 4,5,6: wx,wy,wz — constant model, F already eye(6)

    % Propagate covariance
    P_pred = F * P_est * F' + Q;

    %% ── UPDATE: GYROSCOPE ────────────────────────────────
    H_g = [zeros(3,3), eye(3)];
    z_pred_g = X_pred(4:6);
    y_g = z_gyro - z_pred_g;
    
    S_g = H_g * P_pred * H_g' + R_gyro;
    K_g = P_pred * H_g' * pinv(S_g);   % ← use pinv
    
    X_pred = X_pred + K_g * y_g;
    P_pred = (eye(6) - K_g * H_g) * P_pred;
    
    % Force P symmetric and positive definite
    P_pred = (P_pred + P_pred') / 2;
    P_pred = P_pred + 1e-9 * eye(6);   % ← add small floor
    
    %% ── UPDATE: ACCELEROMETER ────────────────────────────
    phi2   = X_pred(2);
    theta2 = X_pred(3);
    
    ax_pred =  g * sin(theta2);
    ay_pred = -g * sin(phi2) * cos(theta2);
    az_pred = -g * cos(phi2) * cos(theta2);
    z_pred_a = [ax_pred; ay_pred; az_pred];
    
    H_a = zeros(3,6);
    H_a(1,2) = 0;
    H_a(2,2) = -g * cos(phi2) * cos(theta2);
    H_a(3,2) =  g * sin(phi2) * cos(theta2);
    H_a(1,3) =  g * cos(theta2);
    H_a(2,3) =  g * sin(phi2) * sin(theta2);
    H_a(3,3) =  g * cos(phi2) * sin(theta2);
    
    y_a = z_accel - z_pred_a;
    
    S_a = H_a * P_pred * H_a' + R_accel;
    K_a = P_pred * H_a' * pinv(S_a);   % ← use pinv
    
    X_est = X_pred + K_a * y_a;
    P_est = (eye(6) - K_a * H_a) * P_pred;
    
    % Force P symmetric and positive definite
    P_est = (P_est + P_est') / 2;
    P_est = P_est + 1e-9 * eye(6);     % ← add small floor
end