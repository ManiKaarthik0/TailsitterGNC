function [x, P] = ekf_att_bias(x, P, z_gyro, z_accel, z_mag, dt, Q, R_accel)
% 6-STATE ATTITUDE + GYRO-BIAS EKF (gyro as INPUT)
%   x = [psi; phi; theta; bx; by; bz]
    g = 9.81;

    % --- unpack (do NOT re-initialize here) ---
    phi = x(2); theta = x(3);
    b   = x(4:6);

    %% ── PREDICT (gyro drives the prediction) ─────────────
    omega = z_gyro - b;                 % bias-corrected rate
    wx = omega(1); wy = omega(2); wz = omega(3);
    cth=cos(theta); sth=sin(theta); cph=cos(phi); tph=tan(phi);
    inv_cph = 1/sqrt(cph^2 + 1e-6);

    dpsi   = (-wx*sth + wz*cth)*inv_cph;
    dphi   =  wx*cth + wz*sth;
    dtheta =  wy + wx*sth*tph - wz*cth*tph;

    att_pred = x(1:3) + dt*[dpsi; dphi; dtheta];
    x_pred   = [att_pred; b];           % bias: random-walk -> constant prediction

    %% ── Jacobian F = [A_att, -B_att; 0, I] ───────────────
    F = eye(6);
    % A_att  (attitude vs attitude) — same partials as baseline, with the F(3,3) fix
    F(1,2) = dt*((-wx*sth + wz*cth)*sin(phi)*inv_cph^2);
    F(1,3) = dt*((-wx*cth - wz*sth)*inv_cph);
    F(2,3) = dt*(-wx*sth + wz*cth);
    F(3,2) = dt*(wx*sth/cph^2 - wz*cth/cph^2);
    F(3,3) = 1 + dt*(wx*cth*tph + wz*sth*tph);
    % B_att = d(att)/d(omega)  -> bias block is -B_att because omega = z_gyro - b
    B_att = [ dt*(-sth*inv_cph),  0,    dt*(cth*inv_cph);
              dt*cth,             0,    dt*sth;
              dt*(sth*tph),       dt,   dt*(-cth*tph) ];
    F(1:3,4:6) = -B_att;
    % F(4:6,:) already [0 0 0 | I] from eye(6)  (bias random walk)

    P_pred = F*P*F' + Q;

    %% ── UPDATE: ACCELEROMETER ONLY (no gyro update) ──────
    phi2 = x_pred(2); theta2 = x_pred(3);
    z_pred_a = [ g*sin(theta2);
                -g*sin(phi2)*cos(theta2);
                -g*cos(phi2)*cos(theta2) ];

    H_a = zeros(3,6);                   % cols 4:6 (bias) stay ZERO
    H_a(2,2) = -g*cos(phi2)*cos(theta2);
    H_a(3,2) =  g*sin(phi2)*cos(theta2);
    H_a(1,3) =  g*cos(theta2);
    H_a(2,3) =  g*sin(phi2)*sin(theta2);
    H_a(3,3) =  g*cos(phi2)*sin(theta2);

    y_a = z_accel - z_pred_a;
    S_a = H_a*P_pred*H_a' + R_accel;
    K_a = P_pred*H_a'/S_a;              % gain has nonzero BIAS rows -> bias gets corrected

    x = x_pred + K_a*y_a;
    P = (eye(6) - K_a*H_a)*P_pred;
    P = (P + P')/2;                     % keep symmetric
    %% ── UPDATE: MAGNETOMETER (heading) ───────────────────
    
    H_m = zeros(1,6);  H_m(1) = 1;
    R_m = (deg2rad(1))^2;
    y_m = z_mag - x(1);          % <-- z_mag, the argument
    S_m = H_m*P*H_m' + R_m;
    K_m = P*H_m'/S_m;                       % 6x1 gain — note its bz row is nonzero!

    x   = x + K_m*y_m;
    P   = (eye(6) - K_m*H_m)*P;
    P   = (P + P')/2;
end