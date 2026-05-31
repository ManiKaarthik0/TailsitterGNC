function dX = rigid_body(~, X, U, P)
% 16-STATE rigid body dynamics
%
% X  = [x,y,z, psi,phi,theta, vx,vy,vz, wx,wy,wz, T1,T2, d1,d2]
% U  = [T1dot, T2dot, d1dot, d2dot]
% P  = params struct

    %% Unpack state
    % position
    % x=X(1); y=X(2); z=X(3);  (not needed for derivatives)
    psi   = X(4);  phi   = X(5);  theta = X(6);
    vx    = X(7);  vy    = X(8);  vz    = X(9);
    wx    = X(10); wy    = X(11); wz    = X(12);
    T1    = X(13); T2    = X(14);
    d1    = X(15); d2    = X(16);
    %% Saturate actuator rates
    max_thrust_rate = 10;    % N/s
    max_deflect_rate = 1.0;  % rad/s
    
    U(1) = max(-max_thrust_rate,  min(max_thrust_rate,  U(1)));
    U(2) = max(-max_thrust_rate,  min(max_thrust_rate,  U(2)));
    U(3) = max(-max_deflect_rate, min(max_deflect_rate, U(3)));
    U(4) = max(-max_deflect_rate, min(max_deflect_rate, U(4)));
    %% Actuator mixing
    [delta_e, delta_a] = elevon_mix(d1, d2);

    %% Airspeed and aero angles
    V = sqrt(vx^2 + vy^2 + vz^2 + P.EPS);
    [alpha, beta] = aero_angles(vx, vy, vz);

    % alpha_dot: approximate as (w_dot contribution) / V — use wy as proxy
    % for trim validation set to 0; replace with proper estimate in MPC
    alpha_dot = wy;

    %% Gravity in body frame
    F_grav = P.m * P.g * [-sin(theta);
                            sin(phi)*cos(theta);
                            cos(phi)*cos(theta)];

    %% Thrust in body frame (along body x-axis, symmetric)
    T_total = T1 + T2;
    F_thrust = [T_total; 0; 0];

    %% Aero forces and moments
    [F_aero, M_aero] = aero_forces(alpha, beta, V, wx, wy, wz, ...
                                    alpha_dot, delta_e, delta_a, T1, T2, P);

    %% Total forces and moments
    F_total = F_thrust + F_aero + F_grav;
    M_total = M_aero;   % add thrust moment if offset from CG: + [0; T_total*P.l; 0]

    %% Translational dynamics (body frame, Coriolis)
    dvx = F_total(1)/P.m + wz*vy - wy*vz;
    dvy = F_total(2)/P.m + wx*vz - wz*vx;
    dvz = F_total(3)/P.m + wy*vx - wx*vy;

    %% Rotational dynamics (Euler equations)
    Ixx = P.I(1,1); Iyy = P.I(2,2); Izz = P.I(3,3); Ixz = P.I(1,3);
    Gamma = Ixx*Izz - Ixz^2;

    dwx = (Izz*M_total(1) + Ixz*M_total(3) ...
          - (Izz*(Izz-Iyy)+Ixz^2)*wy*wz ...
          + Ixz*(Ixx-Iyy+Izz)*wx*wy) / Gamma;
    dwy = (M_total(2) - (Ixx-Izz)*wx*wz - Ixz*(wx^2-wz^2)) / Iyy;
    dwz = (Ixx*M_total(3) + Ixz*M_total(1) ...
          + (Ixx*(Ixx-Iyy)+Ixz^2)*wx*wy ...
          - Ixz*(Ixx-Iyy+Izz)*wy*wz) / Gamma;

    %% Euler kinematics  (psi, phi, theta order in state)
    cth = cos(theta); sth = sin(theta);
    tph = tan(phi);
    inv_cph = 1.0 / sqrt(cos(phi)^2 + 1e-6);   % safe 1/cos(phi)
    
    dphi   =  wx*cth + wz*sth;
    dpsi   = (-wx*sth + wz*cth) * inv_cph;
    dtheta =  wy + wx*sth*tph - wz*cth*tph;

    %% World position
    R       = rot_matrix(phi, theta, psi);
    pos_dot = R * [vx; vy; vz];

    %% Actuator dynamics (integrator states)
    dT1 = U(1);
    dT2 = U(2);
    dd1 = U(3);
    dd2 = U(4);

    %% Output: state derivative (16x1)
    dX = [pos_dot;
      dpsi; dphi; dtheta;   % order matches X(4:6) = [psi, phi, theta]
      dvx; dvy; dvz;
      dwx; dwy; dwz;
      dT1; dT2; dd1; dd2];
end