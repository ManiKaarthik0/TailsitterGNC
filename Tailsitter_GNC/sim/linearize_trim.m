% linearize_trim.m
addpath('/FixedwingGNC/Tailsitter_GNC/dynamics/', '/FixedwingGNC/Tailsitter_GNC/params/');
run('params.m');
% Build P struct (same as run_sim.m)
P = struct('m',m,'g',g,'I',I,'rho',rho,'S',S,'c',c,'b',b,'l',l, ...
    'AR',AR,'CL0',CL0,'CL_alpha',CL_alpha,'CD0',CD0,'k_ind',k_ind, ...
    'CY0',CY0,'CY_beta',CY_beta,'CY_p',CY_p,'CY_r',CY_r, ...
    'Cl0',Cl0,'Cl_beta',Cl_beta,'Cl_p',Cl_p,'Cl_r',Cl_r,'Cl_delta',Cl_delta, ...
    'Cm0',Cm0,'Cm_alpha',Cm_alpha,'Cm_q',Cm_q,'Cm_delta',Cm_delta,'Cm_alphadot',Cm_alphadot, ...
    'Cn0',Cn0,'Cn_beta',Cn_beta,'Cn_r',Cn_r,'k_rxn',k_rxn, ...
    'alpha_stall',alpha_stall,'alpha_bw',alpha_bw,'alpha_post',alpha_post, ...
    'beta_stall',beta_stall,'beta_bw',beta_bw, ...
    'q_pw',q_pw,'T_pw',T_pw,'k_prop',k_prop,'EPS',1e-8);

%% Trim point
V0      = 20;
q_bar0  = 0.5*rho*V0^2;
CL_trim = (m*g)/(q_bar0*S);

alpha0  = (CL_trim - CL0)/CL_alpha;
theta0  = alpha0;
CD_trim = CD0 + k_ind*CL_trim^2;
T0      = q_bar0*S*CD_trim;
d_trim  = -(Cm0 + Cm_alpha*alpha0)/Cm_delta;
da_trim = -Cl0 / Cl_delta;          % NEW: cancels the Cl0 roll moment at trim
d1_trim = d_trim + da_trim;          % asymmetric:  δe + δa
d2_trim = d_trim - da_trim;          % asymmetric:  δe − δa


X_trim = [0;0;-100; 0;0;theta0; V0*cos(alpha0);0;V0*sin(alpha0); ...
          0;0;0; T0/2;T0/2; d1_trim;d2_trim];      % <-- asymmetric


U_trim = zeros(4,1);

X0 = [0; 0; -100;
      0; 0; theta0;
      V0*cos(alpha0); 0; V0*sin(alpha0);
      0; 0; 0;
      T0/2; T0/2;
      d1_trim; d2_trim];                            % <-- asymmetric

U0 = zeros(4,1);


%% Numerical Jacobian (finite differences)
nx = 16; nu = 4;
eps_x = 1e-5;
eps_u = 1e-5;

f0 = rigid_body([], X_trim, U_trim, P);

% A matrix: df/dX
A = zeros(nx, nx);
for i = 1:nx
    Xp = X_trim; Xp(i) = Xp(i) + eps_x;
    Xm = X_trim; Xm(i) = Xm(i) - eps_x;
    A(:,i) = (rigid_body([],Xp,U_trim,P) - rigid_body([],Xm,U_trim,P)) / (2*eps_x);
end

% B matrix: df/dU
B = zeros(nx, nu);
for i = 1:nu
    Up = U_trim; Up(i) = Up(i) + eps_u;
    Um = U_trim; Um(i) = Um(i) - eps_u;
    B(:,i) = (rigid_body([],X_trim,Up,P) - rigid_body([],X_trim,Um,P)) / (2*eps_u);
end

fprintf('A and B matrices computed.\n');
%%
res = rigid_body([], X_trim, U_trim, P);
fprintf('Trim residual (dynamic states 4:16): %.3e\n', norm(res(4:16)));
assert(norm(res(4:16)) < 0.5, 'TRIM NOT AN EQUILIBRIUM — fix trim before linearizing');

%% Eigenvalues
ev = eig(A);

fprintf('\n=== Eigenvalues ===\n');
for i = 1:length(ev)
    if imag(ev(i)) >= 0
        fprintf('  %+.4f %+.4fi', real(ev(i)), imag(ev(i)));
        if real(ev(i)) > 0
            fprintf('  <-- UNSTABLE');
        end
        fprintf('\n');
    end
end

%% Plot
figure;
scatter(real(ev), imag(ev), 80, 'filled');
hold on;
xline(0, 'r--', 'LineWidth', 1.5);   % stability boundary
yline(0, 'k--', 'LineWidth', 0.5);

% Color unstable ones red
unstable = real(ev) > 0;
scatter(real(ev(unstable)),  imag(ev(unstable)),  120, 'r', 'filled', 'DisplayName', 'Unstable');
scatter(real(ev(~unstable)), imag(ev(~unstable)), 80,  'b', 'filled', 'DisplayName', 'Stable');

xlabel('Real part \sigma');
ylabel('Imaginary part j\omega');
title('Eigenvalues at Trim — Open Loop');
legend; grid on;

% Annotate each eigenvalue
for i = 1:length(ev)
    if imag(ev(i)) >= 0
        text(real(ev(i))+0.002, imag(ev(i))+0.05, ...
             sprintf('%.3f+%.3fi', real(ev(i)), imag(ev(i))), ...
             'FontSize', 7);
    end
end

%% ============================================================
%% SAS DESIGN — lateral and longitudinal
%% ============================================================

%% State indices reference
% X = [x,y,z, psi,phi,theta, vx,vy,vz, wx,wy,wz, T1,T2, d1,d2]
%      1,2,3,  4,  5,  6,    7, 8, 9,  10,11,12, 13,14, 15,16

%% --- LATERAL ---
lat_states = [8, 10, 12, 5];   % vy, p, r, phi
A_lat = A(lat_states, lat_states);   % 4x4

% B for delta_a: differential elevon (d1-d2)/2
% column 15 = d1 effect, column 16 = d2 effect on full state
B_da = (A(lat_states,15) - A(lat_states,16))/2;   % ∂(ẋ_lat)/∂δa
B_dT = (A(lat_states,13) - A(lat_states,14))/2;   % ∂(ẋ_lat)/∂dT

A_aug6 = [ A_lat,       [B_da  B_dT] ;
           zeros(2,4),  zeros(2,2)   ];    % δa_dot, dT_dot = inputs -> rows zero  (6x6)
B_aug6 = [ zeros(4,2) ; eye(2) ];          % two rate inputs                        (6x2)

fprintf('\n=== Lateral open loop eigenvalues ===\n');
disp(eig(A_lat))

% Check controllability
Co_lat = ctrb(A_aug6,B_aug6);
fprintf('Lateral controllability rank: %d (need 4)\n', rank(Co_lat));

% Target poles: move unstable pair to LHP, keep stable ones
target_lat = [-1+6.82i; -1-6.82i; -3.099; -0.5;  -20; -22];  % +2 DISTINCT actuator poles
K_lat = place(A_aug6, B_aug6, target_lat);   % now 2x6 over [vy p r φ  δa  dT]
fprintf('\nK_lat (2x4):\n'); disp(K_lat)
fprintf('Row 1 = [K_vy, K_p, K_r, K_phi, K_da]  → delta_a_rate\n')
fprintf('Row 2 = [K_vy, K_p, K_r, K_phi, K_dT]  → dT_rate\n')

% Verify closed loop
ev_lat = eig(A_aug6 - B_aug6*K_lat);   % implemented loop -> must = targets, all LHP
fprintf('\nLateral closed loop eigenvalues:\n'); disp(ev_lat)
fprintf('Lateral stable: %d\n', all(real(ev_lat) < 0));

% Verify trim output = 0
u_lat_trim = K_lat * zeros(6,1);
fprintf('Lateral SAS at trim: da=%.8f  dT=%.8f\n', u_lat_trim(1), u_lat_trim(2));

X_lat_trim = [0;0;0;0; da_trim; 0];
fprintf('X_lat_trim = '); disp(X_lat_trim');

%% Sign self-test (roll-right should command restoring aileron)
x_roll_right = [0; 0; 0; 0.1; 0; 0];      % +0.1 rad roll (phi is 4th in [vy p r phi da dT])
u_lat        = -K_lat * x_roll_right;      % 2x1 = [da_rate; dT_rate]
da_rate      = u_lat(1);
fprintf('Sign test: roll-right 0.1 rad -> da_rate = %.4f\n', da_rate);
%% --- LONGITUDINAL ---
long_states = [7, 9, 6, 11];   % vx, vz, theta, q
A_long = A(long_states, long_states);   % 4x4

% B for collective elevator delta_e = (d1+d2)/2
B_long = (A(long_states, 15) + A(long_states, 16)) / 2;   % 4x1

A_aug = [ A_long,      B_long ;     % rigid states feel δe through this column
          zeros(1,4),  0          ];    % δe_dot = input  -> its own row is zero  (5x5)
B_aug = [ zeros(4,1) ; 1 ];             % the RATE enters ONLY the δe integrator   (5x1)

fprintf('\n=== Longitudinal open loop eigenvalues ===\n');
disp(eig(A_long))

% Check controllability
Co_long = ctrb(A_aug,B_aug);
fprintf('Longitudinal controllability rank: %d (need 4)\n', rank(Co_long));

% Target poles: improve phugoid damping, keep fast modes
target_long = [-1.0+0.5i; -1.0-0.5i; -7.704; -19.511;  -25];  % +1 fast actuator pole
K_long = place(A_aug, B_aug, target_long);     % now 1x5 over [vx vz θ q  δe]

fprintf('\nK_long (1x5): [vx vz theta q delta_e]\n'); disp(K_long)
fprintf('Maps [vx, vz, theta, q] → delta_e_rate\n')

% Verify closed loop
ev_long_cl = eig(A_aug - B_aug*K_long);   % THIS is the implemented loop — must = targets, all LHP

fprintf('\nLongitudinal closed loop eigenvalues:\n'); disp(ev_long_cl)
fprintf('Longitudinal stable: %d\n', all(real(ev_long_cl) < 0));

% Trim state for longitudinal
X_long_trim = [V0*cos(alpha0); V0*sin(alpha0); alpha0; 0;  d_trim];  % 5x1
fprintf('X_long_trim = '); disp(X_long_trim')

% Verify trim output = 0
u_long_trim = K_long * (X_long_trim - X_long_trim);
fprintf('Longitudinal SAS at trim: de=%.8f\n', u_long_trim);

%% --- SUMMARY ---
fprintf('\n=== SAS DESIGN SUMMARY ===\n');
fprintf('K_lat  (2x4):\n'); disp(K_lat)
fprintf('K_long (1x5):\n'); disp(K_long)
fprintf('X_long_trim (4x1):\n'); disp(X_long_trim)
fprintf('Lateral  all stable: %d\n', all(real(ev_lat)  < 0));
fprintf('Longitud all stable: %d\n', all(real(ev_long_cl) < 0));

%% --- POLE PLOT ---
figure(5); clf;
scatter(real(eig(A_lat)),  imag(eig(A_lat)),  120, 'rx', 'LineWidth', 2, 'DisplayName', 'lat open loop');
hold on;
scatter(real(eig(A_long)), imag(eig(A_long)), 120, 'r+', 'LineWidth', 2, 'DisplayName', 'long open loop');
scatter(real(ev_lat),   imag(ev_lat),   100, 'b^', 'filled',       'DisplayName', 'lat closed loop');
scatter(real(ev_long_cl),  imag(ev_long_cl),  100, 'bs', 'filled',       'DisplayName', 'long closed loop');
xline(0,'k--','LineWidth',1); yline(0,'k:');
legend; grid on;
xlabel('Real'); ylabel('Imag');
title('SAS: open vs closed loop poles');

%% Sign self-test (nose-up should command nose-down)
x_nose_up = X_long_trim + [0; 0; 0.1; 0; 0];     % +0.1 rad pitch
de_rate   = -K_long * (x_nose_up - X_long_trim); % control law u = -K*(x - x_trim)
fprintf('Sign test: nose-up 0.1 rad -> de_rate = %.4f (need < 0)\n', de_rate);
assert(de_rate < 0, 'SIGN ERROR: K_long applied with wrong sign');
fprintf('Longitudinal sign: PASS\n');

%%
fprintf('Eigen values of A_aug-B_aug*K_long): ');disp(eig(A_aug - B_aug*K_long)); 

%% Manually compute what SAS outputs at trim
% At trim all lateral states should be near zero
vy_trim  = 0;
p_trim   = 0;
r_trim   = 0;
phi_trim = 0;
del_a_trim = 0;
del_T_trim = 0;


u_sas = K_lat * [vy_trim; p_trim; r_trim; phi_trim; del_a_trim; del_T_trim];
fprintf('SAS output at trim: delta_a=%.6f  dT=%.6f\n', u_sas(1), u_sas(2));

%% Check what B_lat actually looks like
fprintf('B_da (delta_a effect on lateral states):\n');
disp(B_da)
fprintf('B_dT (diff thrust effect on lateral states):\n');
disp(B_dT)

% Check norms
fprintf('||B_da|| = %.4f\n', norm(B_da));
fprintf('||B_dT|| = %.4f\n', norm(B_dT));

%%
vx_t = V0*cos(alpha0);
vz_t = V0*sin(alpha0);
theta_t = alpha0;
q_t = 0;

de_rate = K_long * [vx_t; vz_t; theta_t; q_t; d_trim];
fprintf('delta_e_rate at trim = %.8f\n', de_rate);

%% First compute and save trim values in workspace
X_long_trim = [V0*cos(alpha0); V0*sin(alpha0); alpha0; 0; d_trim];
fprintf('X_long_trim = '); disp(X_long_trim')

%% What does K_long output at t=0?
X_long_0 = [V0*cos(alpha0); V0*sin(alpha0); alpha0; 0];
de_0 = K_long * (X_long_0 - X_long_trim);
fprintf('de_rate at t=0: %.8f\n', de_0);
% Should be exactly 0

% If aircraft pitches up (theta increases), elevator should push nose down
X_test = X_long_trim + [0; 0; 0.1; 0];   % theta increased by 0.1 rad
de_test = K_long * (X_test - X_long_trim);
fprintf('de_rate for nose-up: %.4f\n', de_test);
% Negative = nose down correction = CORRECT
% Positive = nose up amplification = WRONG, negate K_long

%% If aircraft rolls right (phi increases), delta_a should correct left
X_lat_test = [0; 0; 0; 0.1];   % phi increased by 0.1 rad
da_test = K_lat * X_lat_test;
fprintf('da_rate for roll-right: %.4f\n', da_test(1));
% Should be negative to correct back

%% Re-test with the gain in Simulink convention
da_corrected = -1 * K_lat * [0; 0; 0; 0.1];
fprintf('da_rate for roll-right (corrected): %.4f\n', da_corrected(1));
% Must be negative

de_corrected = -1 * K_long * ([0; 0; 0.1; 0]);
fprintf('de_rate for nose-up (corrected): %.4f\n', de_corrected);
% Must be negative

%%
n    = size(out.x_out1, 1);
tout = linspace(0, 15, n)';

figure;
subplot(4,1,1); plot(tout, rad2deg(out.x_out1(:,6)));  ylabel('\theta [deg]'); title('Pitch');
subplot(4,1,2); plot(tout, rad2deg(out.x_out1(:,5)));  ylabel('\phi [deg]');  title('Roll');
subplot(4,1,3); plot(tout, rad2deg(out.x_out1(:,4)));  ylabel('\psi [deg]');  title('Yaw');
subplot(4,1,4); plot(tout, -out.x_out1(:,3));           ylabel('alt [m]');    title('Altitude');
xlabel('t [s]');
sgtitle('Closed Loop — Lateral SAS Active');

figure;
x =  out.x_out1(:,1);
y =  out.x_out1(:,2);
z = -out.x_out1(:,3);
plot3(x, y, z, 'b', 'LineWidth', 1.5); hold on;
plot3(x(1), y(1), z(1), 'go', 'MarkerSize', 10, 'DisplayName', 'Start');
plot3(x(end), y(end), z(end), 'rx', 'MarkerSize', 10, 'DisplayName', 'End');
xlabel('x [m]'); ylabel('y [m]'); zlabel('alt [m]');
title('Trajectory — Closed Loop Lateral SAS');
legend; grid on; axis equal; view(45,30);


%% Results with EKF filter on 

%%
n    = size(out.x_out2, 1);
tout = linspace(0, 15, n)';

figure;
subplot(4,1,1); plot(tout, rad2deg(out.x_out2(:,6)));  ylabel('\theta [deg]'); title('Pitch');
subplot(4,1,2); plot(tout, rad2deg(out.x_out2(:,5)));  ylabel('\phi [deg]');  title('Roll');
subplot(4,1,3); plot(tout, rad2deg(out.x_out2(:,4)));  ylabel('\psi [deg]');  title('Yaw');
subplot(4,1,4); plot(tout, -out.x_out2(:,3));           ylabel('alt [m]');    title('Altitude');
xlabel('t [s]');
sgtitle('Closed Loop — Lateral SAS Active - EKF Estimates');

figure;
x =  out.x_out2(:,1);
y =  out.x_out2(:,2);
z = -out.x_out2(:,3);
plot3(x, y, z, 'b', 'LineWidth', 1.5); hold on;
plot3(x(1), y(1), z(1), 'go', 'MarkerSize', 10, 'DisplayName', 'Start');
plot3(x(end), y(end), z(end), 'rx', 'MarkerSize', 10, 'DisplayName', 'End');
xlabel('x [m]'); ylabel('y [m]'); zlabel('alt [m]');
title('Trajectory — Closed Loop Lateral SAS - with EKF estimates');
legend; grid on; axis equal; view(45,30);