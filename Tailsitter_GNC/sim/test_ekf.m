% TEST_EKF  Validate EKF on synthetic trajectory before Simulink integration
addpath('/FixedwingGNC/Tailsitter_GNC/dynamics/', '/FixedwingGNC/Tailsitter_GNC/params/');
run('params.m');

dt = 0.01;    % 100 Hz
T  = 10;
N  = T/dt;

%%
dt = 0.01;  T = 15;  tgrid = 0:dt:T;          % uniform 100 Hz grid
% ===== GROUND-TRUTH closed loop (no Simulink) =====
X0p = X_trim;  X0p(5) = X0p(5) + 0.15;  X0p(6) = X0p(6) + 0.10;   % +8.6° roll, +5.7° pitch
[~, Xtrue] = ode45(@(t,X) rigid_body(t,X, ...
                   sas_cmd(X,K_long,K_lat,X_long_trim,X_lat_trim), P), ...
                   tgrid, X0p);                % Xtrue is (N x 16) on the grid

function U = sas_cmd(X, K_long, K_lat, X_long_trim, X_lat_trim)
  de=(X(15)+X(16))/2; da=(X(15)-X(16))/2; dT=(X(13)-X(14))/2;
  de_rate = -K_long * ([X(7);X(9);X(6);X(11);de] - X_long_trim);
  ul      = -K_lat  * ([X(8);X(10);X(12);X(5);da;dT] - X_lat_trim);
  U = [0.5*ul(2); -0.5*ul(2); de_rate+ul(1); de_rate-ul(1)];   % [T1dot T2dot d1dot d2dot]
end

fprintf("Testing EKF")
% --- loop ---
%%

N = numel(tgrid);
truth = zeros(6,N); est = zeros(6,N); sig = zeros(6,N); nees = zeros(1,N);

est       = zeros(3, N);   % attitude estimate [psi;phi;theta]
est_bias  = zeros(3, N);   % bias estimate [bx;by;bz]   <-- the new one

x_est = [0; 0; X_trim(6); 0; 0; 0];          % single state...
% P     = diag([0.1 0.1 0.1 0.01 0.01 0.01]);  % ...and its covariance
% Noise covariances
% Q = diag([1e-4, 1e-4, 1e-4, 1e-5, 1e-5, 1e-5]);   % process
R_gyro  = diag([0.005^2, 0.005^2, 0.005^2]);        % gyro
% R_accel = diag([0.02^2,  0.02^2,  0.02^2]);          % accel
x = [0; 0; theta0; 0; 0; 0];
P = diag([0.1 0.1 0.1, 1e-4 1e-4 1e-4]);
Q = blkdiag(1e-5*eye(3), 1e-7*eye(3));   % [attitude ; bias random-walk]
R_accel = (0.02^2)*eye(3);
for k = 1:N
    Xk = Xtrue(k,:).';
    [zg, za] = imu_simulate(Xk, dt);
    [x, P]        = ekf_att_bias(x, P, zg, za, dt, Q, R_accel);
    est(:,k)      = x(1:3);    % attitude
    est_bias(:,k) = x(4:6);    % bias   <-- log it here
end

%%
b_true = [0.003; -0.002; 0.001];   % the constant bias imu_simulate injects
lab = {'b_x','b_y','b_z'};
figure;
for i = 1:3
    subplot(3,1,i);
    plot(tgrid, est_bias(i,:), 'b'); hold on;
    yline(b_true(i), 'r--');         % the truth it should converge to
    ylabel(lab{i}); grid on;
end
sgtitle('Estimated gyro bias vs truth');

%%
t = tgrid;  lab = {'\psi','\phi','\theta','w_x','w_y','w_z'};

% (1) estimate vs truth
figure;
for i = 1:6
    subplot(3,2,i);
    plot(t, truth(i,:), 'b', t, est(i,:), 'r--');
    ylabel(lab{i}); grid on; if i==1, legend('truth','EKF'); end
end
sgtitle('EKF estimate vs truth');

% (2) error with ±3σ  (the money plot)
figure;
for i = 1:6
    subplot(3,2,i);
    err = truth(i,:) - est(i,:);
    plot(t, err, 'b'); hold on;
    plot(t,  3*sig(i,:), 'r--',  t, -3*sig(i,:), 'r--');
    ylabel(['err ' lab{i}]); grid on;
end
sgtitle('Estimation error with \pm3\sigma bounds');

% (3) NEES (consistency)
figure;
plot(t, nees, 'b'); hold on;
yline(6,     'k-',  'ideal n=6');
yline(14.45, 'r--', '95% upper');   % chi^2(6) bounds
yline(1.24,  'r--', '95% lower');
xlabel('t [s]'); ylabel('NEES'); grid on;
title(sprintf('NEES  (mean = %.2f, ideal = 6)', mean(nees)));

%%
% ===== EKF predict-Jacobian verification =====
dt = 0.01;
x_test = [0.3; 0.2; 0.15; 0.4; -0.2; 0.1];   % nonzero rates AND angles

% (1) numerical Jacobian of the predict step
F_num = numjac(@(x) predict_only(x,dt), x_test, 1e-6);

% (2) analytic Jacobian, evaluated AT x_test  (unpack x_test ITSELF)
psi=x_test(1); phi=x_test(2); theta=x_test(3);
wx =x_test(4); wy =x_test(5); wz =x_test(6);
cth=cos(theta); sth=sin(theta); cph=cos(phi); tph=tan(phi);
inv_cph = 1/sqrt(cph^2 + 1e-6);

F = eye(6);
F(1,2) = dt * ((-wx*sth + wz*cth) * sin(phi) * inv_cph^2);
F(1,3) = dt * ((-wx*cth - wz*sth) * inv_cph);
F(1,4) = dt * (-sth * inv_cph);
F(1,6) = dt * ( cth * inv_cph);
F(2,3) = dt * (-wx*sth + wz*cth);
F(2,4) = dt * cth;
F(2,6) = dt * sth;
F(3,2) = dt * (wx*sth/cph^2 - wz*cth/cph^2);
F(3,3) = 1 + dt * (wx*cth*tph + wz*sth*tph);   % <-- leave the bug; the check should catch it
F(3,4) = dt * (sth*tph);
F(3,5) = dt;
F(3,6) = dt * (-cth*tph);

% (3) compare
fprintf('max mismatch = %.3e\n', max(abs(F_num - F), [], 'all'));
disp(F_num - F);

% ===== functions go LAST in a script file =====
function xn = predict_only(x, dt)
    psi=x(1); phi=x(2); theta=x(3); wx=x(4); wy=x(5); wz=x(6);
    cth=cos(theta); sth=sin(theta); cph=cos(phi); tph=tan(phi);
    inv_cph = 1/sqrt(cph^2 + 1e-6);
    dphi   =  wx*cth + wz*sth;
    dpsi   = (-wx*sth + wz*cth)*inv_cph;
    dtheta =  wy + wx*sth*tph - wz*cth*tph;
    xn = x + dt*[dpsi; dphi; dtheta; 0; 0; 0];
end

function J = numjac(fun, x, h)
    n = numel(x); J = zeros(n);
    for j = 1:n
        xp = x; xp(j)=xp(j)+h;
        xm = x; xm(j)=xm(j)-h;
        J(:,j) = (fun(xp) - fun(xm)) / (2*h);
    end
end

assert(max(abs(F_num - F),[],'all') < 1e-6, 'Jacobian wrong')

%%
mask = tgrid > 1;                      % skip the init transient
fprintf('steady-state NEES = %.2f\n', mean(nees(mask)));