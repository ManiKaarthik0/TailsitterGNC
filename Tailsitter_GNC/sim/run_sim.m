% RUN_SIM  Open-loop trim validation — 16-state model

addpath('/Personal Projects/Tech/Tailsitter GNC/Tailsitter_GNC/dynamics/', '/Personal Projects/Tech/Tailsitter GNC/Tailsitter_GNC/params/');
run('params.m');

%% Pack params into struct for clean function passing
P = struct('m',m,'g',g,'I',I,'rho',rho,'S',S,'c',c,'b',b,'l',l, ...
    'AR',AR,'CL0',CL0,'CL_alpha',CL_alpha,'CD0',CD0,'k_ind',k_ind, ...
    'CY0',CY0,'CY_beta',CY_beta,'CY_p',CY_p,'CY_r',CY_r, ...
    'Cl0',Cl0,'Cl_beta',Cl_beta,'Cl_p',Cl_p,'Cl_r',Cl_r,'Cl_delta',Cl_delta, ...
    'Cm0',Cm0,'Cm_alpha',Cm_alpha,'Cm_q',Cm_q,'Cm_delta',Cm_delta,'Cm_alphadot',Cm_alphadot, ...
    'Cn0',Cn0,'Cn_beta',Cn_beta,'Cn_r',Cn_r,'k_rxn',k_rxn, ...
    'alpha_stall',alpha_stall,'alpha_bw',alpha_bw,'alpha_post',alpha_post, ...
    'beta_stall',beta_stall,'beta_bw',beta_bw, ...
    'q_pw',q_pw,'T_pw',T_pw,'k_prop',k_prop,'EPS',1e-8);

%% Trim
V0      = 20;
q_bar0  = 0.5 * rho * V0^2;
CL_trim = (m * g) / (q_bar0 * S);
alpha0  = (CL_trim - CL0) / CL_alpha;
theta0  = alpha0;

CD_trim      = CD0 + k_ind * CL_trim^2;
T0           = q_bar0 * S * CD_trim;
delta_e_trim = -(Cm0 + Cm_alpha * alpha0) / Cm_delta;
d_trim       = delta_e_trim;

fprintf('alpha = %.2f deg | T0 = %.3f N | delta_e = %.2f deg\n', ...
        rad2deg(alpha0), T0, rad2deg(delta_e_trim));

X0 = [0; 0; -100;
      0; 0; theta0;
      V0*cos(alpha0); 0; V0*sin(alpha0);
      0; 0; 0;
      T0/2; T0/2;
      d_trim; d_trim];

U0 = zeros(4,1);

%% Simulate
[t, X] = ode45(@(t,x) rigid_body(t, x, U0, P), [0 15], X0);

%% Plot
figure;
subplot(4,1,1); plot(t, rad2deg(X(:,6)));  ylabel('\theta [deg]'); title('Pitch');
subplot(4,1,2); plot(t, X(:,7));           ylabel('vx [m/s]');     title('Forward velocity');
subplot(4,1,3); plot(t, -X(:,3));          ylabel('alt [m]');      title('Altitude');
subplot(4,1,4); plot(t, rad2deg(X(:,5)));  ylabel('\phi [deg]');   title('Roll'); xlabel('t [s]');


V_range = linspace(12, 40, 200);
q_bar   = 0.5 * rho * V_range.^2;

CL_trim    = (m * g) ./ (q_bar * S);
alpha_trim = (CL_trim - CL0) / CL_alpha;   % rad
LW_ratio   = (q_bar * S .* CL_trim) / (m * g);

figure;
subplot(2,1,1);
plot(V_range, rad2deg(alpha_trim), 'b', 'LineWidth', 1.5); hold on;
yline(12, 'r--', 'Stall limit'); 
yline(0,  'k--');
ylabel('\alpha_{trim} [deg]');
xlabel('V [m/s]');
title('Trim alpha vs Airspeed');
grid on;

subplot(2,1,2);
plot(V_range, LW_ratio, 'b', 'LineWidth', 1.5); hold on;
yline(1.0, 'r--', 'L/W = 1');
ylabel('L/W');
xlabel('V [m/s]');
title('Lift-to-Weight ratio at trim');
grid on;

