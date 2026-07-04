% TEST_COMPARISON  Truth-fed vs estimate-fed closed loop (estimator-in-the-loop degradation)
addpath('/FixedwingGNC/Tailsitter_GNC/dynamics/', '/FixedwingGNC/Tailsitter_GNC/params/');
run('params.m');
rng(42);                               % reproducible noise; delete for run-to-run variation

%% --- Build P struct, trim, linear model, LQR gains (self-contained) ---
P = struct('m',m,'g',g,'I',I,'rho',rho,'S',S,'c',c,'b',b,'l',l,'AR',AR, ...
    'CL0',CL0,'CL_alpha',CL_alpha,'CD0',CD0,'k_ind',k_ind,'CY0',CY0,'CY_beta',CY_beta, ...
    'CY_p',CY_p,'CY_r',CY_r,'Cl0',Cl0,'Cl_beta',Cl_beta,'Cl_p',Cl_p,'Cl_r',Cl_r, ...
    'Cl_delta',Cl_delta,'Cm0',Cm0,'Cm_alpha',Cm_alpha,'Cm_q',Cm_q,'Cm_delta',Cm_delta, ...
    'Cm_alphadot',Cm_alphadot,'Cn0',Cn0,'Cn_beta',Cn_beta,'Cn_r',Cn_r,'k_rxn',k_rxn, ...
    'alpha_stall',alpha_stall,'alpha_bw',alpha_bw,'alpha_post',alpha_post, ...
    'beta_stall',beta_stall,'beta_bw',beta_bw,'q_pw',q_pw,'T_pw',T_pw,'k_prop',k_prop,'EPS',1e-8);

V0=20; q0=0.5*rho*V0^2; CLt=(m*g)/(q0*S); alpha0=(CLt-CL0)/CL_alpha; theta0=alpha0;
CDt=CD0+k_ind*CLt^2; T0=q0*S*CDt;
d_trim=-(Cm0+Cm_alpha*alpha0)/Cm_delta; da_trim=-Cl0/Cl_delta;
d1t=d_trim+da_trim; d2t=d_trim-da_trim;
X_trim=[0;0;-100;0;0;theta0; V0*cos(alpha0);0;V0*sin(alpha0); 0;0;0; T0/2;T0/2; d1t;d2t];

nx=16; ex=1e-5; A=zeros(nx);
for i=1:nx
    Xp=X_trim; Xp(i)=Xp(i)+ex;  Xm=X_trim; Xm(i)=Xm(i)-ex;
    A(:,i)=(rigid_body([],Xp,zeros(4,1),P)-rigid_body([],Xm,zeros(4,1),P))/(2*ex);
end
lo=[7 9 6 11];  Aa =[A(lo,lo),(A(lo,15)+A(lo,16))/2; zeros(1,4),0];  Ba =[zeros(4,1);1];
K_long=lqr(Aa,Ba,diag([1 1 5 1 0.1]),10);
la=[8 10 12 5]; Aa6=[A(la,la),[(A(la,15)-A(la,16))/2,(A(la,13)-A(la,14))/2]; zeros(2,4),zeros(2,2)]; Ba6=[zeros(4,2);eye(2)];
K_lat=lqr(Aa6,Ba6,diag([1 1 1 5 0.1 0.1]),10*eye(2));
X_long_trim=[V0*cos(alpha0);V0*sin(alpha0);alpha0;0;d_trim];
X_lat_trim =[0;0;0;0;da_trim;0];

%% --- Common setup ---
dt=0.01; T=15; N=round(T/dt); tvec=(1:N)*dt;
X0p=X_trim; X0p(5)=X0p(5)+deg2rad(5); X0p(6)=X0p(6)+deg2rad(5);   % 5 deg roll & pitch

%% --- Run 1: TRUTH-FED ---
X=X0p; TH_t=zeros(1,N); PH_t=zeros(1,N);
for k=1:N
    U=sas_src(X, X(5),X(6), X(10),X(11),X(12), K_lat,K_long,X_long_trim,X_lat_trim);
    X=rk4(X,U,P,dt);  TH_t(k)=X(6); PH_t(k)=X(5);
end

%% --- Run 2: ESTIMATE-FED (EKF in the loop) ---
X=X0p; xe=[0;0;theta0;0;0;0]; Pe=diag([0.1 0.1 0.1 1e-4 1e-4 1e-4]);
Q=blkdiag(1e-5*eye(3),1e-6*eye(3)); R_accel=(0.02^2)*eye(3);
TH_e=zeros(1,N); PH_e=zeros(1,N); sig=zeros(2,N);
for k=1:N
    [zg,za,zm]=imu_simulate(X,dt);
    [xe,Pe]=ekf_att_bias(xe,Pe,zg,za,zm,dt,Q,R_accel);
    om=zg-xe(4:6);                                  % bias-corrected p,q,r
    U=sas_src(X, xe(2),xe(3), om(1),om(2),om(3), K_lat,K_long,X_long_trim,X_lat_trim);
    X=rk4(X,U,P,dt);  TH_e(k)=X(6); PH_e(k)=X(5);
    sig(:,k)=sqrt(diag(Pe(2:3,2:3)));               % phi,theta 1-sigma
end

%% --- Metrics ---
r2d=@(x) x*180/pi;
fprintf('\n=== Estimator-in-the-loop degradation (est-fed vs truth-fed) ===\n');
fprintf('Pitch theta: max|diff|=%.3f deg   RMS=%.3f deg\n', r2d(max(abs(TH_e-TH_t))), r2d(sqrt(mean((TH_e-TH_t).^2))));
fprintf('Roll  phi  : max|diff|=%.3f deg   RMS=%.3f deg\n', r2d(max(abs(PH_e-PH_t))), r2d(sqrt(mean((PH_e-PH_t).^2))));
mask=tvec>2;
fprintf('\n=== EKF attitude 3-sigma (steady state, t>2s) ===\n');
fprintf('phi   3sigma = %.3f deg\n', r2d(3*mean(sig(1,mask))));
fprintf('theta 3sigma = %.3f deg\n', r2d(3*mean(sig(2,mask))));

%% --- Overlay plot ---
figure;
subplot(2,1,1); plot(tvec,r2d(TH_t),'b',tvec,r2d(TH_e),'r--'); ylabel('\theta [deg]'); legend('truth-fed','est-fed'); grid on; title('Pitch');
subplot(2,1,2); plot(tvec,r2d(PH_t),'b',tvec,r2d(PH_e),'r--'); ylabel('\phi [deg]'); xlabel('t [s]'); grid on; title('Roll');
sgtitle('Closed loop: truth-fed vs estimate-fed');

%% ===== local functions (must be at the END of the file) =====
function Xn = rk4(X,U,P,dt)
    k1=rigid_body([],X,U,P); k2=rigid_body([],X+dt/2*k1,U,P);
    k3=rigid_body([],X+dt/2*k2,U,P); k4=rigid_body([],X+dt*k3,U,P);
    Xn=X+dt/6*(k1+2*k2+2*k3+k4);
end
function U = sas_src(X, phi, theta, p, q, r, K_lat, K_long, X_long_trim, X_lat_trim)
    vy=X(8); vx=X(7); vz=X(9); d1=X(15); d2=X(16); T1=X(13); T2=X(14);
    de=(d1+d2)/2; da=(d1-d2)/2; dT=(T1-T2)/2;
    de_rate=-K_long*([vx;vz;theta;q;de]-X_long_trim);
    ul=-K_lat*([vy;p;r;phi;da;dT]-X_lat_trim);
    U=[0.5*ul(2); -0.5*ul(2); de_rate+ul(1); de_rate-ul(1)];
end