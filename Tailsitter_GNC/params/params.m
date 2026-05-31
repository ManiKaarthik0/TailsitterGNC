%% Mass & inertia
m   = 2.5;
g   = 9.81;
Ixx = 0.0084;
Iyy = 0.0118;
Izz = 0.0199;
Ixz = 0.0;
I   = diag([Ixx, Iyy, Izz]);
I(1,3) = Ixz; I(3,1) = Ixz;

%% Geometry
rho = 1.225;
S   = 0.26;
c   = 0.31;
b   = 1.0;
l   = 0.2;
AR  = b^2 / S;

%% Lift
CL0      =  0.071204;
CL_alpha =  2.746404;

%% Drag
CD0   = 0.0025;       % fitted value ~0, clamped to physical minimum
e     = 0.75;
k_ind = 1.0 / (pi * e * AR);

%% Lateral/directional
CY0 = -2.320e-05; CY_beta = -0.010055; CY_p = -0.002661; CY_r =  0.000919;
Cl0 =  9.024e-04; Cl_beta = -0.022865; Cl_p = -0.011875; Cl_r =  0.009785;
Cn0 = 0; Cn_beta =  0.007163;                   Cn_r = -0.000879;

%% Pitch
Cm0         = -0.036607;
Cm_alpha    = -0.048111;
Cm_q        = -0.010728;
Cm_delta    =  0.316238;
Cm_alphadot = -1.8;

%% Control
Cl_delta = 0.188085;   % fitted (note: sign flip from original -0.5)
k_rxn    = 0.06;

%% Stall
alpha_stall = 12 * pi/180;
alpha_bw    =  2 * pi/180;
alpha_post  = 18 * pi/180;
beta_stall  = 12 * pi/180;
beta_bw     =  2 * pi/180;

%% Propwash
q_pw   = 10.0;
T_pw   =  1.0;
k_prop =  0.6;

%% State/input layout
% X = [x,y,z, psi,phi,theta, vx,vy,vz, wx,wy,wz, T1,T2, d1,d2]
% U = [T1dot, T2dot, d1dot, d2dot]