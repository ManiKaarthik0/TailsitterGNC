function [F_body, M_body] = aero_forces(alpha, beta, V, p, q, r, ...
                                         alpha_dot, delta_e, delta_a, T1, T2, P)
% AERO_FORCES  Forces and moments with stall blend and propwash
% Inputs:
%   alpha, beta       [rad]
%   V                 airspeed [m/s]
%   p, q, r           body angular rates [rad/s]
%   alpha_dot         rate of alpha [rad/s]
%   delta_e           elevator deflection [rad]
%   delta_a           aileron (differential elevon) [rad]
%   T1, T2            thrust states [N]
%   P                 params struct

    %% Propwash dynamic pressure boost
    T_total = T1 + T2;
    q_inf   = 0.5 * P.rho * V^2;
    q_eff   = q_inf + P.k_prop * T_total * P.T_pw + P.q_pw;

    %% Stall blend (sigmoid on alpha)
    % sigma: 0 = attached, 1 = fully stalled
    sigma = 1 / (1 + exp(-( abs(alpha) - P.alpha_stall) / P.alpha_bw));

    % Attached CL, post-stall flat plate CL
    CL_att  = P.CL0 + P.CL_alpha * alpha;
    CL_post = 2 * sign(alpha) * sin(alpha)^2 * cos(alpha);
    CL      = (1 - sigma) * CL_att + sigma * CL_post;

    % Induced + zero-lift drag
    CD = P.CD0 + P.k_ind * CL^2;

    %% Lateral coefficients
    CY = P.CY0 + P.CY_beta*beta + P.CY_p*(P.b/(2*V))*p + P.CY_r*(P.b/(2*V))*r;

    Cl = P.Cl0 + P.Cl_beta*beta + P.Cl_p*(P.b/(2*V))*p + P.Cl_r*(P.b/(2*V))*r ...
       + P.Cl_delta * delta_a;

    Cm = P.Cm0 + P.Cm_alpha*alpha + P.Cm_q*(P.c/(2*V))*q ...
       + P.Cm_delta*delta_e + P.Cm_alphadot*(P.c/(2*V))*alpha_dot;

    Cn = P.Cn0 + P.Cn_beta*beta + P.Cn_r*(P.b/(2*V))*r ...
       + P.k_rxn * (T1 - T2);   % reaction yaw from differential thrust

    %% Forces in body frame (stability -> body rotation)
    Fx = (-CD*cos(alpha) + CL*sin(alpha)) * q_eff * P.S;
    Fz = (-CD*sin(alpha) - CL*cos(alpha)) * q_eff * P.S;
    Fy =  CY * q_eff * P.S;

    F_body = [Fx; Fy; Fz];

    %% Moments
    M_body = q_eff * P.S * [P.b * Cl;
                             P.c * Cm;
                             P.b * Cn];
end