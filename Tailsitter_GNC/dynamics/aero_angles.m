% 2. Alpha, beta from body velocity
function [alpha, beta] = aero_angles(u, v, w)
% Body velocities -> alpha, beta
% beta is saturated to avoid large sideslip extrapolation

    V = sqrt(u^2 + v^2 + w^2);
    if V < 1e-8
        alpha = 0; beta = 0;
        return
    end
    alpha = atan2(w, u);
    beta  = asin(v / V);
end