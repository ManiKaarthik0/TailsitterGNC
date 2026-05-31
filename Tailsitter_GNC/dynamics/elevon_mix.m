function [delta_e, delta_a] = elevon_mix(d1, d2)
% ELEVON_MIX  Elevon mixer
%   delta_e = (d1+d2)/2  -> pitch (collective)
%   delta_a = (d1-d2)/2  -> roll  (differential)
    delta_e = (d1 + d2) / 2;
    delta_a = (d1 - d2) / 2;
end
