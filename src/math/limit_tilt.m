function body_unit = limit_tilt(body_unit, world_unit, max_angle)
% Tilt-limit a body unit vector relative to a world reference unit vector.
% Direct port of ControlMath::limitTilt() from
%   src/modules/mc_pos_control/PositionControl/ControlMath.cpp:55-69
%
% body_unit:  3x1 desired body z-axis (in world frame), unit length
% world_unit: 3x1 world reference axis (typically [0;0;1] in NED)
% max_angle:  scalar tilt cap in radians
%
% Returns the (possibly-clamped) body_unit, still unit length.

dot_unit = dot(body_unit, world_unit);
% Clamp to acos domain to avoid NaN from floating-point drift.
dot_unit = max(min(dot_unit, 1.0), -1.0);
angle = acos(dot_unit);
angle = min(angle, max_angle);

rejection = body_unit - dot_unit * world_unit;

if dot(rejection, rejection) < eps
    % Vectors are parallel: pick an arbitrary in-plane direction.
    rejection = [1; 0; 0];
end

rej_norm = norm(rejection);
if rej_norm > 0
    rejection = rejection / rej_norm;
end

body_unit = cos(angle) * world_unit + sin(angle) * rejection;
end
