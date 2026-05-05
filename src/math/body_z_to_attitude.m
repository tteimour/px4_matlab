function q_sp = body_z_to_attitude(body_z, yaw_sp)
% Build an attitude setpoint quaternion from a desired body-z direction
% (expressed in NED) and a desired yaw angle.
%
% Direct port of ControlMath::bodyzToAttitude() from
%   src/modules/mc_pos_control/PositionControl/ControlMath.cpp:71-114
%
% Inputs:
%   body_z : 3x1 desired body z-axis in NED (typically -thrust_sp / |thrust_sp|)
%   yaw_sp : scalar desired yaw (rad)
%
% Output: q_sp = [w; x; y; z], body-to-NED.

if dot(body_z, body_z) < eps
    body_z = [0; 0; 1];   % safe fallback (level)
end
body_z = body_z / norm(body_z);

% Yaw "compass" axis: rotate desired heading by +pi/2 in the XY plane.
y_C = [-sin(yaw_sp); cos(yaw_sp); 0];

% Desired body x-axis: orthogonal to body_z in the y_C plane.
body_x = cross(y_C, body_z);

% Keep nose forward when inverted (PX4 ControlMath.cpp:88-90).
if body_z(3) < 0
    body_x = -body_x;
end

% Degenerate case: thrust vector lies in the world XY plane.
if abs(body_z(3)) < 1e-6
    body_x = [0; 0; 1];
end

bx_norm = norm(body_x);
if bx_norm > 0
    body_x = body_x / bx_norm;
end

body_y = cross(body_z, body_x);

R_sp = [body_x, body_y, body_z];   % columns are body axes in NED
q_sp = dcm_to_quat(R_sp);
end
