function rpy = quat_to_euler(q)
% Convert quaternion [w; x; y; z] to ZYX-intrinsic Euler angles
% [roll; pitch; yaw], matching PX4 matrix::Eulerf(matrix::Quatf).
% Returns a 3x1 column vector of radians.

w = q(1); x = q(2); y = q(3); z = q(4);

% Roll (about body x)
sinr_cosp = 2 * (w*x + y*z);
cosr_cosp = 1 - 2 * (x*x + y*y);
roll = atan2(sinr_cosp, cosr_cosp);

% Pitch (about body y) with clamp at gimbal lock
sinp = 2 * (w*y - z*x);
sinp = max(-1, min(1, sinp));
pitch = asin(sinp);

% Yaw (about body z)
siny_cosp = 2 * (w*z + x*y);
cosy_cosp = 1 - 2 * (y*y + z*z);
yaw = atan2(siny_cosp, cosy_cosp);

rpy = [roll; pitch; yaw];
end
