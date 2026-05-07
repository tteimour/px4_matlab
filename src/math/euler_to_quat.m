function q = euler_to_quat(roll, pitch, yaw)
% Convert ZYX-intrinsic Euler angles [roll; pitch; yaw] (rad) to a
% quaternion [w; x; y; z], body-to-NED, Hamilton convention. Inverse of
% quat_to_euler.m.
cy = cos(yaw / 2);   sy = sin(yaw / 2);
cp = cos(pitch / 2); sp = sin(pitch / 2);
cr = cos(roll / 2);  sr = sin(roll / 2);

q = [cr*cp*cy + sr*sp*sy;
     sr*cp*cy - cr*sp*sy;
     cr*sp*cy + sr*cp*sy;
     cr*cp*sy - sr*sp*cy];
end
