function R = quat_to_dcm(q)
% Convert unit quaternion [w; x; y; z] to a 3x3 direction-cosine matrix.
% Convention: PX4 stores body-to-NED. R rotates a vector expressed in
% body frame into the NED frame:  v_ned = R * v_body.
%
% Equivalent to matrix::Dcm(q) in PX4 (matrix::Dcm constructor from Quat).

w = q(1); x = q(2); y = q(3); z = q(4);

R = [ 1 - 2*(y*y + z*z),   2*(x*y - z*w),     2*(x*z + y*w);
      2*(x*y + z*w),       1 - 2*(x*x + z*z), 2*(y*z - x*w);
      2*(x*z - y*w),       2*(y*z + x*w),     1 - 2*(x*x + y*y) ];
end
