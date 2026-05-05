function ez = quat_dcm_z(q)
% Return the third column of the DCM(q), i.e. the body z-axis expressed
% in the parent (NED) frame. Equivalent to PX4 Quatf::dcm_z().
%
% Useful in the Brescianini reduced-attitude formulation.

w = q(1); x = q(2); y = q(3); z = q(4);
ez = [ 2*(x*z + y*w);
       2*(y*z - x*w);
       1 - 2*(x*x + y*y) ];
end
