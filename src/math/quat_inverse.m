function qi = quat_inverse(q)
% Inverse of a unit quaternion (assumes |q|=1, so inverse = conjugate).
% [w; x; y; z] in, [w; -x; -y; -z] out.
qi = [q(1); -q(2); -q(3); -q(4)];
end
