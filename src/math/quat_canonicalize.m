function q = quat_canonicalize(q)
% Force q to its canonical form (positive scalar part), matching
% PX4 matrix::Quaternion::canonicalize() / canonical().
% This selects the shorter of the two equivalent rotations (q and -q).
if q(1) < 0
    q = -q;
end
end
