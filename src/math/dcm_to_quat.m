function q = dcm_to_quat(R)
% Convert a 3x3 rotation matrix to a unit quaternion [w; x; y; z].
% Uses Shepperd's method for numerical stability across all R.
% Matches PX4 matrix::Quatf(matrix::Dcmf) constructor behavior.

tr = R(1,1) + R(2,2) + R(3,3);

if tr > 0
    s = sqrt(tr + 1.0) * 2;       % s = 4 * w
    w = 0.25 * s;
    x = (R(3,2) - R(2,3)) / s;
    y = (R(1,3) - R(3,1)) / s;
    z = (R(2,1) - R(1,2)) / s;
elseif (R(1,1) > R(2,2)) && (R(1,1) > R(3,3))
    s = sqrt(1.0 + R(1,1) - R(2,2) - R(3,3)) * 2;  % s = 4 * x
    w = (R(3,2) - R(2,3)) / s;
    x = 0.25 * s;
    y = (R(1,2) + R(2,1)) / s;
    z = (R(1,3) + R(3,1)) / s;
elseif R(2,2) > R(3,3)
    s = sqrt(1.0 + R(2,2) - R(1,1) - R(3,3)) * 2;  % s = 4 * y
    w = (R(1,3) - R(3,1)) / s;
    x = (R(1,2) + R(2,1)) / s;
    y = 0.25 * s;
    z = (R(2,3) + R(3,2)) / s;
else
    s = sqrt(1.0 + R(3,3) - R(1,1) - R(2,2)) * 2;  % s = 4 * z
    w = (R(2,1) - R(1,2)) / s;
    x = (R(1,3) + R(3,1)) / s;
    y = (R(2,3) + R(3,2)) / s;
    z = 0.25 * s;
end
q = [w; x; y; z];
q = quat_normalize(q);
end
