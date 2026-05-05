function q = quat_from_two_vectors(a, b)
% Quaternion that rotates unit-vector a onto unit-vector b (shortest arc).
% Mirrors PX4 matrix::Quatf(const Vector3 &src, const Vector3 &dst).
%
% Inputs must be 3x1; they are normalized internally for safety.

a = a / max(norm(a), eps);
b = b / max(norm(b), eps);

cos_theta = dot(a, b);

if cos_theta < -1 + 1e-6
    % 180-deg rotation: pick any axis perpendicular to a.
    axis = cross([1; 0; 0], a);
    if norm(axis) < 1e-6
        axis = cross([0; 1; 0], a);
    end
    axis = axis / norm(axis);
    q = [0; axis(1); axis(2); axis(3)];
else
    axis = cross(a, b);
    s = sqrt((1 + cos_theta) * 2);
    invs = 1 / s;
    q = [0.5 * s; axis(1) * invs; axis(2) * invs; axis(3) * invs];
end
q = quat_normalize(q);
end
