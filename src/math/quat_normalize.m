function q = quat_normalize(q)
% Renormalize a quaternion to unit length. Falls back to identity on zero.
n = norm(q);
if n < eps
    q = [1; 0; 0; 0];
else
    q = q / n;
end
end
