function [R, t, rmse] = umeyama_align(X, Y)
%UMEYAMA_ALIGN  Rigid (rotation + translation, NO scale) alignment X -> Y.
%
% Solves for the proper rotation R (3x3, det = +1) and translation t (3x1)
% that minimise sum_k || (R*X(:,k) + t) - Y(:,k) ||^2 in closed form
% (Umeyama 1991, "Least-Squares Estimation of Transformation Parameters
% Between Two Point Patterns", with_scaling = false).
%
% Inputs:
%   X : 3xN source points (columns are points)
%   Y : 3xN target points, paired one-to-one with X (same N, time-synced)
% Outputs:
%   R    : 3x3 proper rotation mapping X into Y's frame
%   t    : 3x1 translation
%   rmse : RMS of the residual norms after alignment (the ATE)
%
% Used to align an OpenVINS trajectory (gravity-aligned global frame from
% dynamic initialisation, with arbitrary yaw + origin) to the MATLAB NED
% reference before computing position error. Pure rotation+translation
% because monocular-inertial VIO is metric (no scale ambiguity); global yaw
% and position are unobservable gauge freedoms, so removing them by rigid
% alignment is the standard ATE method (same as OpenVINS ov_eval
% AlignTrajectory / TUM evaluate_ate).
%
% Verified against a hand-computed transform in
% tests/unit/test_umeyama_align.m.

assert(size(X,1) == 3 && size(Y,1) == 3, 'X and Y must be 3xN');
assert(size(X,2) == size(Y,2),           'X and Y must have equal N');
n = size(X,2);
assert(n >= 3, 'Need at least 3 points for a 3D alignment');

mu_x = mean(X, 2);
mu_y = mean(Y, 2);
Xc = X - mu_x;
Yc = Y - mu_y;

S = (Yc * Xc.') / n;          % 3x3 cross-covariance (target * source')
[U, ~, V] = svd(S);

D = eye(3);
if det(U * V.') < 0
    D(3,3) = -1;              % reflection fix -> guarantee a proper rotation
end
R = U * D * V.';
t = mu_y - R * mu_x;

resid = Y - (R * X + t);
rmse  = sqrt(mean(sum(resid.^2, 1)));
end
