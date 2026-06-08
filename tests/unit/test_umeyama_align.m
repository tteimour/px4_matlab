function test_umeyama_align()
%TEST_UMEYAMA_ALIGN  Recover a known rigid transform from paired points.
%
% Hand-computed case: take a non-degenerate 3D point set, apply a known
% proper rotation (180 deg about X composed with 90 deg about Z -- mimics the
% ENU<->NED Z flip plus an arbitrary VIO init yaw) and translation, then check
% umeyama_align recovers exactly that R, t with ~zero residual.

addpath(fullfile(fileparts(mfilename('fullpath')), '..', '..', 'src', 'math'));

X = [ 0  1  2  0 -1  3  1 ;
      0  0  1  2  1 -1  2 ;
      0  1  0  1  2  0 -1 ];      % 3x7, not collinear/coplanar

th = pi/2;
Rz = [cos(th) -sin(th) 0; sin(th) cos(th) 0; 0 0 1];   % 90 deg yaw, det +1
Rx = [1 0 0; 0 -1 0; 0 0 -1];                          % 180 deg about X, det +1
R_true = Rx * Rz;                                       % proper rotation
t_true = [5; -3; 2];

Y = R_true * X + t_true;

[R, t, rmse] = umeyama_align(X, Y);

assert(abs(det(R) - 1) < 1e-12, 'R is not a proper rotation');
assert(max(abs(R(:) - R_true(:))) < 1e-9, 'R not recovered');
assert(max(abs(t - t_true))       < 1e-9, 't not recovered');
assert(rmse < 1e-9, 'residual should vanish for an exact transform');
assert(max(abs(reshape(R*X + t - Y, [], 1))) < 1e-9, 'reprojection mismatch');

% Noise case: alignment RMSE should be on the order of the injected noise,
% not the trajectory scale (sanity, not an exact value).
Yn = Y + 0.01 * [ 1 -1 0 1 -1 0 1; 0 1 -1 0 1 -1 0; -1 0 1 -1 0 1 -1];
[~, ~, rmse_n] = umeyama_align(X, Yn);
assert(rmse_n < 0.05, 'noisy-case RMSE unexpectedly large');

fprintf('test_umeyama_align passed (exact rmse=%.2e, noisy rmse=%.3f)\n', ...
        rmse, rmse_n);
end
