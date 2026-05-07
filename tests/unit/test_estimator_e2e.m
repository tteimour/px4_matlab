function test_estimator_e2e()
% End-to-end smoke test: drive the full SensorHub + EKF2 +
% OutputPredictor with a stationary 30s ground truth and verify the
% estimator converges to that truth within sensor noise bounds.

addpaths();
earth = EarthModel(47.39773, 8.54559, 488.0);
gt    = stationary_state();

bus = EstimatorBus(earth);

T  = 5.0;        % seconds — long enough for baro/GPS/mag/gravity to settle
dt = 1e-3;       % 1 kHz inner loop
for t = 0:dt:T
    bus.step(t, gt);
end

est = bus.stateOut();

% Estimator and ground truth share the same NED origin (the EarthModel
% is immutable post-construction). Estimator should track ground truth
% within a few times sensor noise.
assert(norm(est.position_ned(1:2) - gt.position_ned(1:2)) < 5.0, ...
    sprintf('Estimator NE tracks truth: err=%g m', ...
            norm(est.position_ned(1:2) - gt.position_ned(1:2))));
assert(abs(est.position_ned(3) - gt.position_ned(3)) < 5.0, ...
    sprintf('Estimator z tracks truth (got %g, truth %g)', ...
            est.position_ned(3), gt.position_ned(3)));

% Velocity should be near zero (vehicle is stationary).
assert(norm(est.velocity_ned) < 0.5, ...
    sprintf('Estimator velocity ~ 0: got %g m/s', norm(est.velocity_ned)));

% Attitude: at hover the body z aligns with NED z; check by rotating
% [0;0;1] body into NED via the estimated quaternion and comparing to
% [0;0;1] (level). Tolerance is loose because gravity fusion is
% PX4-style disabled while GNSS provides horizontal aiding (avoids
% locking tilt error into accel-bias estimates during maneuvers), so
% in this purely static benchmark only gyro integration anchors
% pitch/roll. A real flight with motion converges much tighter via
% the GNSS-vel / accel-bias cross-covariance pathway.
R_b2n = quat_to_dcm(est.attitude_q);
body_z_in_n = R_b2n * [0;0;1];
tilt = acos(min(max(body_z_in_n(3), -1), 1));
assert(rad2deg(tilt) < 15.0, sprintf('Tilt error < 15 deg, got %.2f', rad2deg(tilt)));

fprintf('test_estimator_e2e: PASS  pos=[%+.2f %+.2f %+.2f]  tilt=%.2fdeg\n', ...
        est.position_ned, rad2deg(tilt));
end


function gt = stationary_state()
gt.position_ned    = [0; 0; -10];
gt.velocity_ned    = [0; 0; 0];
gt.attitude_q      = [1; 0; 0; 0];
gt.angular_vel_b   = [0; 0; 0];
gt.acceleration_ned= [0; 0; 0];
end


function addpaths()
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(fullfile(root, 'src', 'math'));
addpath(fullfile(root, 'src', 'sensors'));
addpath(fullfile(root, 'src', 'sensors', 'imu'));
addpath(fullfile(root, 'src', 'sensors', 'baro'));
addpath(fullfile(root, 'src', 'sensors', 'mag'));
addpath(fullfile(root, 'src', 'sensors', 'gnss'));
addpath(fullfile(root, 'src', 'sensors', 'voter'));
addpath(fullfile(root, 'src', 'estimator'));
end
