function test_sensor_imu()
% Unit tests for IMU sensor models — chip rate, latency, gravity output,
% bias / noise statistics. Hand-computed against ImuSensor.measure() in
% src/sensors/imu/ImuSensor.m.

addpaths();
earth = EarthModel(0, 0, 0);
gt    = stationary_state();

%% Case 1: ICM-45686 produces no sample before its first period elapses,
%  one sample at t = period, multiple samples for longer t.
imu = ImuICM45686([], [], earth);
imu.step(0.0, gt);
assert(isempty(imu.latest()), 'No sample before t=0+latency');

% Step well past the latency to release the first sample.
T = 0.05;       % 50 ms — covers latency + plenty of periods at 1 kHz
imu.step(T, gt);
s = imu.latest();
assert(~isempty(s), 'Sample published after latency');
assert(isfield(s, 'gyro_b') && isfield(s, 'accel_b'), 'IMU sample shape');
assert(isfield(s, 'delta_ang') && isfield(s, 'delta_vel'), 'Delta-form fields');

%% Case 2: Stationary level vehicle -> accel ≈ +g in body z (gravity points down,
%  body z down, sf = -g in NED -> R_n2b * -g_ned with q=identity = -g_body.
%  Wait — sf_body = R_n2b*(a_inertial - g_ned). At rest level, a_inertial=0,
%  g_ned=[0;0;9.81]. So sf_body = -[0;0;9.81] = [0;0;-9.81]. Body z down.
imu.step(2.0, gt);
s = imu.latest();
assert(abs(s.accel_b(3) - (-9.80665)) < 0.5, 'Accel z reads -g at rest level');
assert(norm(s.accel_b(1:2)) < 0.5, 'Accel x,y near zero at rest level');

%% Case 3: Pure body roll rate (1 rad/s) shows up on gyro x.
gt2 = gt;
gt2.angular_vel_b = [1.0; 0; 0];
imu = ImuICM45686([], [], earth);   % fresh
for t = 0:1e-3:0.5, imu.step(t, gt2); end
s = imu.latest();
assert(abs(s.gyro_b(1) - 1.0) < 0.05, 'Gyro x tracks roll rate');
assert(norm(s.gyro_b(2:3)) < 0.05, 'Gyro y,z near zero in pure roll');

%% Case 4: ADIS-16470 noise is ~10x lower than ICM-45686 (datasheet).
icm  = ImuICM45686([], [], earth);
adis = ImuADIS16470([], [], earth);
icm_gyro = []; adis_gyro = [];
for t = 0:1e-3:1.0
    icm.step(t, gt);  adis.step(t, gt);
    if icm.newSampleAvailable(),  icm_gyro(end+1, :)  = icm.latest().gyro_b';  end %#ok<AGROW>
    if adis.newSampleAvailable(), adis_gyro(end+1, :) = adis.latest().gyro_b'; end %#ok<AGROW>
end
% After bias-removal (subtract mean), residual std should be much smaller for ADIS.
icm_std  = std(icm_gyro  - mean(icm_gyro));
adis_std = std(adis_gyro - mean(adis_gyro));
assert(all(adis_std < icm_std), ...
       sprintf('ADIS gyro noise (%s) < ICM (%s)', mat2str(adis_std,3), mat2str(icm_std,3)));

%% Case 5: Sample rate matches chip nominal (1 kHz effective).
n_expected = round(1.0 * 1000);   % 1 second × 1 kHz
n_got = size(icm_gyro, 1);
assert(abs(n_got - n_expected) < 5, sprintf('IMU samples ~1 kHz: %d vs %d', n_got, n_expected));

fprintf('test_sensor_imu: PASS\n');
end


function gt = stationary_state()
gt.position_ned    = [0; 0; -10];   % 10 m up
gt.velocity_ned    = [0; 0; 0];
gt.attitude_q      = [1; 0; 0; 0];   % level
gt.angular_vel_b   = [0; 0; 0];
gt.acceleration_ned= [0; 0; 0];
end


function addpaths()
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(fullfile(root, 'src', 'math'));
addpath(fullfile(root, 'src', 'sensors'));
addpath(fullfile(root, 'src', 'sensors', 'imu'));
end
