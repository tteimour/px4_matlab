function test_ekf2_basics()
% EKF2 sanity checks — predict step, fusion behaviours.
% Hand-computed against src/modules/ekf2/EKF/ekf.cpp predictState() and
% the aid_sources/ implementations.

addpaths();
earth  = EarthModel(0, 0, 0);
params = Ekf2Params();
ekf    = Ekf2(params, earth);

%% Case 1: Zero-input predict over one EKF window preserves state.
% The raw 1 kHz IMU stream is downsampled into EKF2_PREDICT_US (10 ms)
% windows (imu_down_sampler.cpp); predict() returns true only when a
% window completes and the EKF actually stepped.
n_win = round(params.predict_us * 1e-6 / 0.001);   % samples per window
for k = 1:n_win
    imu = make_imu_sample(0.001, [0;0;0], [0;0;-9.80665*0.001]);
    imu.t = k * 0.001;
    upd = ekf.predict(imu);
    if k < n_win
        assert(~upd, 'No EKF update before the downsample window completes');
    else
        assert(upd, 'EKF update fires when the window completes');
    end
end
assert(norm(ekf.pos) < 1e-6, 'Position drift zero with gravity-balancing accel');
assert(norm(ekf.vel) < 1e-6, 'Velocity drift zero with gravity-balancing accel');

%% Case 2: Constant +x specific force drives velocity in body+x (=NED+x at q=identity).
ekf.reset();
dt = 0.001;
% Specific force = +1 m/s^2 in body x; in NED that's +1 (still no rotation).
% Plus we must include the gravity offset since predict adds g.
% Per sample: sf_body = [1; 0; -9.80665] -> dvel = [1e-3; 0; -9.80665e-3];
% over the full 10-sample window: vel += R*dvel_acc + g*dt_win = [0.01; 0; 0]
for k = 1:n_win
    imu = make_imu_sample(dt, [0;0;0], [1*dt; 0; -9.80665*dt]);
    imu.t = k * dt;
    ekf.predict(imu);
end
assert(abs(ekf.vel(1) - n_win*1e-3) < 1e-9, 'Forward accel integrates to +x velocity');
assert(abs(ekf.vel(3)) < 1e-9, 'No vertical velocity with gravity-balanced accel');

%% Case 3: Baro fusion pulls position-z toward measurement.
ekf.reset();
% Drift the position then feed a baro sample.
ekf.pos = [0; 0; -5];                % NED z = -5 (5 m up)
baro.t  = 0.0;
baro.altitude_m  = 8.0;              % 8 m up — baro suggests we're higher
baro.pressure_pa = 0;
baro.temperature_c = 25;
out = ekf.fuseBaro(baro);
assert(out.fused, 'Baro fusion ran');
% After update, pos_z should move toward -8 (i.e. become more negative).
assert(ekf.pos(3) < -5, sprintf('Baro pulls pos_z up: %g -> %g', -5, ekf.pos(3)));

%% Case 4: GNSS fusion pulls position toward measurement after origin set.
ekf.reset();
gps.t = 0; gps.lat_deg = 0; gps.lon_deg = 0; gps.alt_m = 0;
gps.pos_ned = [0; 0; 0]; gps.vel_ned = [0;0;0];
gps.eph = 1; gps.epv = 1; gps.fix_type = 3; gps.satellites_used = 12;
gps.s_variance_mps2 = 0.01;
ekf.fuseGnssPos(gps);   % first call only sets origin
ekf.pos = [1.0; 0; 0];  % small drift inside the 5-sigma gate
ekf.fuseGnssPos(gps);
assert(ekf.pos(1) < 1.0 && ekf.pos(1) > 0, ...
    sprintf('GNSS pulls pos_x toward origin: got %g', ekf.pos(1)));

%% Case 5: Covariance stays positive-definite after random predicts + fusions.
ekf.reset();
rng_old = rng(42);
for k = 1:50
    imu = make_imu_sample(0.001, 0.01*randn(3,1), 1e-3*randn(3,1) + [0;0;-9.80665*0.001]);
    imu.t = k*0.001;
    ekf.predict(imu);
end
rng(rng_old);
P_min_eig = min(eig(ekf.P));
assert(P_min_eig > -1e-9, sprintf('Covariance stays PSD (min eig = %g)', P_min_eig));

fprintf('test_ekf2_basics: PASS\n');
end


function s = make_imu_sample(dt, delta_ang, delta_vel)
s.t = 0; s.instance = 0; s.device_id = uint32(1);
s.gyro_b  = delta_ang / dt;
s.accel_b = delta_vel / dt;
s.delta_ang    = delta_ang;
s.delta_vel    = delta_vel;
s.delta_ang_dt = dt;
s.delta_vel_dt = dt;
end


function addpaths()
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(fullfile(root, 'src', 'math'));
addpath(fullfile(root, 'src', 'sensors'));
addpath(fullfile(root, 'src', 'estimator'));
end
