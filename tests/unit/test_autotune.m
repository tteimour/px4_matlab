function test_autotune()
% Module-level test for McAutotuneAttitudeControl. Runs the full
% identification sequence against a fast synthetic per-axis rate plant
% (no quad/RK4) so it stays cheap enough for the unit suite, and checks
% the state machine sequencing + gain capture rather than exact numbers
% (those are covered against PX4 reference values in test_arx_rls,
% test_system_identification and test_pid_design).

addpaths();
p = px4_params();

%% Structural checks before the sequence starts
at = McAutotuneAttitudeControl(p, struct('apply_mode', 0, 'log_enable', false));
assert(at.state == at.STATE_IDLE, 'FAIL: starts in IDLE');
assert(~at.isDone(), 'FAIL: not done before start');
assert(all(at.injection() == 0), 'FAIL: zero injection while idle');

% A step while idle (start not requested) must stay idle.
inj = at.step(1e-3, [0;0;0], [0;0;0], true, [0;0]);
assert(at.state == at.STATE_IDLE && all(inj == 0), 'FAIL: idle without start request');

%% Full run against a synthetic rate plant
rate_ctl = RateController(p);
imu = ImuICM45686([], [], []);
sig_th   = imu.gyro_nd / sqrt(1 / p.rate_hz.rate);
vib_gain = imu.gyro_vib_gain;
rng(7, 'twister');

% Synthetic plant: angular_accel = b_eff*torque - damp*omega (per axis).
b_eff = [50; 50; 30];
damp  = [0.5; 0.5; 0.4];

dt = 1 / p.rate_hz.rate;
omega = [0; 0; 0];
at.start();

t = 0; t_max = 45;
saw_roll = false; saw_pitch = false; saw_yaw = false;
while t < t_max
    inj    = at.injection();
    torque = rate_ctl.update(omega, inj, [0;0;0], dt, false);   % baseline rate_sp = 0
    alpha  = b_eff .* torque - damp .* omega;
    omega  = omega + alpha * dt;

    omega_meas = omega + sig_th * randn(3,1) + vib_gain * 1.0 * randn(3,1);
    at.step(dt, torque, omega_meas, true, [0;0]);

    saw_roll  = saw_roll  || (at.state == at.STATE_ROLL);
    saw_pitch = saw_pitch || (at.state == at.STATE_PITCH);
    saw_yaw   = saw_yaw   || (at.state == at.STATE_YAW);

    if at.isDone(), break; end
    t = t + dt;
end

assert(at.isDone(), 'FAIL: sequence did not finish within t_max');
assert(saw_roll && saw_pitch && saw_yaw, 'FAIL: did not visit all three axes');

r = at.getResults();
assert(all(r.rate_k > 0),  'FAIL: a rate kc was not captured (axis did not converge)');
assert(all(r.rate_i > 0),  'FAIL: a rate ki was not captured');
assert(all(r.rate_d >= 0), 'FAIL: a rate kd is negative');
% computeGains constrains attitude P to [2, 6.5].
assert(all(r.att_p >= 2 - 1e-6 & r.att_p <= 6.5 + 1e-6), 'FAIL: att_p out of [2,6.5]');
assert(islogical(r.success), 'FAIL: success flag is not logical');

%% getTuned consistency (parallel form vs standard form)
tn = at.getTuned();
assert(abs(tn.MC_ROLLRATE_P - r.rate_k(1)) < 1e-12, 'FAIL: tuned P mismatch');
assert(abs(tn.MC_ROLLRATE_I - r.rate_k(1) * r.rate_i(1)) < 1e-12, 'FAIL: tuned I mismatch');
assert(abs(tn.MC_ROLLRATE_D - r.rate_k(1) * r.rate_d(1)) < 1e-12, 'FAIL: tuned D mismatch');

fprintf('test_autotune: PASS (final state = %s, success = %d)\n', r.state, r.success);
end


function addpaths()
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(fullfile(root, 'src', 'params'));
addpath(fullfile(root, 'src', 'math'));
addpath(fullfile(root, 'src', 'controllers'));
addpath(fullfile(root, 'src', 'autotune'));
addpath(fullfile(root, 'src', 'sensors'));
addpath(fullfile(root, 'src', 'sensors', 'imu'));
end
