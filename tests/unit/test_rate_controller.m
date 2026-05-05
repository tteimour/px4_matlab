function test_rate_controller()
% Unit tests for RateController.
% Hand-computed cases against rate_control.cpp:71-118.

addpaths();
p = px4_params();

%% Case 1: zero error, zero FF -> zero torque (integrator was zero).
rc = RateController(p);
torque = rc.update([0;0;0], [0;0;0], [0;0;0], 1e-3, false);
assert_near(torque, [0;0;0], 1e-12, 'Zero error -> zero torque');

%% Case 2: pure proportional output for small error, no integrator buildup
%  in a single tick (because integrator is added AFTER torque is computed
%  in PX4 — see rate_control.cpp:78 vs 81-82).
rc = RateController(p);
err = [0.1; 0; 0];           % 0.1 rad/s roll error
expected = p.rate.gain_p .* err;   % MC_ROLLRATE_P * MC_ROLLRATE_K * err
torque = rc.update([0;0;0], err, [0;0;0], 1e-3, false);
assert_near(torque, expected, 1e-12, 'P-only torque on first tick');

%% Case 3: integrator accumulates over multiple ticks with i_factor=1
%  (small error so quadratic term ~ 1).
rc = RateController(p);
err = [0.01; 0; 0];          % small error -> i_factor ≈ 1
dt = 1e-3;
n  = 100;
for i = 1:n
    rc.update([0;0;0], err, [0;0;0], dt, false);
end
% Predicted integrator after n steps (i_factor^2 ≈ (1 - (0.01/cutoff)^2)^n ~ 1):
expected_int_x = p.rate.gain_i(1) * err(1) * dt * n;
% Below the int_lim 0.30, so no clamping yet.
assert(rc.rate_int(1) > 0, 'Integrator accumulates positive');
assert_near(rc.rate_int(1), expected_int_x, 5e-5, 'Integrator value matches PID accumulation');

%% Case 4: integrator clamped to MC_RR_INT_LIM = 0.30.
% Rate of accumulation: i_factor * Ki * err * dt ≈ 1 * 0.2 * 1.0 * 1e-3
% = 2e-4 per step, so ~1500 steps reach 0.30. Run 3000 to ensure clamp.
rc = RateController(p);
err = [1.0; 0; 0];           % large enough to saturate quickly
dt = 1e-3;
for i = 1:3000
    rc.update([0;0;0], err, [0;0;0], dt, false);
end
assert(abs(rc.rate_int(1) - p.rate.int_lim(1)) < 1e-9, 'Integrator clamps at MC_RR_INT_LIM');

%% Case 5: landed -> no integrator update.
rc = RateController(p);
rc.update([0;0;0], [0.1;0;0], [0;0;0], 1e-3, true);
assert_near(rc.rate_int, [0;0;0], 1e-12, 'Integrator frozen while landed');

%% Case 6: D term subtracts angular accel. Zero error, alpha=1 rad/s^2.
rc = RateController(p);
torque = rc.update([0;0;0], [0;0;0], [1; 0; 0], 1e-3, false);
expected = -p.rate.gain_d .* [1;0;0];    % -Kd * alpha
assert_near(torque, expected, 1e-12, 'D term applies negative angular_accel');

fprintf('test_rate_controller: PASS\n');
end


function addpaths()
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(fullfile(root, 'src', 'params'));
addpath(fullfile(root, 'src', 'math'));
addpath(fullfile(root, 'src', 'controllers'));
end


function assert_near(a, b, tol, msg)
if any(abs(a - b) > tol, 'all')
    error('FAIL: %s (got %s, expected %s)', msg, mat2str(a, 6), mat2str(b, 6));
end
end
