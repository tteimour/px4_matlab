function test_attitude_controller()
% Unit tests for AttitudeController.
% Hand-computed against AttitudeControl.cpp:55-114.

addpaths();
p = px4_params();

%% Case 1: q == q_sp -> rate setpoint is zero (no error).
ac = AttitudeController(p);
q = [1; 0; 0; 0];
rate_sp = ac.update(q, q, NaN);
assert_near(rate_sp, [0;0;0], 1e-12, 'Identity attitude error -> zero rate');

%% Case 2: small pitch error (about body y), expect rate_sp(2) ~ MC_PITCH_P * angle.
ac = AttitudeController(p);
q   = [1; 0; 0; 0];                        % level
ang = 0.05;                                % 0.05 rad about y
qsp = [cos(ang/2); 0; sin(ang/2); 0];      % rotation about world-y
rate_sp = ac.update(q, qsp, NaN);
% e_q = 2*Im(q_e canonical). For pure pitch, q_e = (cos(ang/2),0,sin(ang/2),0)
% so |e_q| = 2*sin(ang/2). Rate demand = MC_PITCH_P * 2*sin(ang/2).
% (Equals MC_PITCH_P * ang only in the small-angle limit.)
expected_pitch = p.att.gain_p(2) * 2 * sin(ang/2);
assert(abs(rate_sp(1)) < 1e-9, 'No roll demand for pitch error');
assert(abs(rate_sp(3)) < 1e-9, 'No yaw demand for pitch error');
assert_near(rate_sp(2), expected_pitch, 1e-12, 'Pitch rate demand = MC_PITCH_P * 2*sin(ang/2)');

%% Case 3: rate-setpoint clamp at MC_*RATE_MAX.
ac = AttitudeController(p);
q   = [1; 0; 0; 0];
ang = pi/2;
qsp = [cos(ang/2); 0; sin(ang/2); 0];      % 90 deg pitch error
rate_sp = ac.update(q, qsp, NaN);
% MC_PITCHRATE_MAX = 220 deg/s. Demand is way above; should saturate.
assert(abs(rate_sp(2) - p.att.rate_max(2)) < 1e-9, 'Pitch rate clamps to MC_PITCHRATE_MAX');

%% Case 4: yaw-only error, expect proportional rate via yaw_w/(1/yaw_w) compensation.
ac = AttitudeController(p);
q   = [1; 0; 0; 0];
ang = 0.05;
qsp = [cos(ang/2); 0; 0; sin(ang/2)];      % small yaw rotation
rate_sp = ac.update(q, qsp, NaN);
% Effective yaw P = MC_YAW_P (since the controller divides by yaw_w
% internally, then the yaw_w scaling re-applies). For pure yaw, output
% equals MC_YAW_P * sin(yaw_w * angle).
% Reduced attitude: q_red = identity (already aligned in tilt), then
% qd_dyaw = qd. Recomposed qd = (cos(yaw_w*angle/2), 0, 0, sin(yaw_w*angle/2)).
% e_q = 2 * Im(qd) = (0, 0, sin(yaw_w*angle))? Wait, Im of qd_canonical is
% (0,0, sin(yaw_w*angle/2)). 2 * that = (0, 0, 2*sin(yaw_w*angle/2)).
% rate_sp_z = (MC_YAW_P / yaw_w) * 2 * sin(yaw_w*angle/2)
%           ≈ MC_YAW_P * angle for small angles.
expected_yaw = (p.att.gain_p(3) / p.att.yaw_weight) * 2 * sin(p.att.yaw_weight * ang/2);
assert_near(rate_sp(3), expected_yaw, 1e-9, 'Yaw demand matches Brescianini formula');

fprintf('test_attitude_controller: PASS\n');
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
