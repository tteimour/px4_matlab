function test_stick_override()
% Unit tests for StickOverrideDetector.
% Hand-computed against PX4 manual_control sticks_moving logic:
%   ManualControl.cpp:115-122, MovingDiff.hpp, AlphaFilter.hpp:68-77.
% threshold = 0.01 * COM_RC_STICK_OV.

addpaths();
p = px4_params();
ov = p.com.rc_stick_ov;                 % 30 (%) by default
thr = 0.01 * ov;                        % 0.30 norm-stick/s

centred = struct('left_x', 0, 'left_y', 0, 'right_x', 0, 'right_y', 0);

%% Case 1: first sample never triggers (MovingDiff has no previous value).
d = StickOverrideDetector(ov);
assert(~d.update(centred, 0.02), 'First sample must not trigger');

%% Case 2: held centre sticks never trigger (rate stays 0).
d = StickOverrideDetector(ov);
moved = false;
for i = 1:200
    moved = moved || d.update(centred, 0.02);
end
assert(~moved, 'Held centre sticks must never trigger override');

%% Case 3: held NON-centre deflection does not keep triggering.
%  A constant deflection has zero rate of change, so once the entry
%  transient is absorbed the detector must fall silent. Feed a constant
%  0.8 roll (right_x) after the first sample establishes "last".
d = StickOverrideDetector(ov);
held = centred; held.right_x = 0.8;
d.update(held, 0.02);                   % establishes last = 0.8 (returns 0)
moved = false;
for i = 1:200
    moved = moved || d.update(held, 0.02);
end
assert(~moved, 'Constant deflection must not keep triggering');

%% Case 4: slow drift below threshold does NOT trigger.
%  Constant-rate ramp: the filtered derivative converges to the ramp rate.
%  Rate 0.2/s < 0.30 threshold, so it must never fire regardless of frames.
d = StickOverrideDetector(ov);
dt = 0.02; rate = 0.2; val = 0;
moved = false;
for i = 1:200
    val = val + rate * dt;
    s = centred; s.left_y = val;        % throttle drifting up slowly
    moved = moved || d.update(s, dt);
end
assert(rate < thr, 'sanity: chosen slow rate is below threshold');
assert(~moved, 'Sub-threshold drift must not trigger override');

%% Case 5: deliberate movement above threshold DOES trigger.
%  Ramp rate 0.5/s > 0.30; the filter converges toward 0.5 and crosses the
%  threshold within a handful of frames (alpha = dt/(tau+dt) = 0.1667).
d = StickOverrideDetector(ov);
dt = 0.02; rate = 0.5; val = 0;
moved = false; first_fire = -1;
for i = 1:50
    val = val + rate * dt;
    s = centred; s.right_y = val;       % pitch ramped deliberately
    if d.update(s, dt)
        moved = true;
        if first_fire < 0, first_fire = i; end
    end
end
assert(rate > thr, 'sanity: chosen fast rate exceeds threshold');
assert(moved, 'Above-threshold movement must trigger override');
% alpha=0.1667: state = rate*(1-(1-alpha)^n) crosses 0.30 at n>=6
% (first update returns 0, so fire no earlier than the 7th call).
assert(first_fire >= 6, 'Fire timing matches AlphaFilter convergence');

%% Case 6: any single axis is sufficient (yaw = left_x).
d = StickOverrideDetector(ov);
dt = 0.02; rate = 0.5; val = 0; moved = false;
for i = 1:50
    val = val + rate * dt;
    s = centred; s.left_x = val;        % yaw only
    moved = moved || d.update(s, dt);
end
assert(moved, 'Yaw-only movement must trigger override');

%% Case 7: threshold scales with COM_RC_STICK_OV.
%  At a high threshold (80%), the same 0.5/s ramp (< 0.80) must NOT fire.
d = StickOverrideDetector(80);
dt = 0.02; rate = 0.5; val = 0; moved = false;
for i = 1:200
    val = val + rate * dt;
    s = centred; s.right_x = val;
    moved = moved || d.update(s, dt);
end
assert(~moved, 'Ramp below a raised threshold must not trigger');

%% Case 8: reset() clears history (no spurious diff after a jump).
d = StickOverrideDetector(ov);
s = centred; s.right_x = 0.9;
d.update(s, 0.02);                      % establish last = 0.9
d.reset();
% After reset the next sample is a "first sample" again -> returns 0 even
% though right_x jumped from (forgotten) 0.9 to 0.
assert(~d.update(centred, 0.02), 'reset() must clear last-value history');

fprintf('test_stick_override: all cases passed.\n');
end


function addpaths()
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(fullfile(root, 'src', 'params'));
addpath(fullfile(root, 'src', 'flight_modes'));
end
