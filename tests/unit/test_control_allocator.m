function test_control_allocator()
% Unit tests for ControlAllocator with PX4-style normalization.
% Reference: ControlAllocationPseudoInverse.cpp:80-145 (normalization),
%            ControlAllocationSequentialDesaturationTest.cpp (motor convention).
%
% Post-normalization convention: an axis input of 1.0 saturates motors.

addpaths();
p = px4_params();
ca = ControlAllocator(p);

%% Case 1: pure thrust, T=0.5, expect all motors at 0.5 (matches MPC_THR_HOVER).
m = ca.allocate([0;0;0], 0.5);
assert_near(m, [0.5; 0.5; 0.5; 0.5], 1e-9, 'Pure thrust T=0.5 -> 0.5 each');

%% Case 2: pure thrust at saturation.
m = ca.allocate([0;0;0], 1.0);
assert_near(m, [1; 1; 1; 1], 1e-9, 'T=1.0 -> motors saturate at 1.0');

%% Case 3: pure +yaw at unit input (= max yaw command). After normalization
%  yaw column is [+1; +1; -1; -1] (CW rotors increase, CCW decrease).
%  With T=0.5 baseline:
%   raw m = T_col*0.5 + yaw_col*1 = [0.5;0.5;0.5;0.5] + [1;1;-1;-1] = [1.5;1.5;-0.5;-0.5]
%   clamped to [0,1] -> [1;1;0;0]
m = ca.allocate([0; 0; 1], 0.5);
assert_near(m, [1; 1; 0; 0], 1e-9, 'Pure yaw at unit input: CW saturate, CCW clipped to 0');

%% Case 4: pure +roll at small input keeps everyone in [0,1].
%  After normalization roll_scale = sqrt((4*0.25^2) / (4/2)) = 0.3536.
%  Roll column is [-0.7071; +0.7071; +0.7071; -0.7071] per unit input.
%  With T=0.5 + roll=0.1 -> each motor: 0.5 + (-0.07071), (+0.07071), (+0.07071), (-0.07071)
roll_col = [-1; 1; 1; -1] * (0.25 / sqrt(0.125));
m = ca.allocate([0.1; 0; 0], 0.5);
expected = 0.5 + 0.1 * roll_col;
assert_near(m, expected, 1e-9, 'Small +roll command shifts left motors up');

%% Case 5: pure +pitch (front motors increase by symmetry).
pitch_col = [1; -1; 1; -1] * (0.25 / sqrt(0.125));
m = ca.allocate([0; 0.1; 0], 0.5);
expected = 0.5 + 0.1 * pitch_col;
assert_near(m, expected, 1e-9, 'Small +pitch raises front rotors');

%% Case 6: zero input -> zero motors.
m = ca.allocate([0;0;0], 0);
assert_near(m, [0; 0; 0; 0], 1e-9, 'Zero everything -> zero motors');

%% Case 7: signs of pure-axis responses (qualitative check, no clipping).
%  Roll +: rotors 1, 2 (left side, py<0) increase; 0, 3 (right) decrease.
m = ca.allocate([0.05; 0; 0], 0.5);
assert(m(2) > m(1) && m(2) > m(4), 'Roll+: BL > FR and BL > BR');
assert(m(3) > m(1) && m(3) > m(4), 'Roll+: FL > FR and FL > BR');
%  Pitch +: rotors 0, 2 (front, px>0) increase.
m = ca.allocate([0; 0.05; 0], 0.5);
assert(m(1) > m(2) && m(1) > m(4), 'Pitch+: FR > BL and FR > BR');
assert(m(3) > m(2) && m(3) > m(4), 'Pitch+: FL > BL and FL > BR');
%  Yaw +: rotors 0, 1 (CW, km>0) increase.
m = ca.allocate([0; 0; 0.05], 0.5);
assert(m(1) > m(3) && m(1) > m(4), 'Yaw+: FR > FL and FR > BR');
assert(m(2) > m(3) && m(2) > m(4), 'Yaw+: BL > FL and BL > BR');

fprintf('test_control_allocator: PASS\n');
end


function addpaths()
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(fullfile(root, 'src', 'params'));
addpath(fullfile(root, 'src', 'controllers'));
end


function assert_near(a, b, tol, msg)
if any(abs(a - b) > tol, 'all')
    error('FAIL: %s (got %s, expected %s)', msg, mat2str(a, 6), mat2str(b, 6));
end
end
