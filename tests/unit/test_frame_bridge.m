function test_frame_bridge()
% Unit tests for ned_frd_to_enu_flu (px4_matlab -> Cesium/Unity pose bridge).
% Verifies the NED/FRD -> ENU/FLU conversion against hand-computed cases.
% Frame contract: cesium-unity-samples Assets/SynapSim/Components/Vehicle.cs
% consumes a Gazebo ENU-world / FLU-body pose on /world/default/pose/info.

addpaths();

%% Case A: level, heading North; position N=10 E=20 D=-5 (i.e. 5 m up)
[pos, q] = ned_frd_to_enu_flu([10; 20; -5], [1; 0; 0; 0]);
% Position -> ENU [E; N; U]
assert_near(pos, [20; 10; 5], 1e-9, 'A position (NED->ENU)');
% Heading North is +90 deg yaw about ENU Up: q = [cos45; 0; 0; sin45]
assert_near(q, [sqrt(2)/2; 0; 0; sqrt(2)/2], 1e-9, 'A orientation (North -> +90 yaw)');

%% Case B: level, heading East (PX4 yaw +90 deg about Down) -> ENU/FLU identity
qN_east = [sqrt(2)/2; 0; 0; sqrt(2)/2];
[~, q] = ned_frd_to_enu_flu([0; 0; 0], qN_east);
assert_near(q, [1; 0; 0; 0], 1e-9, 'B orientation (East -> ENU identity)');

%% Case C: heading North + 90 deg roll right (about body forward, FRD x)
qN_roll = [sqrt(2)/2; sqrt(2)/2; 0; 0];
[~, q] = ned_frd_to_enu_flu([0; 0; 0], qN_roll);
assert_near(q, [0.5; 0.5; 0.5; 0.5], 1e-9, 'C orientation (North + roll right)');
assert_near(norm(q), 1, 1e-12, 'C quaternion is unit norm');

%% Invariant: for level/North, body Up (FLU) must map to ENU Up
[~, qA] = ned_frd_to_enu_flu([0; 0; 0], [1; 0; 0; 0]);
R = quat_to_dcm(qA);                 % body(FLU) -> ENU
assert_near(R * [0; 0; 1], [0; 0; 1], 1e-9, 'A body-Up (FLU) -> ENU Up');

fprintf('test_frame_bridge: all assertions passed.\n');
end


function addpaths()
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(fullfile(root, 'src', 'math'));
end


function assert_near(a, b, tol, msg)
if any(abs(a - b) > tol, 'all')
    error('FAIL: %s (got %s, expected %s)', msg, mat2str(a(:)', 6), mat2str(b(:)', 6));
end
end
