function run_cesium_bridge(duration_s)
%RUN_CESIUM_BRIDGE  Stream a synthetic trajectory to the Cesium-Unity scene.
%
% Standalone end-to-end check of the MATLAB -> Unity pose bridge, independent
% of the full run_interactive GUI. It flies a level circle (a synthetic test
% trajectory, NOT a physics sim) and publishes pose at 50 Hz so you can
% confirm the drone moves and yaws correctly on the Cesium globe.
%
% Setup (Unity side):
%   1) ros2 launch rosbridge_server rosbridge_websocket_launch.xml
%   2) Play the Quba scene in Unity.
%   (PX4 SITL, the Micro-XRCE agent and Gazebo are NOT required.)
%
% Then in MATLAB:  run_cesium_bridge        % 60 s default
%                  run_cesium_bridge(20)    % 20 s
%
% See CesiumBridge.m for the topic / frame contract.

if nargin < 1 || isempty(duration_s), duration_s = 60; end

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(fullfile(root, 'src', 'math'));
addpath(fullfile(root, 'src', 'bridge'));

bridge  = CesiumBridge();
cleanup = onCleanup(@() delete(bridge));   %#ok<NASGU>  tear down node on exit/Ctrl+C

fs  = 50;          % publish rate [Hz]
dt  = 1 / fs;
R   = 10;          % circle radius [m]
T   = 20;          % circle period [s]
alt = 5;           % altitude above origin [m]  (NED D = -alt)
w   = 2 * pi / T;

fprintf('Streaming to %s at %d Hz for %.0f s. Ctrl+C to stop.\n', ...
        bridge.Topic, fs, duration_s);

n = round(duration_s * fs);
for k = 0:n
    t  = k * dt;
    th = w * t;
    pos_ned = [R * cos(th); R * sin(th); -alt];

    % Heading tangent to the circle (face direction of travel); level.
    psi   = th + pi/2;
    q_b2n = [cos(psi/2); 0; 0; sin(psi/2)];   % yaw about Down (FRD z)

    bridge.publish(pos_ned, q_b2n, t);

    if mod(k, fs) == 0
        % Wrap heading to (-180, 180] for display (matches quat_to_euler);
        % the quaternion sent above is periodic regardless, so the raw psi
        % growing past 360 deg is only a print artifact, not a real drift.
        yaw_deg = rad2deg(mod(psi + pi, 2*pi) - pi);
        fprintf('  t=%4.1fs  N=%+6.2f E=%+6.2f U=%+5.2f  yaw=%+6.1f deg\n', ...
                t, pos_ned(1), pos_ned(2), alt, yaw_deg);
    end
    pause(dt);
end
fprintf('Done.\n');
end
