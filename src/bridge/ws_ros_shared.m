function ros = ws_ros_shared(host, port)
%WS_ROS_SHARED  Session-persistent roslibpy Ros connection (rosbridge WS).
%
% Shared by every WebSocket bridge (CesiumBridgeWs, ImuBridgeWs,
% VioOdomSubWs). Twisted's reactor can be started only ONCE per process and
% MATLAB keeps Python alive for the whole session, so the Ros object is
% created + run() once and reused for every bridge/run. delete() of the
% bridges never stops the reactor. If you ever hit ReactorNotRestartable,
% restart MATLAB once and re-run.
%
% Prerequisites:
%   * Python 3.9-3.12 configured in MATLAB (check with `pyenv`)
%   * pip install roslibpy
%   * rosbridge running and reachable at ws://host:port

if nargin < 1 || isempty(host), host = 'localhost'; end
if nargin < 2 || isempty(port), port = 9090; end

persistent ROS

% Reuse an existing, still-connected session.
if ~isempty(ROS)
    try
        if logical(ROS.is_connected)
            ros = ROS;
            return;
        end
    catch
        ROS = [];   % stale handle -> rebuild below
    end
end

ROS = py.roslibpy.Ros(host, int32(port));
try
    ROS.run();                      % start the reactor (first time in process)
catch ME
    if contains(ME.message, 'ReactorNotRestartable')
        ROS.connect();              % reactor already running -> just connect
    else
        rethrow(ME);
    end
end

t0 = tic;
while ~logical(ROS.is_connected) && toc(t0) < 5
    pause(0.1);
end
if ~logical(ROS.is_connected)
    error('ws_ros_shared:connect', ...
        'Could not connect to ws://%s:%d (is rosbridge running?).', host, port);
end
ros = ROS;
end
