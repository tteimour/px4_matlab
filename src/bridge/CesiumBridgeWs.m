classdef CesiumBridgeWs < handle
%CESIUMBRIDGEWS  Stream pose to the Cesium-Unity scene via rosbridge WebSocket.
%
% Transport alternative to CesiumBridge for setups where a native ROS 2 (DDS)
% node can't reach rosbridge -- notably MATLAB on Windows with rosbridge in
% WSL2, where cross-boundary DDS does not work but a WebSocket to
% ws://localhost:9090 does (WSL2 NAT auto-forwards localhost ports).
%
% It connects to rosbridge with Python's roslibpy and publishes
% geometry_msgs/PoseArray on /world/default/pose/info. The frame conversion
% (PX4 NED/FRD -> Gazebo ENU/FLU) is shared with the DDS bridge via
% ned_frd_to_enu_flu, and Unity reads poses[1] (PoseSubscriber.cs:21).
%
% Prerequisites:
%   * Python 3.9-3.12 configured in MATLAB  (check with `pyenv`)
%   * pip install roslibpy
%   * rosbridge running (NAT networking, not mirrored):
%       ros2 launch rosbridge_server rosbridge_websocket_launch.xml
%     reachable at ws://localhost:9090.
%
% Usage:
%   b = CesiumBridgeWs();                    % localhost:9090
%   b.publish(pos_ned, q_b2n, t_sim);        % call every frame
%   delete(b);

    properties (SetAccess = immutable)
        Topic
    end

    properties (Access = private)
        ros        % py.roslibpy.Ros
        topicObj   % py.roslibpy.Topic
    end

    methods
        function obj = CesiumBridgeWs(host, port, topic)
            if nargin < 1 || isempty(host),  host  = 'localhost'; end
            if nargin < 2 || isempty(port),  port  = 9090; end
            if nargin < 3 || isempty(topic), topic = '/world/default/pose/info'; end

            here = fileparts(mfilename('fullpath'));
            addpath(fullfile(fileparts(here), 'math'));   % ned_frd_to_enu_flu

            % Fail early and clearly if roslibpy isn't importable.
            try
                py.importlib.import_module('roslibpy');
            catch
                error('CesiumBridgeWs:roslibpy', ...
                    ['roslibpy not found in MATLAB''s Python.\n' ...
                     '  1) check `pyenv` shows a valid Python 3.9-3.12\n' ...
                     '  2) pip install roslibpy  (into that Python)']);
            end

            obj.Topic = topic;
            obj.ros   = py.roslibpy.Ros(host, int32(port));
            obj.ros.run();   % connect on a background thread (non-blocking)

            t0 = tic;
            while ~logical(obj.ros.is_connected) && toc(t0) < 5
                pause(0.1);
            end
            if ~logical(obj.ros.is_connected)
                error('CesiumBridgeWs:connect', ...
                    'Could not connect to ws://%s:%d (is rosbridge running?).', host, port);
            end

            obj.topicObj = py.roslibpy.Topic(obj.ros, topic, 'geometry_msgs/PoseArray');
            obj.topicObj.advertise();
        end

        function publish(obj, pos_ned, q_b2n, t_sim)
            if nargin < 4 || isempty(t_sim), t_sim = 0; end
            [pos_enu, q] = ned_frd_to_enu_flu(pos_ned, q_b2n);   % q = [w;x;y;z]

            sec  = floor(t_sim);
            nsec = round((t_sim - sec) * 1e9);

            % poses{1} = ignored placeholder; poses{2} = vehicle (Unity reads poses[1]).
            zeroPose = struct('position',    struct('x', 0, 'y', 0, 'z', 0), ...
                              'orientation', struct('x', 0, 'y', 0, 'z', 0, 'w', 1));
            vehPose  = struct('position',    struct('x', pos_enu(1), 'y', pos_enu(2), 'z', pos_enu(3)), ...
                              'orientation', struct('x', q(2), 'y', q(3), 'z', q(4), 'w', q(1)));

            % sec/nanosec are strict integer fields in ROS; cast to an integer
            % type so jsonencode emits integer literals (a double can be encoded
            % as "2e7"/"2.0e7" on some MATLAB releases -> parsed as a float ->
            % rosbridge rejects it with "nanosec field must be of type 'int'").
            m = struct();
            m.header = struct('stamp', struct('sec', int32(sec), 'nanosec', int32(nsec)), ...
                              'frame_id', 'map');
            m.poses  = {zeroPose, vehPose};   % cell -> JSON array of two poses

            % Hand the message to roslibpy as a dict (struct -> JSON -> py dict).
            msg = py.roslibpy.Message(py.json.loads(jsonencode(m)));
            obj.topicObj.publish(msg);
        end

        function delete(obj)
            % Best-effort teardown; ignore errors if never fully connected.
            try
                obj.topicObj.unadvertise();
            catch
                % nothing to clean up
            end
            try
                obj.ros.terminate();
            catch
                % nothing to clean up
            end
        end
    end
end
