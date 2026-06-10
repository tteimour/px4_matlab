classdef VioOdomSubWs < handle
%VIOODOMSUBWS  Poll OpenVINS odometry via rosbridge WebSocket.
%
% Transport alternative to a native ros2subscriber on /ov_msckf/odomimu for
% setups where MATLAB cannot join the ROS 2 DDS domain (MATLAB on Windows
% with OpenVINS inside WSL2). rosbridge forwards the WSL-side topic over the
% WebSocket; messages land in a Python-side latest-value store
% (vio_ws_store.py) because roslibpy callbacks run on Twisted's background
% thread, from which MATLAB must not be re-entered. The sim loop POLLS
% latest() once per frame instead — the existing per-stamp dedup in the
% consumer makes repeated polls of the same message harmless.
%
% latest() returns a struct shaped like the ros2 nav_msgs/Odometry message
% (header.stamp.sec/nanosec, pose.pose.position/orientation,
% twist.twist.linear/...) so downstream consumers are unchanged, or [] when
% nothing has arrived yet.
%
% Usage:
%   sub  = VioOdomSubWs();          % localhost:9090, /ov_msckf/odomimu
%   vmsg = sub.latest();            % poll each frame
%   delete(sub);

    properties (SetAccess = immutable)
        Topic
    end

    properties (Access = private)
        topicObj   % py.roslibpy.Topic
        store      % py vio_ws_store.VioWsStore
        last_count = -1
        last_msg   = []
    end

    methods
        function obj = VioOdomSubWs(host, port, topic)
            if nargin < 1 || isempty(host),  host  = 'localhost'; end
            if nargin < 2 || isempty(port),  port  = 9090; end
            if nargin < 3 || isempty(topic), topic = '/ov_msckf/odomimu'; end

            try
                py.importlib.import_module('roslibpy');
            catch
                error('VioOdomSubWs:roslibpy', ...
                    ['roslibpy not found in MATLAB''s Python.\n' ...
                     '  1) check `pyenv` shows a valid Python 3.9-3.12\n' ...
                     '  2) pip install roslibpy  (into that Python)']);
            end

            % Make vio_ws_store.py importable (lives next to this file).
            here = fileparts(mfilename('fullpath'));
            if ~any(cellfun(@(s) strcmp(string(s), here), cell(py.sys.path)))
                insert(py.sys.path, int32(0), here);
            end
            mod = py.importlib.import_module('vio_ws_store');

            obj.Topic    = topic;
            ros          = ws_ros_shared(host, port);
            obj.store    = mod.VioWsStore();
            obj.topicObj = py.roslibpy.Topic(ros, topic, 'nav_msgs/Odometry');
            obj.topicObj.subscribe(obj.store);
        end

        function s = latest(obj)
            % Newest odometry as a MATLAB struct, or [] if none / unchanged
            % conversion is cached per message (cheap repeated polling).
            s = [];
            cnt = double(obj.store.count);
            if cnt == 0
                return;
            end
            if cnt == obj.last_count
                s = obj.last_msg;
                return;
            end
            m = obj.store.msg;
            if isequal(m, py.None)
                return;
            end
            % py dict -> JSON -> struct gives the same nested field names
            % as the ros2 message struct (header.stamp.sec, pose.pose...).
            s = jsondecode(char(py.json.dumps(m)));
            obj.last_count = cnt;
            obj.last_msg   = s;
        end

        function delete(obj)
            % Unsubscribe only; the shared reactor/connection must survive
            % for the rest of the MATLAB session (see ws_ros_shared).
            try
                obj.topicObj.unsubscribe();
            catch
                % nothing to clean up
            end
        end
    end
end
