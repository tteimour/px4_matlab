classdef ImuBridgeWs < handle
%IMUBRIDGEWS  Stream IMU samples to OpenVINS via rosbridge WebSocket.
%
% Transport alternative to ImuBridge (native DDS) for setups where MATLAB
% cannot join the ROS 2 DDS domain — notably MATLAB on Windows with ROS 2 +
% OpenVINS inside WSL2, where cross-boundary DDS does not work but a
% WebSocket to ws://localhost:9090 does. rosbridge republishes the message
% into the WSL DDS domain where OpenVINS subscribes natively.
%
% Message semantics are identical to ImuBridge (see its header): body-FRD
% gyro/specific-force with NO frame conversion, sim-time stamps so the IMU
% shares one clock domain with the Unity camera images, frame_id 'imu',
% orientation_covariance(1) = -1 ("no orientation estimate", REP-145).
%
% Prerequisites: Python 3.9-3.12 in `pyenv`, pip install roslibpy, and
% rosbridge running (ws_ros_shared connects on first use).
%
% Usage:
%   bridge = ImuBridgeWs();                     % localhost:9090, default topic
%   bridge.publish(gyro_b, accel_b, imu.t);     % call at imu_pub_hz
%   delete(bridge);

    properties (SetAccess = immutable)
        Topic
    end

    properties (Access = private)
        topicObj   % py.roslibpy.Topic
    end

    methods
        function obj = ImuBridgeWs(host, port, topic)
            if nargin < 1 || isempty(host),  host  = 'localhost'; end
            if nargin < 2 || isempty(port),  port  = 9090; end
            if nargin < 3 || isempty(topic), topic = '/vio_imu/data'; end

            try
                py.importlib.import_module('roslibpy');
            catch
                error('ImuBridgeWs:roslibpy', ...
                    ['roslibpy not found in MATLAB''s Python.\n' ...
                     '  1) check `pyenv` shows a valid Python 3.9-3.12\n' ...
                     '  2) pip install roslibpy  (into that Python)']);
            end

            obj.Topic    = topic;
            ros          = ws_ros_shared(host, port);
            obj.topicObj = py.roslibpy.Topic(ros, topic, 'sensor_msgs/Imu');
            obj.topicObj.advertise();
        end

        function publish(obj, gyro_b, accel_b, t_sim)
            if nargin < 4 || isempty(t_sim), t_sim = 0; end
            sec  = floor(t_sim);
            nsec = round((t_sim - sec) * 1e9);

            % Integer-typed stamp fields so jsonencode emits integer
            % literals (rosbridge rejects "nanosec" given as a float).
            m = struct();
            m.header = struct('stamp', struct('sec', int32(sec), ...
                                              'nanosec', int32(nsec)), ...
                              'frame_id', 'imu');
            m.orientation = struct('x', 0, 'y', 0, 'z', 0, 'w', 1);
            m.orientation_covariance = [-1, 0, 0, 0, 0, 0, 0, 0, 0];
            m.angular_velocity = struct('x', gyro_b(1), 'y', gyro_b(2), ...
                                        'z', gyro_b(3));
            m.angular_velocity_covariance = zeros(1, 9);
            m.linear_acceleration = struct('x', accel_b(1), 'y', accel_b(2), ...
                                           'z', accel_b(3));
            m.linear_acceleration_covariance = zeros(1, 9);

            msg = py.roslibpy.Message(py.json.loads(jsonencode(m)));
            obj.topicObj.publish(msg);
        end

        function delete(obj)
            % Unadvertise only; the shared reactor/connection must survive
            % for the rest of the MATLAB session (see ws_ros_shared).
            try
                obj.topicObj.unadvertise();
            catch
                % nothing to clean up
            end
        end
    end
end
