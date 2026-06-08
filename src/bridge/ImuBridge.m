classdef ImuBridge < handle
%IMUBRIDGE  Stream px4_matlab IMU samples to OpenVINS as sensor_msgs/Imu.
%
% Publishes sensor_msgs/Imu on /vio_imu/data as a native ROS 2 node, taking
% the place of the Gazebo iris_vio gazebo_ros IMU plugin that previously fed
% OpenVINS. The voted vehicle IMU sample (est_bus.sensors.vehicleImu()) is the
% raw sensor stream the EKF consumes; that is exactly what a VIO front-end
% needs (NOT the EKF-corrected state).
%
% Frame: the IMU is published in body FRD with NO conversion. gyro_b is the
% body angular rate (rad/s) and accel_b is the body specific force (m/s^2,
% gravity included), matching ImuSensor.measure(). OpenVINS resolves gravity
% and builds its own Z-up global frame, and maps the camera into this body
% frame via T_imu_cam in kalibr_imucam_chain.yaml — so the FRD IMU frame here
% must agree with that extrinsic.
%
% Timestamps: header.stamp is filled from the sample's sim time (imu.t),
% identical to how CesiumBridge stamps the pose with t_sim. Because the
% Cesium-Unity camera copies the pose stamp onto the rendered image
% (Vehicle.cs StreamCameras), stamping the IMU with sim time too puts the
% camera and IMU in ONE clock domain — the prerequisite for VIO. Do not stamp
% with wall time: the image carries sim time and the domains would not match.
%
% Runtime requirement (OpenVINS side): OpenVINS is a native ROS 2 node, so it
% receives this topic over DDS directly. MATLAB and OpenVINS must share the
% same ROS_DOMAIN_ID (default 0). rosbridge is NOT needed for the IMU (only
% the Unity camera path uses rosbridge).
%
% Usage:
%   bridge = ImuBridge();                       % default topic + node name
%   bridge.publish(gyro_b, accel_b, imu.t);     % call at ~200 Hz
%   ...
%   delete(bridge);                             % tear down the ROS 2 node

    properties (SetAccess = immutable)
        Node
        Publisher
        Topic
    end

    properties (Access = private)
        msg     % cached sensor_msgs/Imu reused every publish
    end

    methods
        function obj = ImuBridge(topic, nodeName, domainID)
            if nargin < 1 || isempty(topic),    topic    = '/vio_imu/data';      end
            if nargin < 2 || isempty(nodeName), nodeName = '/px4_matlab_imu';    end
            if nargin < 3 || isempty(domainID), domainID = 0;                    end

            obj.Topic     = topic;
            obj.Node      = ros2node(nodeName, domainID);
            obj.Publisher = ros2publisher(obj.Node, topic, 'sensor_msgs/Imu');

            % Pre-build the message. frame_id 'imu' is the OpenVINS IMU frame.
            % orientation is not provided by this sensor stream; the leading
            % covariance term = -1 is the sensor_msgs/Imu convention for "no
            % orientation estimate" (REP-145). OpenVINS ignores orientation.
            m = ros2message('sensor_msgs/Imu');
            m.header.frame_id          = 'imu';
            m.orientation.w            = 1;
            m.orientation_covariance(1) = -1;
            obj.msg = m;
        end

        function publish(obj, gyro_b, accel_b, t_sim)
            if nargin < 4 || isempty(t_sim), t_sim = 0; end
            sec  = floor(t_sim);
            nsec = (t_sim - sec) * 1e9;

            m = obj.msg;
            m.header.stamp.sec     = int32(sec);
            m.header.stamp.nanosec = uint32(nsec);

            % Body FRD, no conversion (see class help).
            m.angular_velocity.x = gyro_b(1);
            m.angular_velocity.y = gyro_b(2);
            m.angular_velocity.z = gyro_b(3);

            m.linear_acceleration.x = accel_b(1);
            m.linear_acceleration.y = accel_b(2);
            m.linear_acceleration.z = accel_b(3);

            send(obj.Publisher, m);
            obj.msg = m;
        end
    end
end
