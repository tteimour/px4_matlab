classdef CesiumBridge < handle
%CESIUMBRIDGE  Stream px4_matlab vehicle pose to the Cesium-Unity scene.
%
% Publishes geometry_msgs/PoseArray on /world/default/pose/info as a native
% ROS 2 node, taking the place of the PX4 SITL gz_x500 pose source. The
% Cesium-Unity scene reads poses[1] (cesium-unity-samples
% Assets/SynapSim/Components/PoseSubscriber.cs:21) and Vehicle.cs:68-76 maps
% that Gazebo ENU/FLU pose into Unity, so this node emits exactly that frame.
% Conversion is done by ned_frd_to_enu_flu(); ROS quaternion order is
% (x,y,z,w) whereas px4_matlab uses [w;x;y;z].
%
% Also publishes the vehicle attitude as geometry_msgs/QuaternionStamped on
% /matlab/attitude (default): the NATIVE body->NED FRD quaternion q_b2n (NOT
% the ENU/FLU pose orientation), stamped with the same sim time as the pose.
% A consumer that needs a body(FRD)->camera extrinsic -- e.g. the
% hybrid_tracker_vpi ego-motion projector -- uses this directly; the ego-shift
% projector only uses the inter-frame attitude DELTA, so NED vs ENU world
% cancels and only the FRD body convention matters.
%
% Runtime requirement on the Unity side -- ONLY rosbridge is needed:
%   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
% PX4 SITL, the Micro-XRCE-DDS agent, Gazebo and ros_gz_bridge are NOT
% required: MATLAB is the pose source.
%
% MATLAB and rosbridge must share the same ROS_DOMAIN_ID (default 0).
%
% Usage:
%   bridge = CesiumBridge();                 % default topic + node name
%   bridge.publish(pos_ned, q_b2n, t_sim);   % call every frame
%   ...
%   delete(bridge);                          % tear down the ROS 2 node

    properties (SetAccess = immutable)
        Node
        Publisher
        Topic
        AttPublisher    % geometry_msgs/QuaternionStamped (body->NED FRD attitude)
        AttTopic
    end

    properties (Access = private)
        msg     % cached PoseArray (2 poses) reused every publish
        amsg    % cached QuaternionStamped reused every publish
    end

    methods
        function obj = CesiumBridge(topic, nodeName, domainID, attTopic)
            if nargin < 1 || isempty(topic),    topic    = '/world/default/pose/info'; end
            if nargin < 2 || isempty(nodeName), nodeName = '/px4_matlab_bridge';       end
            if nargin < 3 || isempty(domainID), domainID = 0;                           end
            if nargin < 4 || isempty(attTopic), attTopic = '/matlab/attitude';          end

            % Make the pure conversion reachable regardless of caller cwd.
            here = fileparts(mfilename('fullpath'));
            addpath(fullfile(fileparts(here), 'math'));

            obj.Topic     = topic;
            obj.Node      = ros2node(nodeName, domainID);
            obj.Publisher = ros2publisher(obj.Node, topic, 'geometry_msgs/PoseArray');

            % Pre-build a 2-element PoseArray. poses(1) is the gz world/ground
            % placeholder that Unity ignores; poses(2) is the vehicle.
            m = ros2message('geometry_msgs/PoseArray');
            m.header.frame_id = 'map';
            p = ros2message('geometry_msgs/Pose');
            p.orientation.w = 1;          % identity placeholder
            m.poses = [p; p];             % 2x1 struct array
            obj.msg = m;

            % Attitude publisher: native body->NED FRD quaternion (q_b2n),
            % NOT the ENU/FLU pose orientation, so a body(FRD)->camera
            % extrinsic downstream stays valid. ROS order is (x,y,z,w).
            obj.AttTopic     = attTopic;
            obj.AttPublisher = ros2publisher(obj.Node, attTopic, ...
                                             'geometry_msgs/QuaternionStamped');
            a = ros2message('geometry_msgs/QuaternionStamped');
            a.header.frame_id = 'ned';
            a.quaternion.w = 1;           % identity placeholder
            obj.amsg = a;
        end

        function publish(obj, pos_ned, q_b2n, t_sim)
            % PX4 NED/FRD -> Gazebo ENU/FLU.
            [pos_enu, q_enu_flu] = ned_frd_to_enu_flu(pos_ned, q_b2n);

            if nargin < 4 || isempty(t_sim), t_sim = 0; end
            sec  = floor(t_sim);
            nsec = (t_sim - sec) * 1e9;

            m = obj.msg;
            m.header.stamp.sec     = int32(sec);
            m.header.stamp.nanosec = uint32(nsec);

            % poses(2) = vehicle (Unity reads the 0-indexed poses[1]).
            m.poses(2).position.x    = pos_enu(1);
            m.poses(2).position.y    = pos_enu(2);
            m.poses(2).position.z    = pos_enu(3);
            m.poses(2).orientation.w = q_enu_flu(1);
            m.poses(2).orientation.x = q_enu_flu(2);
            m.poses(2).orientation.y = q_enu_flu(3);
            m.poses(2).orientation.z = q_enu_flu(4);

            send(obj.Publisher, m);
            obj.msg = m;

            % Native body->NED FRD attitude (q_b2n = [w;x;y;z]) for the tracker,
            % stamped identically to the pose so they pair by timestamp.
            a = obj.amsg;
            a.header.stamp.sec     = int32(sec);
            a.header.stamp.nanosec = uint32(nsec);
            a.quaternion.w = q_b2n(1);
            a.quaternion.x = q_b2n(2);
            a.quaternion.y = q_b2n(3);
            a.quaternion.z = q_b2n(4);
            send(obj.AttPublisher, a);
            obj.amsg = a;
        end
    end
end
