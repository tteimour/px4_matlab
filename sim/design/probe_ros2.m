function probe_ros2()
% Probe MATLAB ROS2 capabilities relevant to the LIVE FEED panel.
ml = ros2('msg','list');
fprintf('NMSG=%d\n', numel(ml));
fprintf('Detection2DArray=%d\n', any(strcmp(ml,'vision_msgs/Detection2DArray')));
fprintf('Detection2D=%d\n', any(strcmp(ml,'vision_msgs/Detection2D')));
fprintf('CompressedImage=%d\n', any(strcmp(ml,'sensor_msgs/CompressedImage')));
fprintf('Image=%d\n', any(strcmp(ml,'sensor_msgs/Image')));
fprintf('RegionOfInterest=%d\n', any(strcmp(ml,'sensor_msgs/RegionOfInterest')));
fprintf('Float32MultiArray=%d\n', any(strcmp(ml,'std_msgs/Float32MultiArray')));
fprintf('anyVisionMsgs=%d\n', any(contains(ml,'vision_msgs')));
fprintf('rosReadImage=%d  ros2genmsg=%d\n', exist('rosReadImage'), exist('ros2genmsg'));
end
