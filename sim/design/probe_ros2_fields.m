function probe_ros2_fields()
% Confirm MATLAB ros2 message field names + rosReadImage on a ros2 Image.
im = ros2message('sensor_msgs/Image');
fprintf('IMAGE_FIELDS: %s\n', strjoin(fieldnames(im), ', '));
roi = ros2message('sensor_msgs/RegionOfInterest');
fprintf('ROI_FIELDS: %s\n', strjoin(fieldnames(roi), ', '));
fa = ros2message('std_msgs/Float32MultiArray');
fprintf('F32_FIELDS: %s\n', strjoin(fieldnames(fa), ', '));

% 2x2 rgb8 test image -> rosReadImage should give 2x2x3 uint8
im.height = uint32(2); im.width = uint32(2); im.encoding = 'rgb8'; im.step = uint32(6);
im.data = uint8((1:12)');
try
    I = rosReadImage(im);
    fprintf('ROSREADIMAGE_OK size=%s class=%s px11=%s\n', ...
            mat2str(size(I)), class(I), mat2str(squeeze(I(1,1,:))'));
catch ME
    fprintf('ROSREADIMAGE_FAIL %s\n', ME.message);
end
end
