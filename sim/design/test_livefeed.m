function test_livefeed()
% End-to-end MATLAB ROS2 round-trip for the LIVE FEED + lock wiring:
%   publish a fake /detection/image (rgb8) + /detection/boxes, confirm the
%   run_interactive-style callbacks receive them (rosReadImage decode +
%   Float32MultiArray), then build + publish a RegionOfInterest from slot 1's
%   box and confirm the values round-trip.
fig = figure('Visible','off');
node = ros2node('test_livefeed_node');

pimg = ros2publisher(node, '/detection/image', 'sensor_msgs/Image');
pbox = ros2publisher(node, '/detection/boxes', 'std_msgs/Float32MultiArray');
proi = ros2publisher(node, '/tracker/roi', 'sensor_msgs/RegionOfInterest');

simg = ros2subscriber(node, '/detection/image', 'sensor_msgs/Image', ...
    @(m) setappdata(fig, 'det_image', rosReadImage(m))); %#ok<NASGU>
sbox = ros2subscriber(node, '/detection/boxes', 'std_msgs/Float32MultiArray', ...
    @(m) setappdata(fig, 'det_boxes', double(m.data(:)))); %#ok<NASGU>
sroi = ros2subscriber(node, '/tracker/roi', 'sensor_msgs/RegionOfInterest', ...
    @(m) setappdata(fig, 'last_roi', double([m.x_offset m.y_offset m.width m.height]))); %#ok<NASGU>
pause(1.0);   % discovery

W = 512; H = 512;
im = ros2message('sensor_msgs/Image');
im.height = uint32(H); im.width = uint32(W); im.encoding = 'rgb8';
im.is_bigendian = uint8(0); im.step = uint32(3*W);
im.data = uint8(mod(0:(H*W*3-1), 256)');
send(pimg, im);

fa = ros2message('std_msgs/Float32MultiArray');
fa.data = single([2, 100,120,40,60, 300,260,80,50]);
send(pbox, fa);

for k = 1:30
    pause(0.1);
    if ~isempty(getappdata(fig,'det_image')) && ~isempty(getappdata(fig,'det_boxes')), break; end
end
di = getappdata(fig,'det_image'); db = getappdata(fig,'det_boxes');
fprintf('IMG_SIZE=%s\n', mat2str(size(di)));
fprintf('BOXES=%s\n', mat2str(db'));

% emulate the slot-1 (L1) lock: cx=100 cy=120 w=40 h=60 -> ROI [80 90 40 60]
slot = 1; off = 2 + (slot-1)*4;
cx = db(off); cy = db(off+1); w = db(off+2); h = db(off+3);
roi = ros2message('sensor_msgs/RegionOfInterest');
roi.x_offset = uint32(max(0, round(cx - w/2)));
roi.y_offset = uint32(max(0, round(cy - h/2)));
roi.width    = uint32(round(w));
roi.height   = uint32(round(h));
send(proi, roi);
for k = 1:30, pause(0.1); if ~isempty(getappdata(fig,'last_roi')), break; end, end
fprintf('ROI_RX=%s EXPECT=[80 90 40 60]\n', mat2str(getappdata(fig,'last_roi')));
delete(fig);
end
