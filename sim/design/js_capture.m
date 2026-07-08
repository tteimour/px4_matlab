function js_capture()
% Headless capture of JoystickReader channels to confirm the DS4 axis map.
addpath('/home/teymur/git/px4_matlab/src/io');
js = JoystickReader(1);
T = 25; fs = 25; n = T*fs; L = zeros(n,4); A = zeros(n,8);
fprintf('CAPTURE START\n');
for i = 1:n
    [s,~,ax] = js.read();
    L(i,:) = [s.left_x s.left_y s.right_x s.right_y];
    m = min(8, numel(ax)); A(i,1:m) = ax(1:m)';
    pause(1/fs);
end
js.close();
fprintf('--- mapped stick channels  [min  max] ---\n');
fprintf('left_x  (yaw)      [%+.2f %+.2f]\n', min(L(:,1)), max(L(:,1)));
fprintf('left_y  (throttle) [%+.2f %+.2f]\n', min(L(:,2)), max(L(:,2)));
fprintf('right_x (roll)     [%+.2f %+.2f]\n', min(L(:,3)), max(L(:,3)));
fprintf('right_y (pitch)    [%+.2f %+.2f]\n', min(L(:,4)), max(L(:,4)));
fprintf('--- raw triggers (must NOT appear in sticks) ---\n');
fprintf('raw axis3 L2 [%+.2f %+.2f]  raw axis6 R2 [%+.2f %+.2f]\n', ...
        min(A(:,3)), max(A(:,3)), min(A(:,6)), max(A(:,6)));
end
