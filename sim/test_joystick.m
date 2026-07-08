function test_joystick(id)
% TEST_JOYSTICK  Live monitor for a physical game controller (PS4 / DS4 etc).
%
% Run this FIRST after plugging in the pad. It shows, updating live:
%   * every raw axis by index (so you can see which index each stick moves)
%   * every button state
%   * the four mapped sim sticks (left_x/left_y/right_x/right_y) using
%     JoystickReader's default map
%
% Move the LEFT stick: left_x (yaw) and left_y (throttle) should respond.
% Move the RIGHT stick: right_x (roll) and right_y (pitch) should respond.
% Stick UP must give POSITIVE left_y / right_y. If indices or signs are
% wrong, note the correct axis index per channel and pass a map override to
% JoystickReader, e.g. JoystickReader(1, struct('rx',4,'ry',5)).
%
% Usage:  test_joystick        % device id 1 (default)
%         test_joystick(2)

if nargin < 1 || isempty(id), id = 1; end

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(fullfile(root, 'src', 'io'));

js = JoystickReader(id);
if ~js.isConnected()
    error(['test_joystick: could not open joystick id %d.\n' ...
           'Is the pad connected? Check that /dev/input/js%d exists ' ...
           '(ls /dev/input/js*) and try `jstest /dev/input/js0`.'], id, id-1);
end

fprintf('Joystick id %d opened: %d axes, %d buttons.\n', ...
        id, js.nAxes, js.nButtons);
fprintf('Move each stick and confirm the mapped sticks respond correctly.\n');
fprintf('Close the window (or press the STOP button) to finish.\n');

% --- Build the monitor figure ---------------------------------------------
fig = figure('Name', sprintf('Joystick test  (id %d)', id), ...
             'NumberTitle', 'off', 'Color', [0.12 0.12 0.14], ...
             'Position', [200 200 560 520]);

ax = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.30 0.40 0.65 0.55]);
bars = barh(ax, 1:js.nAxes, zeros(1, js.nAxes), 'FaceColor', [0.20 0.70 0.95]);
set(ax, 'XLim', [-1.1 1.1], 'YLim', [0.5 js.nAxes+0.5], ...
        'YTick', 1:js.nAxes, 'YDir', 'reverse', ...
        'Color', [0.16 0.16 0.18], 'XColor', 'w', 'YColor', 'w');
ylabel(ax, 'axis index'); xlabel(ax, 'value  [-1, 1]');
title(ax, 'Raw axes', 'Color', 'w');
yticklabels(ax, arrayfun(@(i) sprintf('axis %d', i), 1:js.nAxes, ...
            'UniformOutput', false));

txt = annotation(fig, 'textbox', [0.03 0.02 0.94 0.32], ...
                 'Color', 'w', 'EdgeColor', [0.3 0.3 0.3], ...
                 'FontName', 'monospaced', 'FontSize', 11, ...
                 'Interpreter', 'none', 'BackgroundColor', [0.16 0.16 0.18], ...
                 'String', '');

running = true;
uicontrol(fig, 'Style', 'pushbutton', 'String', 'STOP', ...
          'Units', 'normalized', 'Position', [0.03 0.93 0.18 0.06], ...
          'BackgroundColor', [0.8 0.3 0.3], 'ForegroundColor', 'w', ...
          'FontWeight', 'bold', 'Callback', @(~,~) stop());
set(fig, 'CloseRequestFcn', @(src,~) stop(src));

    function stop(src)
        running = false;
        if nargin >= 1 && ishandle(src), delete(src); end
    end

% --- Live loop ------------------------------------------------------------
cleanupObj = onCleanup(@() js.close());
while running && ishandle(fig)
    [s, btn, axraw] = js.read();
    if isempty(s)                       % pad unplugged mid-test -> wait
        txt.String = 'Joystick disconnected -- plug it back in...';
        drawnow limitrate; continue;
    end

    % barh: XData = category locations (fixed), YData = bar lengths (values).
    set(bars, 'YData', axraw(:)');

    pressed = find(btn > 0.5);
    if isempty(pressed)
        btnStr = '(none)';
    else
        btnStr = strtrim(sprintf('%d ', pressed));
    end

    txt.String = sprintf([ ...
        'MAPPED STICKS (sim convention, up = +):\n' ...
        '  left_x  (yaw)      = %+5.2f\n' ...
        '  left_y  (throttle) = %+5.2f\n' ...
        '  right_x (roll)     = %+5.2f\n' ...
        '  right_y (pitch)    = %+5.2f\n\n' ...
        'BUTTONS pressed: %s'], ...
        s.left_x, s.left_y, s.right_x, s.right_y, btnStr);

    drawnow limitrate;
end

if ishandle(fig), delete(fig); end
fprintf('test_joystick: done.\n');
end
