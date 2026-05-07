function plot_sim(log)
% Plot tracking signals from a run_interactive log.
%
%   plot_sim()        -- pulls `sim_log` from the base workspace (where
%                        run_interactive writes it on Stop).
%   plot_sim(log)     -- plot a log struct you have in hand.
%
% After drawing, format_all_figures() and vertical_line() are applied if
% they exist on the path.

if nargin < 1
    try
        log = evalin('base', 'sim_log');
    catch
        error(['plot_sim: no `sim_log` in base workspace. ' ...
               'Run run_interactive and press Stop first.']);
    end
end

% --- Position tracking + active flight mode trace -------------------------
figure('Name', 'Position tracking', 'NumberTitle', 'off', 'Color', 'w');
labels = {'North (m)', 'East (m)', 'Down (m)'};
for i = 1:3
    subplot(4, 1, i); hold on; grid on; box on;
    plot(log.t, log.pos(:, i),    'LineWidth', 1.4);
    plot(log.t, log.pos_sp(:, i), '--', 'LineWidth', 1.0);
    ylabel(labels{i});
    if i == 1, title('Position tracking (solid = actual, dashed = setpoint)'); end
end
subplot(4, 1, 4); hold on; grid on; box on;
plot(log.t, log.mode_idx, 'LineWidth', 1.4);
if isfield(log, 'mode_strings')
    yticks(1:numel(log.mode_strings));
    yticklabels(log.mode_strings);
    ylim([0.5, numel(log.mode_strings) + 0.5]);
end
ylabel('mode'); xlabel('time (s)');

% --- Velocity ------------------------------------------------------------
figure('Name', 'Velocity tracking', 'NumberTitle', 'off', 'Color', 'w');
labels = {'vN (m/s)', 'vE (m/s)', 'vD (m/s)'};
for i = 1:3
    subplot(3, 1, i); hold on; grid on; box on;
    plot(log.t, log.vel(:, i),    'LineWidth', 1.4);
    plot(log.t, log.vel_sp(:, i), '--', 'LineWidth', 1.0);
    ylabel(labels{i});
    if i == 1, title('Velocity tracking (solid = actual, dashed = setpoint)'); end
    if i == 3, xlabel('time (s)'); end
end

% --- Attitude ------------------------------------------------------------
figure('Name', 'Attitude tracking', 'NumberTitle', 'off', 'Color', 'w');
labels = {'roll (deg)', 'pitch (deg)', 'yaw (deg)'};
for i = 1:3
    subplot(3, 1, i); hold on; grid on; box on;
    plot(log.t, rad2deg(log.rpy(:, i)),    'LineWidth', 1.4);
    plot(log.t, rad2deg(log.rpy_sp(:, i)), '--', 'LineWidth', 1.0);
    ylabel(labels{i});
    if i == 1, title('Attitude tracking (solid = actual, dashed = setpoint)'); end
    if i == 3, xlabel('time (s)'); end
end

% --- Body rates ----------------------------------------------------------
figure('Name', 'Body rate tracking', 'NumberTitle', 'off', 'Color', 'w');
labels = {'p (deg/s)', 'q (deg/s)', 'r (deg/s)'};
for i = 1:3
    subplot(3, 1, i); hold on; grid on; box on;
    plot(log.t, rad2deg(log.omega(:, i)),   'LineWidth', 1.4);
    plot(log.t, rad2deg(log.rate_sp(:, i)), '--', 'LineWidth', 1.0);
    ylabel(labels{i});
    if i == 1, title('Body rate tracking (solid = actual, dashed = setpoint)'); end
    if i == 3, xlabel('time (s)'); end
end

% --- Motors --------------------------------------------------------------
figure('Name', 'Motor outputs', 'NumberTitle', 'off', 'Color', 'w');
hold on; grid on; box on;
plot(log.t, log.motor, 'LineWidth', 1.2);
xlabel('time (s)'); ylabel('motor command [0..1]');
title('Per-rotor allocator output');
legend({'m1', 'm2', 'm3', 'm4'}, 'Location', 'best');

try, format_all_figures(); catch, end
try, vertical_line();      catch, end
end
