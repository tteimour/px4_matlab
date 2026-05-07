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

% --- IMU (voted vehicle_imu) ---------------------------------------------
if isfield(log, 'imu_gyro')
    figure('Name', 'IMU (voted)', 'NumberTitle', 'off', 'Color', 'w');
    labels_g = {'p (deg/s)', 'q (deg/s)', 'r (deg/s)'};
    for i = 1:3
        subplot(3, 2, 2*i - 1); hold on; grid on; box on;
        plot(log.t, rad2deg(log.imu_gyro(:, i)), 'LineWidth', 1.0);
        plot(log.t, rad2deg(log.omega(:, i)), '--', 'LineWidth', 1.0);
        ylabel(labels_g{i});
        if i == 1, title('Gyro: voted IMU vs ground truth'); end
        if i == 3, xlabel('time (s)'); legend({'imu', 'truth'}, 'Location', 'best'); end
    end
    labels_a = {'a_x (m/s^2)', 'a_y (m/s^2)', 'a_z (m/s^2)'};
    for i = 1:3
        subplot(3, 2, 2*i); hold on; grid on; box on;
        plot(log.t, log.imu_accel(:, i), 'LineWidth', 1.0);
        ylabel(labels_a{i});
        if i == 1, title('Accel (specific force, body frame)'); end
        if i == 3, xlabel('time (s)'); end
    end
end

% --- Barometer -----------------------------------------------------------
if isfield(log, 'baro_alt')
    figure('Name', 'Barometer (voted)', 'NumberTitle', 'off', 'Color', 'w');
    hold on; grid on; box on;
    truth_alt = -log.pos(:, 3);              % NED z negated -> altitude AGL
    plot(log.t, log.baro_alt - log.baro_alt(find(~isnan(log.baro_alt), 1)), ...
         'LineWidth', 1.2);
    plot(log.t, truth_alt, '--', 'LineWidth', 1.0);
    xlabel('time (s)'); ylabel('altitude (m)');
    title('Voted baro altitude (zeroed at start) vs ground-truth altitude');
    legend({'baro - baro(0)', 'truth (-z)'}, 'Location', 'best');
end

% --- Magnetometer --------------------------------------------------------
if isfield(log, 'mag_b')
    figure('Name', 'Magnetometer (voted)', 'NumberTitle', 'off', 'Color', 'w');
    labels_m = {'mag_x (G)', 'mag_y (G)', 'mag_z (G)'};
    for i = 1:3
        subplot(3, 1, i); hold on; grid on; box on;
        plot(log.t, log.mag_b(:, i), 'LineWidth', 1.0);
        ylabel(labels_m{i});
        if i == 1, title('Voted magnetometer body-frame field'); end
        if i == 3, xlabel('time (s)'); end
    end
end

% --- GNSS ----------------------------------------------------------------
if isfield(log, 'gps_pos')
    figure('Name', 'GNSS (voted)', 'NumberTitle', 'off', 'Color', 'w');
    labels_p = {'pN (m)', 'pE (m)', 'pD (m)'};
    for i = 1:3
        subplot(3, 2, 2*i - 1); hold on; grid on; box on;
        plot(log.t, log.gps_pos(:, i), 'LineWidth', 1.0);
        plot(log.t, log.pos(:, i), '--', 'LineWidth', 1.0);
        ylabel(labels_p{i});
        if i == 1, title('GPS position (NED) vs truth'); end
        if i == 3, xlabel('time (s)'); legend({'gps', 'truth'}, 'Location', 'best'); end
    end
    labels_v = {'vN (m/s)', 'vE (m/s)', 'vD (m/s)'};
    for i = 1:3
        subplot(3, 2, 2*i); hold on; grid on; box on;
        plot(log.t, log.gps_vel(:, i), 'LineWidth', 1.0);
        plot(log.t, log.vel(:, i), '--', 'LineWidth', 1.0);
        ylabel(labels_v{i});
        if i == 1, title('GPS velocity (NED) vs truth'); end
        if i == 3, xlabel('time (s)'); end
    end
end

% --- Estimator vs ground truth -------------------------------------------
if isfield(log, 'est_pos')
    figure('Name', 'Estimator vs truth', 'NumberTitle', 'off', 'Color', 'w');
    labels = {'pN (m)', 'pE (m)', 'pD (m)'};
    for i = 1:3
        subplot(3, 3, i); hold on; grid on; box on;
        plot(log.t, log.est_pos(:, i), 'LineWidth', 1.2);
        plot(log.t, log.pos(:, i), '--', 'LineWidth', 1.0);
        ylabel(labels{i});
        if i == 2, title('EKF state (solid) vs ground truth (dashed)'); end
    end
    labels = {'vN (m/s)', 'vE (m/s)', 'vD (m/s)'};
    for i = 1:3
        subplot(3, 3, 3 + i); hold on; grid on; box on;
        plot(log.t, log.est_vel(:, i), 'LineWidth', 1.2);
        plot(log.t, log.vel(:, i), '--', 'LineWidth', 1.0);
        ylabel(labels{i});
    end
    labels = {'roll (deg)', 'pitch (deg)', 'yaw (deg)'};
    for i = 1:3
        subplot(3, 3, 6 + i); hold on; grid on; box on;
        plot(log.t, rad2deg(log.est_rpy(:, i)), 'LineWidth', 1.2);
        plot(log.t, rad2deg(log.rpy(:, i)), '--', 'LineWidth', 1.0);
        ylabel(labels{i});
        if i == 2, xlabel('time (s)'); end
    end
end

try, format_all_figures(); catch, end
try, vertical_line();      catch, end
end
