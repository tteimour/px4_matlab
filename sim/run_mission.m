function results = run_mission()
% End-to-end mission sim:  Navigator -> PositionControl -> AttitudeControl
%   -> RateControl -> ControlAllocator -> QuadrotorDynamics.
%
% Mission: a 500 m-square pattern at 30 m altitude, traversing
% NE -> SE -> SW -> NW corners and back to start, then landing.
%
% Loops run at PX4 rates (CLAUDE.md):
%   navigator  10 Hz
%   position   50 Hz
%   attitude  250 Hz
%   rate     1000 Hz   (the master sim tick)
%
% Two figures are saved:
%   sim/mission_result.png   trajectory + altitude + motors
%   sim/tracking_errors.png  velocity / attitude / rate setpoint vs actual

% --- path setup ---
here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(fullfile(root, 'src', 'params'));
addpath(fullfile(root, 'src', 'math'));
addpath(fullfile(root, 'src', 'plant'));
addpath(fullfile(root, 'src', 'controllers'));
addpath(fullfile(root, 'src', 'navigator'));

p = px4_params();

% --- modules ---
plant = QuadrotorDynamics(p);
plant.reset([0; 0; 0], 0);

pos_ctl  = PositionController(p);
att_ctl  = AttitudeController(p);
rate_ctl = RateController(p);
alloc    = ControlAllocator(p);

% --- mission: 500 m square at 30 m altitude (NED z = -30) ---
alt_ned = -30;
side = 250;     % half-side -> 500 m square
wp = [
    %  N      E      D       yaw     acc_rad
       0      0      alt_ned   0       3;     % takeoff
       side   0      alt_ned   0       NaN;   % NE corner
       side   side   alt_ned   pi/2    NaN;   % SE
      -side   side   alt_ned   pi      NaN;   % SW
      -side  -side   alt_ned  -pi/2    NaN;   % NW
       side  -side   alt_ned   0       NaN;   % back to N
       0      0      alt_ned   0       3;     % return-to-start
       0      0     -1         0       1;     % land (1 m altitude)
];
nav = Navigator(p, wp);

% --- timing ---
dt_rate = 1 / p.rate_hz.rate;        % 1 ms
n_att   = round(p.rate_hz.rate / p.rate_hz.attitude);    % 4
n_pos   = round(p.rate_hz.rate / p.rate_hz.position);    % 20
n_nav   = round(p.rate_hz.rate / p.rate_hz.navigator);   % 100
dt_pos  = n_pos * dt_rate;
dt_nav  = n_nav * dt_rate;

T_max = 400;                  % seconds, hard cap
N_max = T_max * p.rate_hz.rate;

% --- preallocate logs (50 Hz logging) ---
log_stride = round(p.rate_hz.rate / 50);
n_log = ceil(N_max / log_stride);
log.t        = zeros(n_log, 1);
log.pos      = zeros(n_log, 3);
log.vel      = zeros(n_log, 3);
log.q        = zeros(n_log, 4);
log.omega    = zeros(n_log, 3);
log.pos_sp   = zeros(n_log, 3);
log.vel_sp   = zeros(n_log, 3);
log.q_sp     = zeros(n_log, 4);
log.rate_sp  = zeros(n_log, 3);
log.motors   = zeros(n_log, 4);
log_i = 0;

% --- persistent setpoints between rates (downsampling state) ---
q_sp           = [1; 0; 0; 0];
thrust_body_z  = -p.pos.thr_hover;
yawspeed_sp    = NaN;
rate_sp        = [0; 0; 0];
pos_sp         = [0; 0; 0];
yaw_sp         = 0;
vel_sp         = [0; 0; 0];
vel_sp_ff      = [0; 0; 0];

mission_done = false;
landed = true;

for k = 0:N_max-1
    t = k * dt_rate;
    s = plant.state();

    % ---- NAV (10 Hz) ----
    if mod(k, n_nav) == 0
        [pos_sp, yaw_sp, vel_sp_ff, mission_done] = nav.update(s.position_ned, dt_nav);
    end

    % ---- POS (50 Hz) ----
    if mod(k, n_pos) == 0
        [q_sp, thrust_body_z, vel_sp, ~, ~] = pos_ctl.update( ...
            s.position_ned, s.velocity_ned, s.acceleration_ned, ...
            pos_sp, yaw_sp, vel_sp_ff, [], dt_pos);
        yawspeed_sp = NaN;
    end

    % ---- ATT (250 Hz) ----
    if mod(k, n_att) == 0
        rate_sp = att_ctl.update(s.attitude_q, q_sp, yawspeed_sp);
    end

    % ---- RATE (1000 Hz) ----
    angular_accel = [0; 0; 0];
    landed = (s.position_ned(3) > -0.5) && (norm(s.velocity_ned) < 0.3);
    torque = rate_ctl.update(s.angular_vel_b, rate_sp, angular_accel, dt_rate, landed);

    % ---- ALLOC ----
    T_mag = max(0, -thrust_body_z);
    [m, sat_pos, sat_neg] = alloc.allocate(torque, T_mag);
    rate_ctl.setSaturationStatus(sat_pos, sat_neg);

    % ---- PLANT ----
    plant.step(m, dt_rate);

    % ---- LOG ----
    if mod(k, log_stride) == 0
        log_i = log_i + 1;
        log.t(log_i)        = t;
        log.pos(log_i, :)   = s.position_ned';
        log.vel(log_i, :)   = s.velocity_ned';
        log.q(log_i, :)     = s.attitude_q';
        log.omega(log_i, :) = s.angular_vel_b';
        log.pos_sp(log_i, :) = pos_sp';
        log.vel_sp(log_i, :) = vel_sp';
        log.q_sp(log_i, :)   = q_sp';
        log.rate_sp(log_i, :) = rate_sp';
        log.motors(log_i, :) = m';
    end

    if mission_done && norm(s.velocity_ned) < 0.2 && s.position_ned(3) > -2
        break;
    end
end

% --- truncate logs ---
fields = fieldnames(log);
for f = 1:numel(fields)
    log.(fields{f}) = log.(fields{f})(1:log_i, :);
end

results.log = log;
results.waypoints = wp;
results.params = p;

plot_mission(results);
plot_tracking(results);
end


function plot_mission(r)
log = r.log;
wp = r.waypoints;

fig = figure('Name', 'PX4-MATLAB mission', 'Color', 'w', 'Position', [80 80 1200 720]);

% --- 3D trajectory (NED -> ENU plot for human-readable axes) ---
ax3 = subplot(2, 2, [1 3]);
hold(ax3, 'on'); grid(ax3, 'on'); axis(ax3, 'equal');
plot3(ax3, log.pos(:,2), log.pos(:,1), -log.pos(:,3), 'b-', 'LineWidth', 1.5);
plot3(ax3, log.pos_sp(:,2), log.pos_sp(:,1), -log.pos_sp(:,3), 'g-', 'LineWidth', 0.8);
plot3(ax3, wp(:,2), wp(:,1), -wp(:,3), 'rs--', 'MarkerSize', 9, ...
      'MarkerFaceColor', [1 0.7 0.7], 'LineWidth', 1.2);
plot3(ax3, log.pos(1,2), log.pos(1,1), -log.pos(1,3), 'go', ...
      'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(ax3, log.pos(end,2), log.pos(end,1), -log.pos(end,3), 'kx', ...
      'MarkerSize', 12, 'LineWidth', 2);
xlabel(ax3, 'East (m)'); ylabel(ax3, 'North (m)'); zlabel(ax3, 'Up (m)');
title(ax3, 'Trajectory'); legend(ax3, {'flown', 'pos\_sp (slewed)', 'waypoints', 'start', 'end'}, 'Location', 'best');
view(ax3, 35, 25);

% --- altitude vs time ---
ax_a = subplot(2, 2, 2);
plot(ax_a, log.t, -log.pos(:,3), 'b-', 'LineWidth', 1.2);
hold(ax_a, 'on');
plot(ax_a, log.t, -log.pos_sp(:,3), 'r--', 'LineWidth', 1);
grid(ax_a, 'on'); xlabel(ax_a, 't (s)'); ylabel(ax_a, 'altitude (m)');
title(ax_a, 'Altitude'); legend(ax_a, {'actual', 'setpoint'}, 'Location', 'best');

% --- motor commands ---
ax_m = subplot(2, 2, 4);
plot(ax_m, log.t, log.motors); grid(ax_m, 'on');
xlabel(ax_m, 't (s)'); ylabel(ax_m, 'motor cmd [0,1]');
title(ax_m, 'Motors (0=FR, 1=BL, 2=FL, 3=BR)');
legend(ax_m, {'FR', 'BL', 'FL', 'BR'}, 'Location', 'best');

drawnow;

out_path = fullfile(fileparts(mfilename('fullpath')), 'mission_result.png');
try, exportgraphics(fig, out_path, 'Resolution', 150);
catch, saveas(fig, out_path); end
fprintf('Saved %s\n', out_path);
end


function plot_tracking(r)
log = r.log;
N = numel(log.t);

% Convert quaternions -> Euler angles for human-readable plots.
rpy_actual = zeros(N, 3);
rpy_sp     = zeros(N, 3);
for i = 1:N
    rpy_actual(i, :) = quat_to_euler(log.q(i, :)')';
    rpy_sp(i, :)     = quat_to_euler(log.q_sp(i, :)')';
end
% Unwrap yaw to avoid wrap discontinuities in the plot.
rpy_actual(:, 3) = unwrap(rpy_actual(:, 3));
rpy_sp(:, 3)     = unwrap(rpy_sp(:, 3));

fig = figure('Name', 'PX4-MATLAB tracking errors', 'Color', 'w', 'Position', [120 120 1400 900]);

axis_names_v = {'v_N (m/s)', 'v_E (m/s)', 'v_D (m/s)'};
axis_names_a = {'roll (deg)', 'pitch (deg)', 'yaw (deg)'};
axis_names_r = {'p (deg/s)', 'q (deg/s)', 'r (deg/s)'};

% --- Row 1: velocity tracking ---
for j = 1:3
    ax = subplot(3, 3, j);
    plot(ax, log.t, log.vel_sp(:, j), 'r--', 'LineWidth', 1.0); hold(ax, 'on');
    plot(ax, log.t, log.vel(:, j),    'b-',  'LineWidth', 1.0);
    grid(ax, 'on');
    title(ax, sprintf('Velocity %s', axis_names_v{j}));
    xlabel(ax, 't (s)'); ylabel(ax, axis_names_v{j});
    if j == 1, legend(ax, {'setpoint', 'actual'}, 'Location', 'best'); end
end

% --- Row 2: attitude tracking (Euler, deg) ---
for j = 1:3
    ax = subplot(3, 3, 3 + j);
    plot(ax, log.t, rad2deg(rpy_sp(:, j)),     'r--', 'LineWidth', 1.0); hold(ax, 'on');
    plot(ax, log.t, rad2deg(rpy_actual(:, j)), 'b-',  'LineWidth', 1.0);
    grid(ax, 'on');
    title(ax, sprintf('Attitude %s', axis_names_a{j}));
    xlabel(ax, 't (s)'); ylabel(ax, axis_names_a{j});
end

% --- Row 3: body-rate tracking ---
for j = 1:3
    ax = subplot(3, 3, 6 + j);
    plot(ax, log.t, rad2deg(log.rate_sp(:, j)), 'r--', 'LineWidth', 1.0); hold(ax, 'on');
    plot(ax, log.t, rad2deg(log.omega(:, j)),   'b-',  'LineWidth', 1.0);
    grid(ax, 'on');
    title(ax, sprintf('Body rate %s', axis_names_r{j}));
    xlabel(ax, 't (s)'); ylabel(ax, axis_names_r{j});
end

drawnow;

out_path = fullfile(fileparts(mfilename('fullpath')), 'tracking_errors.png');
try, exportgraphics(fig, out_path, 'Resolution', 150);
catch, saveas(fig, out_path); end
fprintf('Saved %s\n', out_path);

% --- compact RMS error report to stdout ---
err_v = log.vel - log.vel_sp;
err_a = rpy_actual - rpy_sp;
err_r = log.omega - log.rate_sp;
fprintf('\nRMS tracking errors:\n');
fprintf('  velocity   (m/s):  vN=%.3f  vE=%.3f  vD=%.3f\n', sqrt(mean(err_v.^2)));
fprintf('  attitude   (deg):  roll=%.3f  pitch=%.3f  yaw=%.3f\n', rad2deg(sqrt(mean(err_a.^2))));
fprintf('  body rate  (deg/s): p=%.3f  q=%.3f  r=%.3f\n', rad2deg(sqrt(mean(err_r.^2))));

format_all_figures();
vertical_line();
end

