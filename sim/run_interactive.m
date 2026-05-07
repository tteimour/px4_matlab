function run_interactive()
% Interactive PX4 controller demo with full flight-mode dispatcher.
%
% Modes (from the dropdown), all PX4-faithful:
%   stabilized  attitude + thrust direct (no XY/Z hold)
%   altitude    attitude + altitude hold
%   position    velocity FF (sticks) + XY/Z hold on stick centre
%   hold        loiter at current position
%   mission     follow a list of NED waypoints
%   rtl         climb to RTL_ALT, cruise to home, land
%   land        descend at MPC_LAND_SPEED, crawl near ground
%   takeoff     climb to MIS_TAKEOFF_ALT then auto-Hold
%
% UI:
%   * Two square joystick boxes (mouse-drag, spring back to centre).
%       LEFT  = yaw (X) + throttle (Y)
%       RIGHT = roll (X) + pitch (Y, stick forward = nose down)
%   * Mode dropdown (top-right panel).
%   * Mission editor: free-form text, one "N E D" per line; the Upload
%     button parses it and arms the Navigator.
%   * Set Home: pin RTL home to the current vehicle position.
%   * Reset / Stop.
%
% On Stop, all logged tracking signals are plotted in their own figures
% and then `format_all_figures()` / `vertical_line()` are called.

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(fullfile(root, 'src', 'params'));
addpath(fullfile(root, 'src', 'math'));
addpath(fullfile(root, 'src', 'plant'));
addpath(fullfile(root, 'src', 'controllers'));
addpath(fullfile(root, 'src', 'navigator'));
addpath(fullfile(root, 'src', 'flight_modes'));
addpath(fullfile(root, 'src', 'sensors'));
addpath(fullfile(root, 'src', 'sensors', 'imu'));
addpath(fullfile(root, 'src', 'sensors', 'baro'));
addpath(fullfile(root, 'src', 'sensors', 'mag'));
addpath(fullfile(root, 'src', 'sensors', 'gnss'));
addpath(fullfile(root, 'src', 'sensors', 'voter'));
addpath(fullfile(root, 'src', 'estimator'));

p = px4_params();

range_xy = 50;            % ground-plane half-width (m), purely visual

% =====================================================================
% Figure / 3D axes
% =====================================================================
fig = figure('Name', 'PX4 interactive flight modes', ...
             'NumberTitle', 'off', 'Color', 'w', ...
             'Position', [120 80 1280 800]);

ax = axes('Parent', fig, 'Units', 'normalized', ...
          'Position', [0.04 0.36 0.55 0.62]);
hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on'); axis(ax, 'equal');
view_half = 8;
xlim(ax, [-view_half view_half]);
ylim(ax, [-view_half view_half]);
zlim(ax, [0 view_half * 1.2]);
xlabel(ax, 'East (m)'); ylabel(ax, 'North (m)'); zlabel(ax, 'Altitude (m)');
title(ax, 'PX4 cascaded controller — pick a mode and fly');
view(ax, 35, 25);

gp = range_xy;
ground = patch(ax, [-gp gp gp -gp], [-gp -gp gp gp], [0 0 0 0], ...
      [0.93 0.95 0.93], 'EdgeColor', [0.7 0.75 0.7], 'FaceAlpha', 0.6); %#ok<NASGU>

trail   = animatedline(ax, 'Color', [0.2 0.4 0.9], 'LineWidth', 1.0, ...
                       'MaximumNumPoints', 6000);
sp_dot  = plot3(ax, 0, 0, 0, 'rs', 'MarkerSize', 12, ...
                'MarkerFaceColor', [1 0.7 0.7]);
sp_line = plot3(ax, [0 0], [0 0], [0 0], 'r:', 'LineWidth', 0.8);
mission_h = plot3(ax, NaN, NaN, NaN, 'g--o', 'LineWidth', 1.0, ...
                  'MarkerSize', 6, 'MarkerFaceColor', [0.6 0.95 0.6]);

drone = makeDrone(ax, root);

% Wind indicator: arrow rendered at a fixed offset above the drone,
% direction = total wind in NED, length proportional to speed.
wind_arrow = quiver3(ax, 0, 0, 0, 0, 0, 0, ...
    'Color', [0.10 0.55 0.85], 'LineWidth', 2.0, ...
    'AutoScale', 'off', 'MaxHeadSize', 1.0);

% =====================================================================
% Right panel: mode dropdown + mission editor + state readout
% =====================================================================
right_panel = uipanel(fig, 'Units', 'normalized', ...
    'Position', [0.61 0.36 0.36 0.62], ...
    'Title', 'Control', 'BackgroundColor', 'w', 'FontWeight', 'bold');

mode_strings = {'stabilized', 'altitude', 'position', 'hold', ...
                'mission', 'rtl', 'land', 'takeoff'};
default_mode_idx = 3;             % start in Position
mode_dd = uicontrol(right_panel, 'Style', 'popupmenu', 'Units', 'normalized', ...
    'String', mode_strings, 'Value', default_mode_idx, ...
    'Position', [0.04 0.92 0.92 0.06], ...
    'BackgroundColor', 'w', 'FontWeight', 'bold');

uicontrol(right_panel, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.04 0.86 0.92 0.04], 'BackgroundColor', 'w', ...
    'HorizontalAlignment', 'left', 'String', 'Waypoint (NED, metres):');

% N / E / D number entries with labels.
uicontrol(right_panel, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.04 0.79 0.04 0.05], 'BackgroundColor', 'w', ...
    'HorizontalAlignment', 'right', 'String', 'N');
n_edit = uicontrol(right_panel, 'Style', 'edit', 'Units', 'normalized', ...
    'Position', [0.09 0.79 0.16 0.05], 'String', '0', ...
    'BackgroundColor', [0.99 0.99 0.97]);

uicontrol(right_panel, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.27 0.79 0.04 0.05], 'BackgroundColor', 'w', ...
    'HorizontalAlignment', 'right', 'String', 'E');
e_edit = uicontrol(right_panel, 'Style', 'edit', 'Units', 'normalized', ...
    'Position', [0.32 0.79 0.16 0.05], 'String', '0', ...
    'BackgroundColor', [0.99 0.99 0.97]);

uicontrol(right_panel, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.50 0.79 0.04 0.05], 'BackgroundColor', 'w', ...
    'HorizontalAlignment', 'right', 'String', 'D');
d_edit = uicontrol(right_panel, 'Style', 'edit', 'Units', 'normalized', ...
    'Position', [0.55 0.79 0.16 0.05], 'String', '-5', ...
    'BackgroundColor', [0.99 0.99 0.97]);

uicontrol(right_panel, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.73 0.79 0.23 0.05], 'String', 'Add', 'FontWeight', 'bold', ...
    'Callback', @(~,~) setappdata(fig, 'add_request', true));

uicontrol(right_panel, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.04 0.72 0.30 0.05], 'String', 'Clear', ...
    'Callback', @(~,~) setappdata(fig, 'clear_request', true));
uicontrol(right_panel, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.36 0.72 0.30 0.05], 'String', 'Remove last', ...
    'Callback', @(~,~) setappdata(fig, 'pop_request', true));
uicontrol(right_panel, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.68 0.72 0.28 0.05], 'String', 'Set Home', ...
    'Callback', @(~,~) setappdata(fig, 'set_home_request', true));

wp_listbox = uicontrol(right_panel, 'Style', 'listbox', 'Units', 'normalized', ...
    'Position', [0.04 0.42 0.92 0.28], 'String', {}, ...
    'FontName', 'Courier New', 'FontSize', 9, ...
    'BackgroundColor', [0.99 0.99 0.97]);

state_lbl = uicontrol(right_panel, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.04 0.04 0.92 0.36], 'BackgroundColor', 'w', ...
    'HorizontalAlignment', 'left', 'FontName', 'Courier New', ...
    'FontSize', 9, 'String', '');

% =====================================================================
% Bottom: joystick boxes + Reset/Stop buttons
% =====================================================================
ax_left = axes('Parent', fig, 'Units', 'normalized', ...
               'Position', [0.04 0.04 0.20 0.28]);
ax_right = axes('Parent', fig, 'Units', 'normalized', ...
                'Position', [0.27 0.04 0.20 0.28]);

[left_h, right_h] = makeJoysticks(fig, ax_left, ax_right);

action_panel = uipanel(fig, 'Units', 'normalized', ...
    'Position', [0.50 0.04 0.47 0.28], 'Title', 'Actions', ...
    'BackgroundColor', 'w', 'FontWeight', 'bold');

uicontrol(action_panel, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.04 0.66 0.45 0.28], 'String', 'Reset', 'FontWeight', 'bold', ...
    'Callback', @(~,~) setappdata(fig, 'reset_request', true));
uicontrol(action_panel, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.51 0.66 0.45 0.28], 'String', 'Stop', 'FontWeight', 'bold', ...
    'BackgroundColor', [0.95 0.85 0.85], ...
    'Callback', @(~,~) setappdata(fig, 'running', false));

% Wind sub-panel (steady NED + turbulence sigma).
wind_panel = uipanel(action_panel, 'Units', 'normalized', ...
    'Position', [0.02 0.04 0.96 0.58], 'Title', 'Wind (NED, m/s)', ...
    'BackgroundColor', 'w', 'FontWeight', 'bold');

uicontrol(wind_panel, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.02 0.55 0.07 0.30], 'BackgroundColor', 'w', ...
    'HorizontalAlignment', 'right', 'String', 'N');
wind_n_edit = uicontrol(wind_panel, 'Style', 'edit', 'Units', 'normalized', ...
    'Position', [0.10 0.55 0.18 0.30], 'String', '0', ...
    'BackgroundColor', [0.99 0.99 0.97]);
uicontrol(wind_panel, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.31 0.55 0.07 0.30], 'BackgroundColor', 'w', ...
    'HorizontalAlignment', 'right', 'String', 'E');
wind_e_edit = uicontrol(wind_panel, 'Style', 'edit', 'Units', 'normalized', ...
    'Position', [0.39 0.55 0.18 0.30], 'String', '0', ...
    'BackgroundColor', [0.99 0.99 0.97]);
uicontrol(wind_panel, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.60 0.55 0.07 0.30], 'BackgroundColor', 'w', ...
    'HorizontalAlignment', 'right', 'String', 'D');
wind_d_edit = uicontrol(wind_panel, 'Style', 'edit', 'Units', 'normalized', ...
    'Position', [0.68 0.55 0.18 0.30], 'String', '0', ...
    'BackgroundColor', [0.99 0.99 0.97]);

turb_cb = uicontrol(wind_panel, 'Style', 'checkbox', 'Units', 'normalized', ...
    'Position', [0.02 0.10 0.45 0.30], 'String', 'Turbulence', ...
    'BackgroundColor', 'w', 'Value', 0);
uicontrol(wind_panel, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.46 0.10 0.20 0.30], 'BackgroundColor', 'w', ...
    'HorizontalAlignment', 'right', 'String', 'sigma');
turb_sigma_edit = uicontrol(wind_panel, 'Style', 'edit', 'Units', 'normalized', ...
    'Position', [0.68 0.10 0.18 0.30], 'String', '1.0', ...
    'BackgroundColor', [0.99 0.99 0.97]);

% Estimator toggle: ground-truth state vs EKF2 sensor-driven state.
est_cb = uicontrol(action_panel, 'Style', 'checkbox', 'Units', 'normalized', ...
    'Position', [0.04 0.50 0.92 0.13], 'String', 'Use EKF2 estimator (sensor-driven)', ...
    'BackgroundColor', 'w', 'Value', 0, 'FontWeight', 'bold');

% =====================================================================
% Sim modules
% =====================================================================
plant    = QuadrotorDynamics(p);
plant.reset([0;0;0], 0);
pos_ctl  = PositionController(p);
att_ctl  = AttitudeController(p);
rate_ctl = RateController(p);
alloc    = ControlAllocator(p);
fmm      = FlightModeManager(p);

% Lead-compensator pre-filters (non-PX4 augmentation; see p.lead.*).
% Each filter is reset on Reset and on every mode change so that
% discontinuous setpoint jumps cannot kick the filter state.
pos_lead = LeadCompensator(p.lead.pos.Ts, p.lead.pos.Tp);
vel_lead = LeadCompensator(p.lead.vel.Ts, p.lead.vel.Tp);
att_lead = LeadCompensator(p.lead.att.Ts, p.lead.att.Tp);

% Sensor + estimator stack (V6X_6 hardware, M9N GNSS).
% Earth origin chosen arbitrarily for the sim — a real flight would
% take this from the first GNSS fix (handled by Ekf2.fuseGnssPos).
earth   = EarthModel(47.39773, 8.54559, 488.0);   % Zurich-ish
est_bus = EstimatorBus(earth);

% Wind disturbance: steady NED component + first-order turbulence.
wind = WindModel(p);
set(wind_n_edit,     'String', num2str(p.wind.steady(1)));
set(wind_e_edit,     'String', num2str(p.wind.steady(2)));
set(wind_d_edit,     'String', num2str(p.wind.steady(3)));
set(turb_cb,         'Value',  double(p.wind.turb_enable));
set(turb_sigma_edit, 'String', num2str(p.wind.turb_sigma));

dt_rate = 1 / p.rate_hz.rate;
n_att   = round(p.rate_hz.rate / p.rate_hz.attitude);
n_pos   = round(p.rate_hz.rate / p.rate_hz.position);
dt_pos  = n_pos * dt_rate;

fps      = 50;
dt_frame = 1 / fps;

% Initialize FMM in the chosen mode against the spawned vehicle pose.
fmm.setHome([0; 0; 0]);
prev_mode = mode_strings{default_mode_idx};
fmm.setMode(prev_mode, plant.state());

q_sp          = [1; 0; 0; 0];
thrust_body_z = -p.pos.thr_hover;
yawspeed_sp   = NaN;
rate_sp       = [0; 0; 0];
m_last        = [0; 0; 0; 0];
vel_sp_used   = nan(3, 1);
cmd           = struct('kind', 'position', 'pos_sp', [0;0;0], ...
                       'vel_sp_ff', [], 'acc_sp_ff', [], ...
                       'yaw_sp', 0, 'yawspeed_sp', NaN, ...
                       'mode', prev_mode);

% =====================================================================
% Log buffers (preallocated, trimmed at end)
% =====================================================================
max_log = 60000;
log.t        = nan(max_log, 1);
log.pos      = nan(max_log, 3);     % ground-truth position
log.pos_sp   = nan(max_log, 3);
log.vel      = nan(max_log, 3);     % ground-truth velocity
log.vel_sp   = nan(max_log, 3);
log.rpy      = nan(max_log, 3);     % ground-truth roll/pitch/yaw
log.rpy_sp   = nan(max_log, 3);
log.omega    = nan(max_log, 3);
log.rate_sp  = nan(max_log, 3);
log.motor    = nan(max_log, 4);
log.mode_idx = nan(max_log, 1);
% Sensor + estimator logs — populated every frame regardless of toggle.
log.imu_gyro  = nan(max_log, 3);    % voted vehicle_imu gyro_b (rad/s)
log.imu_accel = nan(max_log, 3);    % voted vehicle_imu accel_b (m/s^2)
log.baro_alt  = nan(max_log, 1);    % voted vehicle_air_data altitude (m)
log.mag_b     = nan(max_log, 3);    % voted vehicle_magnetometer mag_b (G)
log.gps_pos   = nan(max_log, 3);    % vehicle_gps_position pos_ned (m)
log.gps_vel   = nan(max_log, 3);    % vehicle_gps_position vel_ned (m/s)
log.gps_eph   = nan(max_log, 1);
log.est_pos   = nan(max_log, 3);    % EKF/output-predictor position
log.est_vel   = nan(max_log, 3);
log.est_rpy   = nan(max_log, 3);
log.use_est   = false(max_log, 1);  % logical: was the controller fed EKF state?
log_idx      = 0;

setappdata(fig, 'running', true);
setappdata(fig, 'reset_request', false);
setappdata(fig, 'add_request', false);
setappdata(fig, 'clear_request', false);
setappdata(fig, 'pop_request', false);
setappdata(fig, 'set_home_request', false);
waypoints = zeros(0, 3);                        % Nx3 [N E D]

k     = 0;
t_sim = 0;
while ishandle(fig) && getappdata(fig, 'running')
    frame_t0 = tic;

    % --- Reset ---
    if getappdata(fig, 'reset_request')
        plant.reset([0;0;0], 0);
        pos_ctl.reset();
        rate_ctl.reset();
        pos_lead.reset();
        vel_lead.reset();
        att_lead.reset();
        wind.reset();
        est_bus.ekf.reset();
        est_bus.output_pred.reset();
        clearpoints(trail);
        t_sim   = 0;
        log_idx = 0;
        fmm.setHome([0; 0; 0]);
        fmm.setMode(prev_mode, plant.state());
        setappdata(fig, 'reset_request', false);
    end

    % --- Set Home (manual override) ---
    if getappdata(fig, 'set_home_request')
        fmm.setHome(plant.pos_ned);
        setappdata(fig, 'set_home_request', false);
    end

    % --- Add waypoint ---
    if getappdata(fig, 'add_request')
        nv = str2double(get(n_edit, 'String'));
        ev = str2double(get(e_edit, 'String'));
        dv = str2double(get(d_edit, 'String'));
        if all(isfinite([nv ev dv]))
            waypoints(end+1, :) = [nv ev dv]; %#ok<AGROW>
            fmm.setMission(waypoints);
            updateWaypointUI(wp_listbox, mission_h, waypoints);
        end
        setappdata(fig, 'add_request', false);
    end

    % --- Remove last waypoint ---
    if getappdata(fig, 'pop_request')
        if ~isempty(waypoints)
            waypoints(end, :) = [];
            if isempty(waypoints), fmm.clearMission(); else, fmm.setMission(waypoints); end
            updateWaypointUI(wp_listbox, mission_h, waypoints);
        end
        setappdata(fig, 'pop_request', false);
    end

    % --- Clear all waypoints ---
    if getappdata(fig, 'clear_request')
        waypoints = zeros(0, 3);
        fmm.clearMission();
        updateWaypointUI(wp_listbox, mission_h, waypoints);
        setappdata(fig, 'clear_request', false);
    end

    % --- Mode change detection (dropdown) ---
    new_mode = mode_strings{get(mode_dd, 'Value')};
    if ~strcmp(new_mode, prev_mode)
        fmm.setMode(new_mode, plant.state());
        % Takeoff pins home to the takeoff point so RTL returns here.
        if strcmp(new_mode, 'takeoff')
            fmm.setHome(plant.pos_ned);
        end
        pos_ctl.reset();
        rate_ctl.reset();
        pos_lead.reset();
        vel_lead.reset();
        att_lead.reset();
        prev_mode = new_mode;
    end

    % --- Read sticks (each frame) ---
    sticks.left_x  = get(left_h,  'XData');
    sticks.left_y  = get(left_h,  'YData');
    sticks.right_x = get(right_h, 'XData');
    sticks.right_y = get(right_h, 'YData');

    % --- Read wind UI (cheap, once per frame) ---
    wn = str2double(get(wind_n_edit, 'String'));
    we = str2double(get(wind_e_edit, 'String'));
    wd = str2double(get(wind_d_edit, 'String'));
    if all(isfinite([wn we wd]))
        wind.steady_ned = [wn; we; wd];
    end
    wind.turb_enable = logical(get(turb_cb, 'Value'));
    sg = str2double(get(turb_sigma_edit, 'String'));
    if isfinite(sg) && sg >= 0
        wind.turb_sigma = sg;
    end

    % --- Advance physics by dt_frame in dt_rate substeps ---
    n_steps = max(1, round(dt_frame / dt_rate));
    use_est = logical(get(est_cb, 'Value'));
    for i = 1:n_steps
        s_truth = plant.state();

        % Always step the estimator so it has fresh sensor samples
        % regardless of the toggle. The toggle controls whose state
        % the controllers consume.
        est_bus.step(t_sim + (i-1)*dt_rate, s_truth);

        if use_est
            s = est_bus.stateOut();
        else
            s = s_truth;
        end

        if mod(k, n_pos) == 0
            cmd = fmm.update(s, sticks, dt_pos);
            % Mode may have auto-transitioned (Takeoff -> Hold). Reflect in
            % dropdown and reset lead state across the discontinuity.
            if ~strcmp(cmd.mode, prev_mode)
                idx = find(strcmp(mode_strings, cmd.mode), 1);
                if ~isempty(idx), set(mode_dd, 'Value', idx); end
                prev_mode = cmd.mode;
                pos_lead.reset();
                vel_lead.reset();
                att_lead.reset();
            end

            if strcmp(cmd.kind, 'attitude')
                q_sp          = cmd.q_sp;
                thrust_body_z = cmd.thrust_body_z;
                yawspeed_sp   = cmd.yawspeed_sp;
                vel_sp_used   = nan(3, 1);
            else
                % Lead-shape the position-cascade inputs. DC gain = 1, so
                % static setpoints are unchanged; only ramps/steps get
                % their phase advanced.
                if p.lead.pos.enable
                    pos_sp_in = pos_lead.update(cmd.pos_sp, dt_pos);
                else
                    pos_sp_in = cmd.pos_sp;
                end
                if p.lead.vel.enable && ~isempty(cmd.vel_sp_ff)
                    vel_sp_ff_in = vel_lead.update(cmd.vel_sp_ff, dt_pos);
                else
                    vel_sp_ff_in = cmd.vel_sp_ff;
                end
                [q_sp, thrust_body_z, vel_sp_used, ~, ~] = pos_ctl.update( ...
                    s.position_ned, s.velocity_ned, s.acceleration_ned, ...
                    pos_sp_in, cmd.yaw_sp, vel_sp_ff_in, cmd.acc_sp_ff, dt_pos);
                yawspeed_sp = cmd.yawspeed_sp;
            end

            % Lead-shape the attitude setpoint on roll/pitch. Yaw is
            % passed through to avoid +/-pi wrap discontinuities in the
            % filter state.
            if p.lead.att.enable
                rpy_sp_pre = quat_to_euler(q_sp);
                rp_lead    = att_lead.update(rpy_sp_pre(1:2), dt_pos);
                q_sp       = euler_to_quat(rp_lead(1), rp_lead(2), rpy_sp_pre(3));
            end
        end

        if mod(k, n_att) == 0
            rate_sp = att_ctl.update(s.attitude_q, q_sp, yawspeed_sp);
        end

        % Landed: ground + slow + commanded altitude target near ground.
        % Use ground truth so estimator noise doesn't cause hover flapping.
        landed = (s_truth.position_ned(3) > -0.05) && ...
                 (norm(s_truth.velocity_ned) < 0.3) && ...
                 (cmd.pos_sp(3) > -0.10);
        torque = rate_ctl.update(s.angular_vel_b, rate_sp, [0;0;0], dt_rate, landed);
        T_mag  = max(0, -thrust_body_z);
        [m, sat_pos, sat_neg] = alloc.allocate(torque, T_mag);
        rate_ctl.setSaturationStatus(sat_pos, sat_neg);
        m_last = m;

        wind_ned = wind.update(dt_rate);
        plant.step(m, dt_rate, wind_ned);

        if plant.pos_ned(3) > 0
            plant.pos_ned(3) = 0;
            if plant.vel_ned(3) > 0, plant.vel_ned(3) = 0; end
        end

        k = k + 1;
    end

    % --- Render ---
    s = plant.state();
    eN = s.position_ned(1);
    eE = s.position_ned(2);
    eU = max(0, -s.position_ned(3));
    addpoints(trail, eE, eN, eU);

    % Setpoint marker (uses the mode's pos_sp).
    eN_sp = cmd.pos_sp(1); eE_sp = cmd.pos_sp(2); eU_sp = -cmd.pos_sp(3);
    set(sp_dot,  'XData', eE_sp, 'YData', eN_sp, 'ZData', eU_sp);
    set(sp_line, 'XData', [eE eE_sp], 'YData', [eN eN_sp], 'ZData', [eU eU_sp]);

    drone = updateDrone(drone, s.position_ned, s.attitude_q, m_last, dt_frame);

    % Wind arrow: anchored ~3 m above the drone in plot frame, direction
    % is total NED wind mapped to (E, N, U). Length 0.4 m per m/s.
    w_ned   = wind.last_wind_ned;
    w_plot  = [w_ned(2); w_ned(1); -w_ned(3)];
    w_scale = 0.4;
    set(wind_arrow, ...
        'XData', eE,        'YData', eN,        'ZData', eU + 3, ...
        'UData', w_plot(1) * w_scale, ...
        'VData', w_plot(2) * w_scale, ...
        'WData', w_plot(3) * w_scale);

    % Camera follow that brackets drone + setpoint.
    half_xy = 8;
    cx = (eE + eE_sp) / 2;
    cy = (eN + eN_sp) / 2;
    pad = max(half_xy, max(abs(eE - eE_sp), abs(eN - eN_sp)) / 2 + 4);
    xlim(ax, [cx - pad, cx + pad]);
    ylim(ax, [cy - pad, cy + pad]);
    zlim(ax, [0, max(8, max(eU, eU_sp) + 4)]);

    rpy    = quat_to_euler(s.attitude_q);
    rpy_sp = quat_to_euler(q_sp);

    t_sim = t_sim + dt_frame;

    if log_idx < max_log
        log_idx = log_idx + 1;
        log.t(log_idx)          = t_sim;
        log.pos(log_idx, :)     = s.position_ned';
        log.pos_sp(log_idx, :)  = cmd.pos_sp';
        log.vel(log_idx, :)     = s.velocity_ned';
        log.vel_sp(log_idx, :)  = vel_sp_used';
        log.rpy(log_idx, :)     = rpy';
        log.rpy_sp(log_idx, :)  = rpy_sp';
        log.omega(log_idx, :)   = s.angular_vel_b';
        log.rate_sp(log_idx, :) = rate_sp';
        log.motor(log_idx, :)   = m_last';
        idx_mode = find(strcmp(mode_strings, prev_mode), 1);
        if isempty(idx_mode), idx_mode = NaN; end
        log.mode_idx(log_idx)   = idx_mode;

        % Sensor + estimator snapshot (always logged, regardless of toggle).
        imu_pub = est_bus.sensors.vehicleImu();
        if ~isempty(imu_pub)
            log.imu_gyro(log_idx, :)  = imu_pub.gyro_b';
            log.imu_accel(log_idx, :) = imu_pub.accel_b';
        end
        air_pub = est_bus.sensors.vehicleAirData();
        if ~isempty(air_pub)
            log.baro_alt(log_idx) = air_pub.altitude_m;
        end
        mag_pub = est_bus.sensors.vehicleMagnetometer();
        if ~isempty(mag_pub)
            log.mag_b(log_idx, :) = mag_pub.mag_b';
        end
        gps_pub = est_bus.sensors.vehicleGpsPosition();
        if ~isempty(gps_pub)
            log.gps_pos(log_idx, :) = gps_pub.pos_ned';
            log.gps_vel(log_idx, :) = gps_pub.vel_ned';
            log.gps_eph(log_idx)    = gps_pub.eph;
        end
        est_state = est_bus.stateOut();
        log.est_pos(log_idx, :) = est_state.position_ned';
        log.est_vel(log_idx, :) = est_state.velocity_ned';
        log.est_rpy(log_idx, :) = quat_to_euler(est_state.attitude_q)';
        log.use_est(log_idx)    = use_est;
    end

    set(state_lbl, 'String', sprintf( ...
        ['Mode    %s\n' ...
         'pos     N=%+6.2f  E=%+6.2f  Alt=%5.2f\n' ...
         'pos_sp  N=%+6.2f  E=%+6.2f  Alt=%5.2f\n' ...
         'vel     %5.2f m/s\n' ...
         'yaw     %+6.1f deg   yaw_sp %+6.1f\n' ...
         'sticks  L=(%+.2f,%+.2f)  R=(%+.2f,%+.2f)'], ...
        prev_mode, eN, eE, eU, eN_sp, eE_sp, eU_sp, ...
        norm(s.velocity_ned), rad2deg(rpy(3)), rad2deg(cmd.yaw_sp), ...
        sticks.left_x, sticks.left_y, sticks.right_x, sticks.right_y));

    drawnow limitrate;

    elapsed = toc(frame_t0);
    if elapsed < dt_frame
        pause(dt_frame - elapsed);
    end
end

if ishandle(fig)
    delete(fig);
end

% =====================================================================
% Push the log to base workspace so plot_sim() can pick it up on demand.
% No automatic plotting here.
% =====================================================================
if log_idx > 1
    fields = {'t', 'pos', 'pos_sp', 'vel', 'vel_sp', 'rpy', 'rpy_sp', ...
              'omega', 'rate_sp', 'motor', 'mode_idx', ...
              'imu_gyro', 'imu_accel', 'baro_alt', 'mag_b', ...
              'gps_pos', 'gps_vel', 'gps_eph', ...
              'est_pos', 'est_vel', 'est_rpy', 'use_est'};
    for f = fields
        log.(f{1}) = log.(f{1})(1:log_idx, :);
    end
    log.mode_strings = mode_strings;
    assignin('base', 'sim_log', log);
    fprintf(['Logged %d samples to base workspace as `sim_log`. ' ...
             'Run plot_sim() to plot.\n'], log_idx);
end
end


% =========================================================================
% Joystick UI: two square axes with a draggable handle that snaps back
% to centre on mouse-up.
% =========================================================================
function [left_h, right_h] = makeJoysticks(fig, ax_l, ax_r)
for ax = [ax_l, ax_r]
    hold(ax, 'on'); axis(ax, 'equal');
    xlim(ax, [-1.05 1.05]); ylim(ax, [-1.05 1.05]);
    set(ax, 'XTick', [], 'YTick', [], 'Box', 'on', ...
            'Color', [0.96 0.96 0.96], ...
            'XColor', [0.4 0.4 0.4], 'YColor', [0.4 0.4 0.4]);
    plot(ax, [-1 1 1 -1 -1], [-1 -1 1 1 -1], 'k-', 'LineWidth', 1.2);
    plot(ax, [-1 1], [0 0],  'Color', [0.7 0.7 0.7]);
    plot(ax, [0 0],  [-1 1], 'Color', [0.7 0.7 0.7]);
end
title(ax_l, 'Yaw  /  Throttle');
title(ax_r, 'Roll /  Pitch');

left_h  = plot(ax_l, 0, 0, 'o', 'MarkerSize', 22, ...
               'MarkerFaceColor', [0.95 0.40 0.30], 'LineWidth', 1.5);
right_h = plot(ax_r, 0, 0, 'o', 'MarkerSize', 22, ...
               'MarkerFaceColor', [0.40 0.55 0.95], 'LineWidth', 1.5);

set(ax_l,    'ButtonDownFcn', @(~,~) startDrag(fig, 'left'));
set(ax_r,    'ButtonDownFcn', @(~,~) startDrag(fig, 'right'));
set(left_h,  'ButtonDownFcn', @(~,~) startDrag(fig, 'left'));
set(right_h, 'ButtonDownFcn', @(~,~) startDrag(fig, 'right'));

set(fig, 'WindowButtonMotionFcn', @(~,~) onMotion(fig, ax_l, ax_r, left_h, right_h));
set(fig, 'WindowButtonUpFcn',     @(~,~) endDrag(fig, left_h, right_h));

setappdata(fig, 'drag_target', '');
end

function startDrag(fig, which_)
setappdata(fig, 'drag_target', which_);
end

function onMotion(fig, ax_l, ax_r, left_h, right_h)
target = getappdata(fig, 'drag_target');
if isempty(target), return; end
switch target
    case 'left'
        cp = get(ax_l, 'CurrentPoint');
        x = max(-1, min(1, cp(1, 1)));
        y = max(-1, min(1, cp(1, 2)));
        set(left_h, 'XData', x, 'YData', y);
    case 'right'
        cp = get(ax_r, 'CurrentPoint');
        x = max(-1, min(1, cp(1, 1)));
        y = max(-1, min(1, cp(1, 2)));
        set(right_h, 'XData', x, 'YData', y);
end
end

function endDrag(fig, left_h, right_h)
target = getappdata(fig, 'drag_target');
if isempty(target), return; end
switch target
    case 'left',  set(left_h,  'XData', 0, 'YData', 0);
    case 'right', set(right_h, 'XData', 0, 'YData', 0);
end
setappdata(fig, 'drag_target', '');
end


% =========================================================================
% Update the waypoint listbox + 3D-view polyline from the current Nx3
% waypoint list (NED metres).
% =========================================================================
function updateWaypointUI(listbox, mission_h, wps)
if isempty(wps)
    set(listbox, 'String', {}, 'Value', 1);
    set(mission_h, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
    return;
end
n = size(wps, 1);
strs = cell(n, 1);
for i = 1:n
    strs{i} = sprintf('W%-2d  N=%+7.2f  E=%+7.2f  D=%+7.2f', ...
                      i, wps(i, 1), wps(i, 2), wps(i, 3));
end
set(listbox, 'String', strs, 'Value', max(1, min(n, get(listbox, 'Value'))));
% 3D view: plot in (East, North, Up).
set(mission_h, 'XData', wps(:, 2), 'YData', wps(:, 1), 'ZData', -wps(:, 3));
end


% =========================================================================
% Drone visual: STL mesh body + four spinning propellers (hgtransform-based).
% Rotor mounts auto-detected from the body STL by quadrant clustering.
% Frame mapping STL -> FRD: (X, Y, Z) -> (X, Z, Y).
% =========================================================================
function drone = makeDrone(ax, root)
arm = 1.2;

% --- Body STL ---
[bV, bF] = readStlAny(fullfile(root, 'QuadCopter_Body.stl'));
bV = bV - mean(bV, 1);

horiz_r = sqrt(bV(:, 1).^2 + bV(:, 3).^2);
mask_outer = horiz_r > quantile(horiz_r, 0.85);
quads = [+1 +1; -1 +1; +1 -1; -1 -1];
mounts_raw = zeros(4, 3);
for i = 1:4
    m = mask_outer & (sign(bV(:, 1)) == quads(i, 1)) ...
                   & (sign(bV(:, 3)) == quads(i, 2));
    if any(m)
        mounts_raw(i, :) = mean(bV(m, :), 1);
    else
        mounts_raw(i, :) = [quads(i, 1)*max(abs(bV(:,1))), 0, ...
                            quads(i, 2)*max(abs(bV(:,3)))];
    end
end

bV          = swapToFRD(bV);
mounts_frd  = swapToFRD(mounts_raw);

horiz_extent = max(max(abs(bV(:, 1:2)), [], 1));
scale = arm / max(horiz_extent, eps);
bV          = bV * scale;
mounts_frd  = mounts_frd * scale;

if size(bF, 1) > 12000
    fv = reducepatch(struct('faces', bF, 'vertices', bV), 12000);
    bV = fv.vertices; bF = fv.faces;
end

drone.body_xform = hgtransform('Parent', ax);
drone.body_h = patch('Parent', drone.body_xform, ...
                     'Faces', bF, 'Vertices', bV, ...
                     'FaceColor', [0.40 0.42 0.48], 'EdgeColor', 'none', ...
                     'FaceLighting', 'gouraud', 'AmbientStrength', 0.4);

% --- Propeller STL (shared by all 4 props) ---
[pV, pF] = readStlAny(fullfile(root, 'QuadCopter_Propeller.stl'));
pV = pV - mean(pV, 1);
pV = swapToFRD(pV);
prop_horiz = max(max(abs(pV(:, 1:2)), [], 1));
pV = pV * (arm * 0.45 / max(prop_horiz, eps));

drone.prop_xform   = gobjects(1, 4);
for i = 1:4
    drone.prop_xform(i) = hgtransform('Parent', drone.body_xform);
    patch('Parent', drone.prop_xform(i), 'Faces', pF, 'Vertices', pV, ...
          'FaceColor', [0.10 0.10 0.12], 'EdgeColor', 'none', ...
          'FaceLighting', 'gouraud', 'AmbientStrength', 0.5);
end
drone.rotors_body  = mounts_frd';
drone.prop_angle   = zeros(1, 4);
drone.prop_spin_dir = [+1 -1 +1 -1];
drone.prop_spin_max = 250;

drone.front = line('Parent', drone.body_xform, ...
                   'XData', [0, 1.4*arm], 'YData', [0, 0], 'ZData', [0, 0], ...
                   'Color', 'r', 'LineWidth', 3.5);

if isempty(findobj(ax, 'Type', 'light'))
    camlight(ax, 'headlight');
    lighting(ax, 'gouraud');
end
end


function drone = updateDrone(drone, pos_ned, q, motor_cmd, dt)
ned_to_plot = [0 1 0; 1 0 0; 0 0 -1];
R_b2n = quat_to_dcm(q);
M_body = eye(4);
M_body(1:3, 1:3) = ned_to_plot * R_b2n;
M_body(1:3, 4)   = ned_to_plot * pos_ned;
set(drone.body_xform, 'Matrix', M_body);

for i = 1:4
    drone.prop_angle(i) = drone.prop_angle(i) + ...
        drone.prop_spin_dir(i) * drone.prop_spin_max * sqrt(max(0, motor_cmd(i))) * dt;
    a = drone.prop_angle(i);
    M_prop = eye(4);
    M_prop(1:3, 1:3) = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
    M_prop(1:3, 4)   = drone.rotors_body(:, i);
    set(drone.prop_xform(i), 'Matrix', M_prop);
end
end


function V = swapToFRD(V)
V = [V(:, 1), V(:, 3), V(:, 2)];
end


function [V, F] = readStlAny(path)
% Cross-version STL loader. MATLAB's built-in stlread (R2018b+) returns a
% triangulation object with Points / ConnectivityList; older toolboxes
% (and the File Exchange version) return a struct with vertices / faces.
s = stlread(path);
if isa(s, 'triangulation')
    V = s.Points;
    F = s.ConnectivityList;
else
    V = s.vertices;
    F = s.faces;
end
end
