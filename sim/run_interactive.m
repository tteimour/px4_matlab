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
% UI ("flight deck", dark night-ops theme):
%   * Top telemetry strip: flight-mode badge, ALT / V/S / GS / HDG / N / E
%     readouts, EKF-GPS-LINK health pills, mission clock.
%   * FLIGHT tab: satellite mission map (click to add waypoints) + an
%     attitude indicator with pitch ladder and a heading tape, waypoint
%     table, .plan save/load.
%   * Global bottom bar: flight-mode buttons (work from ANY tab),
%     Set Home, Reset, Stop.
%   * Stick console (separate floating window, flies from any tab):
%       LEFT  = yaw (X) + throttle (Y)
%       RIGHT = roll (X) + pitch (Y, stick forward = nose down)
%   * Colors follow the avionics conventions of FAA AC 25-11B: red =
%     warning, amber = caution, green = normal/engaged, cyan = sky,
%     tan = ground, magenta = active navigation reference.
%
% On Stop, all logged tracking signals are plotted in their own figures
% and then `format_all_figures()` / `vertical_line()` are called.

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(fullfile(root, 'src', 'params'));
addpath(fullfile(root, 'src', 'math'));
addpath(fullfile(root, 'src', 'plant'));
addpath(fullfile(root, 'src', 'controllers'));
addpath(fullfile(root, 'src', 'autotune'));
addpath(fullfile(root, 'src', 'navigator'));
addpath(fullfile(root, 'src', 'flight_modes'));
addpath(fullfile(root, 'src', 'sensors'));
addpath(fullfile(root, 'src', 'sensors', 'imu'));
addpath(fullfile(root, 'src', 'sensors', 'baro'));
addpath(fullfile(root, 'src', 'sensors', 'mag'));
addpath(fullfile(root, 'src', 'sensors', 'gnss'));
addpath(fullfile(root, 'src', 'sensors', 'voter'));
addpath(fullfile(root, 'src', 'estimator'));
addpath(fullfile(root, 'src', 'bridge'));

p = px4_params();

% =====================================================================
% Tabbed GCS GUI ("flight deck"). The 3D scene was removed -- Unity/Cesium
% now provides the visualization, so the tabs are parameter editors plus
% the FLIGHT tab (mission map, live 2D trails, attitude/heading
% instruments). A telemetry strip (top) and the
% flight-mode bar (bottom) are global, outside the tabs. Manual control
% lives in a separate floating window so you can fly from any tab.
% =====================================================================
T = gcsTheme();

fig = figure('Name', 'PX4 GCS — Flight Deck (MATLAB SIL)', ...
             'NumberTitle', 'off', 'Color', T.bg, ...
             'MenuBar', 'none', 'ToolBar', 'none', ...
             'Position', [80 60 1500 920]);
applyThemeDefaults(fig, T);

strip = buildStatusStrip(fig);   % top telemetry strip (updated per frame)

tg = uitabgroup(fig, 'Units', 'normalized', 'Position', [0 0.088 1 0.842]);
tab_mission = uitab(tg, 'Title', '  FLIGHT  ',     'BackgroundColor', T.panel);
tab_ctrl = uitab(tg, 'Title', '  CONTROLLER  ',    'BackgroundColor', T.panel);
tab_ekf  = uitab(tg, 'Title', '  EKF  ',           'BackgroundColor', T.panel);
tab_sens = uitab(tg, 'Title', '  SENSORS  ',       'BackgroundColor', T.panel);
tab_wind = uitab(tg, 'Title', '  WIND  ',          'BackgroundColor', T.panel);
tab_auto = uitab(tg, 'Title', '  AUTOTUNE  ',      'BackgroundColor', T.panel);
tab_cesium = uitab(tg, 'Title', '  CESIUM  ',      'BackgroundColor', T.panel);

mode_strings = {'stabilized', 'altitude', 'position', 'hold', ...
                'mission', 'rtl', 'land', 'takeoff'};
default_mode_idx = 3;             % start in Position

% Global bottom bar: flight-mode buttons + Arm / Set Home / Reset / Stop,
% usable from any tab. The sim loop polls mode_bg's SelectedObject as before.
[mode_bg, arm_btn] = buildModeBar(fig, mode_strings);

% =====================================================================
% Manual control lives in a SEPARATE floating window (ctrl_fig) so you can
% fly the drone from ANY tab -- the joysticks used to sit on the 3D tab and
% vanished when you switched tabs. The drag callbacks are wired to ctrl_fig;
% the sim loop reads left_h/right_h regardless of which figure holds them.
% Reset/Stop are here too (they act on the main figure `fig`). Closing this
% window stops the sim.
% =====================================================================
ctrl_fig = figure('Name', 'Stick console — flies from any tab', ...
    'NumberTitle', 'off', 'MenuBar', 'none', 'ToolBar', 'none', ...
    'Color', T.bg, 'Position', [60 90 520 330], ...
    'CloseRequestFcn', @(~,~) ctrl_fig_close(fig, ctrl_fig));
applyThemeDefaults(ctrl_fig, T);
ax_left = axes('Parent', ctrl_fig, 'Units', 'normalized', ...
               'Position', [0.06 0.30 0.40 0.64]);
ax_right = axes('Parent', ctrl_fig, 'Units', 'normalized', ...
                'Position', [0.54 0.30 0.40 0.64]);
[left_h, right_h] = makeJoysticks(ctrl_fig, ax_left, ax_right);

uicontrol(ctrl_fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.06 0.06 0.40 0.16], 'String', 'RESET', ...
    'FontWeight', 'bold', 'BackgroundColor', T.btn, 'ForegroundColor', T.warn, ...
    'Callback', @(~,~) setappdata(fig, 'reset_request', true));
uicontrol(ctrl_fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.54 0.06 0.40 0.16], 'String', 'STOP', ...
    'FontWeight', 'bold', 'BackgroundColor', T.badbg, 'ForegroundColor', T.bad, ...
    'Callback', @(~,~) setappdata(fig, 'running', false));
figure(fig);   % bring the main tabbed window back to the front

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

% --- Arming + smooth takeoff (PX4 commander / mc_pos_control Takeoff) ---
% The vehicle spawns DISARMED: motors stay stopped and the position
% controller is held in reset until the pilot arms (ARM button, or
% auto-arm on a Takeoff command). Thrust then ramps over MPC_TKO_RAMP_T
% (Takeoff.cpp; MulticopterPositionControl.cpp:496-531).
tko = TakeoffHandling(p);
armed         = false;
was_airborne  = false;  % true once actually flying; enables auto-disarm
landed_latch  = true;   % landed state from the previous substep
landed_since  = -1;     % t_sim when post-flight landing was first seen
prev_armed_ui = [];     % ARM button / pill restyle guard

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

% Known body-frame bias injected into every IMU so the EKF has a
% concrete truth to estimate. Edit these to test the convergence
% behaviour. Realistic bias magnitudes are on the order of a few
% hundredths of a rad/s for gyros and a few tenths of m/s² for accels.
true_gyro_bias  = [ 0.000; -0.00;  0.00];   % rad/s
true_accel_bias = [ 0.0;  -0.0;   0.0 ];   % m/s²
est_bus.sensors.applyImuBias(true_gyro_bias, true_accel_bias);

% Wind disturbance: steady NED component + first-order turbulence.
wind = WindModel(p);

% =====================================================================
% Parameter tabs (all bind directly to the live module objects, so edits
% take effect on the next sim tick / fusion).
% =====================================================================
ctrl_refresh = buildControllerTab(tab_ctrl, p, pos_ctl, att_ctl, rate_ctl);
est_cb       = buildEkfTab(tab_ekf, est_bus);
buildSensorTab(tab_sens, est_bus);
buildWindTab(tab_wind, p, wind);
cesium_cb = buildCesiumTab(tab_cesium);

% Flight tab: north-up satellite map at the Cesium origin (Baku); click to
% drop waypoints, set per-waypoint altitude. Writes the same `waypoints`
% array (via map_add_request / alt_edit_request) the cockpit editor uses.
% Also hosts the attitude/heading instruments (pfd), updated per frame.
mission_map = buildMissionTab(tab_mission, fig, fmm);
mission_map.mode_bg = mode_bg;       % setModeButton() selects + restyles via this
state_lbl = mission_map.state_lbl;   % live telemetry text on the Flight tab
pfd = mission_map.pfd;               % attitude indicator + heading tape updater

% --- System-identification autotuner (runs live in the sim loop) -------
% Preconditions (airborne, low speed, position/hold mode, sticks centred)
% are checked at start time. The tuner consumes a *modeled* gyro = ground
% truth + ICM-45686 thermal noise + motor-vibration noise, so the
% recursive-least-squares stays well-conditioned (PX4 relies on the real
% gyro's broadband content for the same reason; a noise-free rate makes
% the AR part unobservable). On success the identified gains are applied
% to the live controllers AND mirrored onto the Controller tab.
at_opts = struct('apply_mode', 0, 'gyro_cutoff', 40.0, ...
                 'sysid_amp', 0.7, 'rise_time', 0.14, 'log_enable', false);
at = McAutotuneAttitudeControl(p, at_opts);
autotune_active = false;
autotune_status = 'idle';
% Read the gyro-noise model from the live primary IMU chip so the Sensors
% tab also influences the tuner (recomputed at each Start).
imu0          = est_bus.sensors.imu.sensors{1};
gyro_sig_th   = imu0.gyro_nd / sqrt(1 / p.rate_hz.rate);
gyro_vib_gain = imu0.gyro_vib_gain;
[at_btn, amp_edit, rise_edit, at_status_lbl, at_results_lbl] = ...
    buildAutotuneTab(tab_auto, fig, at_opts);

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
% Logging (optional, OFF by default for performance).
%
%   LOG_ENABLE = true  -> record tracking / sensor / estimator signals into
%                         `sim_log` (base workspace) so plot_sim() can plot.
%   LOG_ENABLE = false -> skip ALL per-frame logging work. Default. Keeps the
%                         sim loop light so the Cesium/Unity pose stream stays
%                         smooth. Flip this single flag to turn logging on/off.
% =====================================================================
LOG_ENABLE = false;       % 1 = log to base workspace, 0 = no logging (default)

max_log = 60000;
log_idx = 0;
log     = struct();
if LOG_ENABLE
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
    % Sensor + estimator logs.
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
    % IMU bias estimation logs.
    log.true_gyro_bias  = nan(max_log, 3);   % live body-frame bias of primary IMU
    log.true_accel_bias = nan(max_log, 3);
    log.est_gyro_bias   = nan(max_log, 3);   % EKF state.gyro_b
    log.est_accel_bias  = nan(max_log, 3);   % EKF state.accel_b
end

setappdata(fig, 'running', true);
setappdata(fig, 'reset_request', false);
setappdata(fig, 'clear_request', false);
setappdata(fig, 'pop_request', false);
setappdata(fig, 'set_home_request', false);
setappdata(fig, 'arm_request', false);
setappdata(fig, 'autotune_request', false);
setappdata(fig, 'map_add_request', []);         % [N E D] from a Mission-map click
setappdata(fig, 'alt_edit_request', []);        % [row alt_m] from the Mission table
setappdata(fig, 'load_plan_request', []);       % struct(waypoints,lat0,lon0) from a .plan
setappdata(fig, 'save_plan_request', false);    % Save-.plan button
waypoints = zeros(0, 3);                        % Nx3 [N E D]
sticks = struct('left_x', 0, 'left_y', 0, 'right_x', 0, 'right_y', 0);

k     = 0;
t_sim = 0;
ui_tick = 0;           % strip/readout refresh decimation (every 2nd frame)
cesium_bridge  = [];   % lazily created when the Cesium toggle is first enabled

% Auto-launch rosbridge_server for the Unity/Cesium path. onCleanup guarantees
% it is stopped on any exit (Stop, window close, or an error in the loop).
rosbridge_pid = startRosbridge();
cleanup_rosbridge = onCleanup(@() stopRosbridge(rosbridge_pid));

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
        % Full estimator reset: sensors (clocks/queues/validators), EKF,
        % output predictor, and the staleness trackers — so post-Reset sim time
        % restarts cleanly at 0 and fresh IMU samples flow again (fixes the
        % pre-existing estimator stall on Reset).
        est_bus.reset();
        % Re-inject the known bias after reset so post-Reset runs
        % have the same truth bias to estimate.
        est_bus.sensors.applyImuBias(true_gyro_bias, true_accel_bias);
        t_sim   = 0;
        clearpoints(mission_map.gt_trail);  clearpoints(mission_map.ekf_trail);
        log_idx = 0;
        fmm.setHome([0; 0; 0]);
        fmm.setMode(prev_mode, plant.state());
        armed = false; was_airborne = false; landed_since = -1;
        landed_latch = true; tko.reset();
        setappdata(fig, 'reset_request', false);
    end

    % --- Set Home (manual override) ---
    if getappdata(fig, 'set_home_request')
        fmm.setHome(plant.pos_ned);
        setappdata(fig, 'set_home_request', false);
    end

    % --- Arm / Disarm toggle (bottom-bar button) ---
    if getappdata(fig, 'arm_request')
        setappdata(fig, 'arm_request', false);
        armed = ~armed;
        if armed
            was_airborne = false; landed_since = -1;
            fprintf('ARMED (spoolup %.1f s).\n', p.com.spoolup_time);
        else
            fprintf('DISARMED.\n');
        end
    end

    % --- Autotune start/cancel (preconditions enforced here) ---
    if getappdata(fig, 'autotune_request')
        setappdata(fig, 'autotune_request', false);
        if autotune_active
            autotune_active = false;            % cancel -> stop injecting
            autotune_status = 'autotune cancelled';
        else
            st = plant.state();                 % ground truth for gating
            airborne = (-st.position_ned(3)) > 1.5;
            slow     = norm(st.velocity_ned) < 0.6;
            mode_ok  = any(strcmp(prev_mode, {'position', 'hold'}));
            centred  = (abs(sticks.right_x) < 0.05) && (abs(sticks.right_y) < 0.05);
            if airborne && slow && mode_ok && centred
                % Pull amplitude / rise-time from the Autotune tab.
                av = str2double(get(amp_edit, 'String'));
                rv = str2double(get(rise_edit, 'String'));
                if isfinite(av) && av > 0, at_opts.sysid_amp = av; end
                if isfinite(rv) && rv > 0, at_opts.rise_time = rv; end
                % Re-read the gyro-noise model from the (possibly edited)
                % primary IMU chip so the Sensors tab influences the tuner.
                gyro_sig_th   = imu0.gyro_nd / sqrt(dt_rate);
                gyro_vib_gain = imu0.gyro_vib_gain;
                at = McAutotuneAttitudeControl(p, at_opts);   % fresh run
                at.start();
                autotune_active = true;
                autotune_status = 'running: roll...';
            else
                reasons = {};
                if ~mode_ok,  reasons{end+1} = 'use position/hold'; end %#ok<AGROW>
                if ~airborne, reasons{end+1} = 'climb >1.5m';       end %#ok<AGROW>
                if ~slow,     reasons{end+1} = 'hold still';        end %#ok<AGROW>
                if ~centred,  reasons{end+1} = 'centre sticks';     end %#ok<AGROW>
                autotune_status = ['autotune blocked: ' strjoin(reasons, ', ')];
            end
        end
    end

    % --- Add waypoint (Mission-tab map click: [N E D]) ---
    map_add = getappdata(fig, 'map_add_request');
    if ~isempty(map_add)
        waypoints(end+1, :) = map_add; %#ok<AGROW>
        fmm.setMission(waypoints);
        updateMissionMap(mission_map, waypoints);
        setappdata(fig, 'map_add_request', []);
    end

    % --- Edit a waypoint altitude (Mission table: [row alt_m]) ---
    alt_edit = getappdata(fig, 'alt_edit_request');
    if ~isempty(alt_edit)
        row = alt_edit(1);
        if row >= 1 && row <= size(waypoints, 1) && isfinite(alt_edit(2))
            waypoints(row, 3) = -alt_edit(2);   % D = -altitude (positive = up)
            fmm.setMission(waypoints);
                updateMissionMap(mission_map, waypoints);
        end
        setappdata(fig, 'alt_edit_request', []);
    end

    % --- Load a QGC .plan (re-anchors the map on the plan's home) ---
    load_plan = getappdata(fig, 'load_plan_request');
    if ~isempty(load_plan)
        setMissionAnchor(mission_map, load_plan.lat0, load_plan.lon0);
        waypoints = load_plan.waypoints;
        if isempty(waypoints), fmm.clearMission(); else, fmm.setMission(waypoints); end
        updateMissionMap(mission_map, waypoints);
        setappdata(fig, 'load_plan_request', []);
    end

    % --- Save the current waypoints as a QGC .plan ---
    if getappdata(fig, 'save_plan_request')
        savePlanDialog(mission_map, waypoints);
        setappdata(fig, 'save_plan_request', false);
    end

    % --- Remove last waypoint ---
    if getappdata(fig, 'pop_request')
        if ~isempty(waypoints)
            waypoints(end, :) = [];
            if isempty(waypoints), fmm.clearMission(); else, fmm.setMission(waypoints); end
                updateMissionMap(mission_map, waypoints);
        end
        setappdata(fig, 'pop_request', false);
    end

    % --- Clear all waypoints ---
    if getappdata(fig, 'clear_request')
        waypoints = zeros(0, 3);
        fmm.clearMission();
        updateMissionMap(mission_map, waypoints);
        setappdata(fig, 'clear_request', false);
    end

    % --- Mode change detection (big mode buttons on the Mission tab) ---
    sel_mode_btn = get(mode_bg, 'SelectedObject');
    if isempty(sel_mode_btn), new_mode = prev_mode;
    else,                     new_mode = get(sel_mode_btn, 'Tag'); end
    if ~strcmp(new_mode, prev_mode)
        fmm.setMode(new_mode, plant.state());
        % Takeoff pins home to the takeoff point so RTL returns here, and
        % auto-arms (QGC sends an arm with the takeoff command).
        if strcmp(new_mode, 'takeoff')
            fmm.setHome(plant.pos_ned);
            if ~armed
                armed = true; was_airborne = false; landed_since = -1;
                fprintf('Auto-armed by Takeoff (spoolup %.1f s).\n', ...
                        p.com.spoolup_time);
            end
        end
        pos_ctl.reset();
        rate_ctl.reset();
        pos_lead.reset();
        vel_lead.reset();
        att_lead.reset();
        prev_mode = new_mode;
    end

    % --- Read sticks (each frame; guard if the control window was closed) ---
    if ishandle(left_h) && ishandle(right_h)
        sticks.left_x  = get(left_h,  'XData');
        sticks.left_y  = get(left_h,  'YData');
        sticks.right_x = get(right_h, 'XData');
        sticks.right_y = get(right_h, 'YData');
    end

    % Wind parameters are set directly by the Wind tab callbacks.

    % --- Advance physics by dt_frame in dt_rate substeps ---
    n_steps = max(1, round(dt_frame / dt_rate));
    use_est = logical(get(est_cb, 'Value'));
    % Cesium/Unity link state. Read once per frame and surfaced on the status
    % strip as the LINK pill.
    stream_on = ishandle(cesium_cb) && get(cesium_cb, 'Value') == 1;
    for i = 1:n_steps
        s_truth = plant.state();

        % vib_level scales motor / prop vibration into the IMU model.
        % norm(m_last) is ~0 idle, ~1 hover (4 motors at ~0.5), ~1.8 full
        % thrust. Sensors look it up in their measure() override.
        s_truth.vib_level = norm(m_last);

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
            % the mode buttons and reset lead state across the discontinuity.
            if ~strcmp(cmd.mode, prev_mode)
                setModeButton(mission_map, cmd.mode);
                prev_mode = cmd.mode;
                pos_lead.reset();
                vel_lead.reset();
                att_lead.reset();
            end

            % --- Smooth-takeoff state machine -------------------------------
            % PX4 Takeoff.cpp + MulticopterPositionControl.cpp:463-531: the
            % vehicle produces zero thrust until armed + spooled up + a climb
            % is wanted; then the upward-speed limit ramps from the zero-
            % thrust value to the configured climb limit over MPC_TKO_RAMP_T.
            if strcmp(cmd.kind, 'position')
                climb_cmd = (isfinite(cmd.pos_sp(3)) && ...
                             cmd.pos_sp(3) < s.position_ned(3) - 0.1) || ...
                            (~isempty(cmd.vel_sp_ff) && ...
                             isfinite(cmd.vel_sp_ff(3)) && cmd.vel_sp_ff(3) < -0.05);
            else
                climb_cmd = sticks.left_y > 0.05;   % manual throttle raised
            end
            want_takeoff = armed && climb_cmd;
            tko.generateInitialRampValue(pos_ctl.gain_vel_p(3));
            tko.updateTakeoffState(armed, landed_latch, want_takeoff, dt_pos);
            flying = tko.state >= TakeoffHandling.FLIGHT;
            speed_up = tko.updateRamp(dt_pos, pos_ctl.lim_vel_up);
            if flying
                pos_ctl.setRuntimeLimits(speed_up, [], []);
            else
                % zero minimum thrust + landing tilt limit until airborne
                % (MulticopterPositionControl.cpp:519-530)
                pos_ctl.setRuntimeLimits(speed_up, 0.0, p.pos.tilt_max_lnd);
            end
            if tko.state < TakeoffHandling.RAMPUP
                % Not flying yet: empty trajectory setpoint with a high
                % downward acceleration so thrust is exactly zero, and the
                % position-loop integrator held in reset
                % (MulticopterPositionControl.cpp:509-517).
                pos_ctl.reset();
                cmd.kind      = 'position';
                cmd.pos_sp    = nan(3, 1);
                cmd.vel_sp_ff = nan(3, 1);
                cmd.acc_sp_ff = [0; 0; 100];
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

        % Autotune excitation: add the injected rate setpoint (held between
        % the tuner's 100 ms publishes) on top of the attitude-loop output.
        if autotune_active
            inj = at.injection();
        else
            inj = [0; 0; 0];
        end
        rate_sp_cmd = rate_sp + inj;

        % Landed: ground + slow + commanded altitude target near ground (or
        % pre-takeoff, where motors are stopped by construction).
        % Use ground truth so estimator noise doesn't cause hover flapping.
        landed = (s_truth.position_ned(3) > -0.05) && ...
                 (norm(s_truth.velocity_ned) < 0.3) && ...
                 ((isfinite(cmd.pos_sp(3)) && cmd.pos_sp(3) > -0.10) || ...
                  tko.state < TakeoffHandling.RAMPUP);
        landed_latch = landed;
        if ~landed, was_airborne = true; end
        torque = rate_ctl.update(s.angular_vel_b, rate_sp_cmd, [0;0;0], dt_rate, landed);
        T_mag  = max(0, -thrust_body_z);
        [m, sat_pos, sat_neg] = alloc.allocate(torque, T_mag);
        rate_ctl.setSaturationStatus(sat_pos, sat_neg);
        m_last = m;

        % Motors stopped while disarmed / spooling / waiting for takeoff
        % (PX4: actuators disarmed below RAMPUP; ControlAllocator.cpp:
        % 336-374, MulticopterPositionControl.cpp:530).
        if tko.state < TakeoffHandling.RAMPUP
            m = zeros(4, 1);
            m_last = m;
        end

        % Feed the tuner the torque it produced + a modeled (noisy) gyro;
        % a roll/pitch stick deflection aborts the run inside step().
        if autotune_active
            omega_meas = s_truth.angular_vel_b ...
                + gyro_sig_th * randn(3, 1) ...
                + gyro_vib_gain * norm(m_last) * randn(3, 1);
            at.step(dt_rate, torque, omega_meas, ~landed, ...
                    [sticks.right_x; sticks.right_y]);
        end

        wind_ned = wind.update(dt_rate);
        plant.step(m, dt_rate, wind_ned);

        if plant.pos_ned(3) > 0
            plant.pos_ned(3) = 0;
            if plant.vel_ned(3) > 0, plant.vel_ned(3) = 0; end
        end

        k = k + 1;

        % Yield to MATLAB's event queue mid-frame so joystick clicks and
        % mouse-motion callbacks fire promptly. limitrate self-throttles
        % the actual render so this stays cheap.
        if mod(i, 4) == 0
            drawnow limitrate;
        end
    end

    % --- Flight phase to the estimator + auto-disarm after landing -------
    % in_air gates mag-3D tilt updates and gravity-fusion at-rest handling
    % (PX4 commander/land detector -> ekf2 control flags). COM_DISARM_LAND
    % auto-disarms a few seconds after a post-flight landing.
    est_bus.setInAir(~landed_latch);
    if armed && was_airborne && landed_latch
        if landed_since < 0, landed_since = t_sim; end
        if t_sim - landed_since > p.com.disarm_land
            armed = false; was_airborne = false; landed_since = -1;
            fprintf('Auto-disarmed %.1f s after landing (COM_DISARM_LAND).\n', ...
                    p.com.disarm_land);
        end
    else
        landed_since = -1;
    end
    if ~isequal(prev_armed_ui, armed)
        prev_armed_ui = armed;
        if armed
            set(arm_btn, 'String', 'DISARM', 'ForegroundColor', T.warn);
        else
            set(arm_btn, 'String', 'ARM', 'ForegroundColor', T.good);
        end
    end

    % --- Autotune lifecycle: report progress, apply gains on success ---
    if autotune_active
        autotune_status = sprintf('running: %s', at.stateName());
        if at.isDone()
            r = at.getResults();
            if r.success
                applyTunedGains(rate_ctl, att_ctl, r);
                ctrl_refresh();   % mirror the new gains onto the Controller tab
                autotune_status = sprintf( ...
                    'DONE — gains applied (rate P r/p/y = %.3f/%.3f/%.3f)', ...
                    r.rate_k(1), r.rate_k(2), r.rate_k(3));
            else
                autotune_status = 'FAILED — gains unchanged';
            end
            set(at_results_lbl, 'String', autotuneResultText(r));
            autotune_active = false;
        end
    end
    if autotune_active
        set(at_btn, 'String', 'Cancel Autotune', ...
            'BackgroundColor', T.warnbg, 'ForegroundColor', T.warn);
    else
        set(at_btn, 'String', 'Start Autotune', ...
            'BackgroundColor', T.goodbg, 'ForegroundColor', T.good);
    end
    set(at_status_lbl, 'String', ['autotune: ' autotune_status]);

    % --- State for the readout + bridges (no in-GUI 3D view; Unity renders) ---
    % The strip and instruments display the CONTROLLER-FEED state (the
    % vehicle's own estimate when the EKF feed is on) — what a real GCS
    % telemeters. The Cesium/Unity bridge keeps publishing ground truth.
    s_disp = s;                       % last substep's controller-feed state
    s = plant.state();
    eN = s_disp.position_ned(1);
    eE = s_disp.position_ned(2);
    eU = max(0, -s_disp.position_ned(3));
    eN_sp = cmd.pos_sp(1); eE_sp = cmd.pos_sp(2); eU_sp = -cmd.pos_sp(3);

    % --- Stream pose to Cesium/Unity (opt-in via the Cesium tab) ----------
    % Ground-truth pose, converted NED/FRD->ENU/FLU
    % and published as geometry_msgs/PoseArray. Failures disable the toggle
    % rather than killing the sim.
    if ishandle(cesium_cb) && get(cesium_cb, 'Value') == 1
        if isempty(cesium_bridge)
            try
                % Windows (MATLAB + rosbridge in WSL2) can't do cross-boundary
                % DDS, so use the WebSocket transport there; native DDS on Linux.
                if ispc
                    cesium_bridge = CesiumBridgeWs();
                else
                    cesium_bridge = CesiumBridge();
                end
                fprintf('Cesium bridge: publishing to %s\n', cesium_bridge.Topic);
            catch ME
                warning('Cesium bridge failed to start (%s). Disabling.', ME.message);
                set(cesium_cb, 'Value', 0);
                cesium_bridge = [];
            end
        end
        if ~isempty(cesium_bridge)
            try
                cesium_bridge.publish(s.position_ned, s.attitude_q, t_sim);
            catch ME
                warning('Cesium bridge publish failed (%s). Disabling.', ME.message);
                set(cesium_cb, 'Value', 0);
                delete(cesium_bridge);
                cesium_bridge = [];
            end
        end
    end

    % --- live 2D map trails (Mission tab), always-on ----------------------
    % Ground truth (white) and EKF estimate (cyan) appended every frame so the
    % Mission map shows the flown path regardless of any external aiding.
    gtn  = plant.state();
    estn = est_bus.stateOut();
    addpoints(mission_map.gt_trail,  gtn.position_ned(2),  gtn.position_ned(1));
    addpoints(mission_map.ekf_trail, estn.position_ned(2), estn.position_ned(1));

    rpy    = quat_to_euler(s.attitude_q);        % ground truth (for logging)
    rpy_d  = quat_to_euler(s_disp.attitude_q);   % displayed (estimate feed)
    rpy_sp = quat_to_euler(q_sp);

    t_sim = t_sim + dt_frame;

    if LOG_ENABLE && log_idx < max_log
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

        % Sensor + estimator snapshot (logged independent of the EKF-feed toggle).
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

        % IMU bias estimation: truth comes from the currently-voted
        % primary IMU (so it tracks bias-random-walk drift), estimate
        % comes straight from the EKF state.
        [tg, ta] = est_bus.sensors.primaryImuTrueBias();
        log.true_gyro_bias(log_idx, :)  = tg';
        log.true_accel_bias(log_idx, :) = ta';
        log.est_gyro_bias(log_idx, :)   = est_bus.ekf.gyro_b';
        log.est_accel_bias(log_idx, :)  = est_bus.ekf.accel_b';
    end

    % --- Telemetry strip + instruments + Flight-tab readout ---------------
    % The attitude instruments refresh every frame (cheap line/transform
    % updates); the uicontrol text strip refreshes at half rate to keep the
    % loop light (uicontrol String sets are the expensive part).
    hdg = mod(rad2deg(rpy_d(3)), 360);
    pfd.update(rpy_d(1), rpy_d(2), hdg);
    ui_tick = ui_tick + 1;
    if mod(ui_tick, 2) == 0
        vs  = -s_disp.velocity_ned(3);            % climb rate, +up
        gs  = norm(s_disp.velocity_ned(1:2));     % ground speed
        updateStatusStrip(strip, prev_mode, eU, vs, gs, hdg, eN, eE, t_sim, ...
                          armed, use_est, stream_on);
        set(state_lbl, 'String', sprintf( ...
            ['TGT    N %s   E %s   ALT %s\n' ...
             'YAW    %+6.1f deg     CMD %+6.1f deg\n' ...
             'STICKS L %+5.2f %+5.2f   R %+5.2f %+5.2f\n' ...
             'TUNE   %s'], ...
            n2s(eN_sp, '%+7.2f'), n2s(eE_sp, '%+7.2f'), n2s(eU_sp, '%6.2f'), ...
            rad2deg(rpy_d(3)), rad2deg(cmd.yaw_sp), ...
            sticks.left_x, sticks.left_y, sticks.right_x, sticks.right_y, ...
            autotune_status));
    end

    drawnow limitrate;

    elapsed = toc(frame_t0);
    if elapsed < dt_frame
        pause(dt_frame - elapsed);
    end
end

% Tear down the Cesium/Unity ROS 2 node if it was started.
if ~isempty(cesium_bridge) && isvalid(cesium_bridge)
    delete(cesium_bridge);
end
% rosbridge_server is stopped by the onCleanup guard registered at start.

if exist('ctrl_fig', 'var') && ishandle(ctrl_fig)
    delete(ctrl_fig);            % close the floating manual-control window
end
if ishandle(fig)
    delete(fig);
end

% =====================================================================
% Push the log to base workspace so plot_sim() can pick it up on demand.
% No automatic plotting here.
% =====================================================================
if LOG_ENABLE && log_idx > 1
    fields = {'t', 'pos', 'pos_sp', 'vel', 'vel_sp', 'rpy', 'rpy_sp', ...
              'omega', 'rate_sp', 'motor', 'mode_idx', ...
              'imu_gyro', 'imu_accel', 'baro_alt', 'mag_b', ...
              'gps_pos', 'gps_vel', 'gps_eph', ...
              'est_pos', 'est_vel', 'est_rpy', 'use_est', ...
              'true_gyro_bias', 'true_accel_bias', ...
              'est_gyro_bias', 'est_accel_bias'};
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
% Apply autotune results to the live controllers. The tuner reports gains
% in standard form (kc, ki, kd); RateController holds effective parallel-
% form gains (with MC_*RATE_K = 1): P = kc, I = kc*ki, D = kc*kd. The
% attitude P uses AttitudeController.setProportionalGain (preserves the
% current yaw weight). This is the in-air apply path; safe in simulation.
% =========================================================================
function applyTunedGains(rate_ctl, att_ctl, r)
rate_ctl.gain_p = r.rate_k(:);
rate_ctl.gain_i = r.rate_k(:) .* r.rate_i(:);
rate_ctl.gain_d = r.rate_k(:) .* r.rate_d(:);
att_ctl.setProportionalGain(r.att_p(:), att_ctl.yaw_w);
end


% =========================================================================
% Joystick UI: two rounded "stick pads" with guide rings and a draggable
% stick (tether line + cap) that snaps back to centre on mouse-up.
% =========================================================================
function [left_h, right_h] = makeJoysticks(fig, ax_l, ax_r)
T = gcsTheme();
caps = {'YAW  /  THROTTLE', 'ROLL  /  PITCH'};
axs  = [ax_l, ax_r];
thc  = linspace(0, 2*pi, 90);
for k = 1:2
    ax = axs(k);
    hold(ax, 'on'); axis(ax, 'equal');
    xlim(ax, [-1.18 1.18]); ylim(ax, [-1.45 1.18]);
    set(ax, 'XTick', [], 'YTick', [], 'Box', 'off', ...
            'Color', T.bg, 'XColor', 'none', 'YColor', 'none');
    rectangle(ax, 'Position', [-1.1 -1.1 2.2 2.2], 'Curvature', 0.18, ...
              'FaceColor', T.field, 'EdgeColor', T.edge, 'LineWidth', 1.2, ...
              'HitTest', 'off', 'PickableParts', 'none');
    for r = [0.5 1.0]
        plot(ax, r*cos(thc), r*sin(thc), '-', 'Color', T.edge, ...
             'LineWidth', 0.8, 'HitTest', 'off', 'PickableParts', 'none');
    end
    plot(ax, [-1 1; 0 0]', [0 0; -1 1]', '-', 'Color', T.edge, ...
         'LineWidth', 0.8, 'HitTest', 'off', 'PickableParts', 'none');
    text(ax, 0, -1.32, caps{k}, 'Color', T.sub, 'FontName', T.font, ...
         'FontSize', 8.5, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
         'HitTest', 'off', 'PickableParts', 'none');
end

% Tether lines (centre -> stick cap), drawn under the caps.
left_t  = plot(ax_l, [0 0], [0 0], '-', 'Color', T.sub, 'LineWidth', 2.5, ...
               'HitTest', 'off', 'PickableParts', 'none');
right_t = plot(ax_r, [0 0], [0 0], '-', 'Color', T.sub, 'LineWidth', 2.5, ...
               'HitTest', 'off', 'PickableParts', 'none');
left_h  = plot(ax_l, 0, 0, 'o', 'MarkerSize', 24, ...
               'MarkerFaceColor', T.warn, 'MarkerEdgeColor', T.text, 'LineWidth', 1.2);
right_h = plot(ax_r, 0, 0, 'o', 'MarkerSize', 24, ...
               'MarkerFaceColor', T.data, 'MarkerEdgeColor', T.text, 'LineWidth', 1.2);

set(ax_l,    'ButtonDownFcn', @(~,~) startDrag(fig, 'left'));
set(ax_r,    'ButtonDownFcn', @(~,~) startDrag(fig, 'right'));
set(left_h,  'ButtonDownFcn', @(~,~) startDrag(fig, 'left'));
set(right_h, 'ButtonDownFcn', @(~,~) startDrag(fig, 'right'));

set(fig, 'WindowButtonMotionFcn', ...
    @(~,~) onMotion(fig, ax_l, ax_r, left_h, right_h, left_t, right_t));
set(fig, 'WindowButtonUpFcn', ...
    @(~,~) endDrag(fig, left_h, right_h, left_t, right_t));

setappdata(fig, 'drag_target', '');
end

function startDrag(fig, which_)
setappdata(fig, 'drag_target', which_);
end

function onMotion(fig, ax_l, ax_r, left_h, right_h, left_t, right_t)
target = getappdata(fig, 'drag_target');
if isempty(target), return; end
switch target
    case 'left'
        cp = get(ax_l, 'CurrentPoint');
        x = max(-1, min(1, cp(1, 1)));
        y = max(-1, min(1, cp(1, 2)));
        set(left_h, 'XData', x, 'YData', y);
        set(left_t, 'XData', [0 x], 'YData', [0 y]);
    case 'right'
        cp = get(ax_r, 'CurrentPoint');
        x = max(-1, min(1, cp(1, 1)));
        y = max(-1, min(1, cp(1, 2)));
        set(right_h, 'XData', x, 'YData', y);
        set(right_t, 'XData', [0 x], 'YData', [0 y]);
end
end

function endDrag(fig, left_h, right_h, left_t, right_t)
target = getappdata(fig, 'drag_target');
if isempty(target), return; end
switch target
    case 'left'
        set(left_h, 'XData', 0, 'YData', 0);
        set(left_t, 'XData', [0 0], 'YData', [0 0]);
    case 'right'
        set(right_h, 'XData', 0, 'YData', 0);
        set(right_t, 'XData', [0 0], 'YData', [0 0]);
end
setappdata(fig, 'drag_target', '');
end


% =========================================================================
% Live controller-gain tuning window.
%
% Builds a separate figure with one panel per controller (outer -> inner:
% position, attitude, rate). Every tunable is a slider with a numeric
% readout. Dragging a slider writes the new value straight onto the
% handle-class controller object *continuously* (via the slider's
% ContinuousValueChange event), so the running sim loop uses it on the
% very next tick -- no commit / button press needed.
%
% Each panel has its own "Reset defaults" button that restores that
% controller's params from px4_params (`p`).
%
% Gains shown are the *effective* values held by the objects. For the rate
% loop that means MC_*RATE_K is already folded into P/I/D. Angles (tilt,
% rate-max) are shown in degrees and converted to radians on write.
%
% rowSpec fields: label, tip, get(), set(v), lo, hi (slider range), def
% (default value in display units, from px4_params). lo/hi/def may be
% scalar (broadcast to all components) or per-component vectors.
% =========================================================================
% =========================================================================
% Controller-gains tab. Returns a refresh() handle that re-reads the live
% controller gains into the sliders/edits (used after autotuning).
% =========================================================================
function refresh = buildControllerTab(parent, p, pos_ctl, att_ctl, rate_ctl)
% --- Position controller (outer loop) ---
pos_rows = {
    rowSpec('Pos P  (N E D)',         'MPC_XY_P, MPC_XY_P, MPC_Z_P', ...
            @() pos_ctl.gain_pos_p, @(v) setProp(pos_ctl, 'gain_pos_p', v), ...
            0, 3, p.pos.gain_pos_p);
    rowSpec('Vel P  (N E D)',         'MPC_XY_VEL_P_ACC / MPC_Z_VEL_P_ACC', ...
            @() pos_ctl.gain_vel_p, @(v) setProp(pos_ctl, 'gain_vel_p', v), ...
            0, 8, p.pos.gain_vel_p);
    rowSpec('Vel I  (N E D)',         'MPC_XY_VEL_I_ACC / MPC_Z_VEL_I_ACC', ...
            @() pos_ctl.gain_vel_i, @(v) setProp(pos_ctl, 'gain_vel_i', v), ...
            0, 5, p.pos.gain_vel_i);
    rowSpec('Vel D  (N E D)',         'MPC_XY_VEL_D_ACC / MPC_Z_VEL_D_ACC', ...
            @() pos_ctl.gain_vel_d, @(v) setProp(pos_ctl, 'gain_vel_d', v), ...
            0, 2, p.pos.gain_vel_d);
    rowSpec('Vel max (xy up dn) m/s', 'MPC_XY_VEL_MAX, MPC_Z_VEL_MAX_UP, _DN', ...
            @() [pos_ctl.lim_vel_horizontal; pos_ctl.lim_vel_up; pos_ctl.lim_vel_down], ...
            @(v) setVelLims(pos_ctl, v), ...
            [0;0;0], [25;10;10], [p.pos.vel_xy_max; p.pos.vel_z_up; p.pos.vel_z_down]);
    rowSpec('Tilt max (deg)',         'MPC_TILTMAX_AIR', ...
            @() rad2deg(pos_ctl.lim_tilt), @(v) setProp(pos_ctl, 'lim_tilt', deg2rad(v)), ...
            0, 80, rad2deg(p.pos.tilt_max));
    rowSpec('Thrust (min hov max)',   'MPC_THR_MIN, MPC_THR_HOVER, MPC_THR_MAX', ...
            @() [pos_ctl.thr_min; pos_ctl.hover_thrust; pos_ctl.thr_max], ...
            @(v) setThr(pos_ctl, v), ...
            [0;0;0], [0.5;1;1], [p.pos.thr_min; p.pos.thr_hover; p.pos.thr_max]);
};

% --- Attitude controller ---
att_rows = {
    rowSpec('Att P  (r p y)',         'MC_ROLL_P, MC_PITCH_P, MC_YAW_P', ...
            @() attPGet(att_ctl), @(v) att_ctl.setProportionalGain(v(:), att_ctl.yaw_w), ...
            0, 12, p.att.gain_p);
    rowSpec('Yaw weight',             'MC_YAW_WEIGHT (0..1)', ...
            @() att_ctl.yaw_w, @(v) attYawSet(att_ctl, v), ...
            0, 1, p.att.yaw_weight);
    rowSpec('Rate max (r p y) deg/s', 'MC_ROLLRATE_MAX, MC_PITCHRATE_MAX, MC_YAWRATE_MAX', ...
            @() rad2deg(att_ctl.rate_limit), @(v) setProp(att_ctl, 'rate_limit', deg2rad(v)), ...
            0, 360, rad2deg(p.att.rate_max));
};

% --- Rate controller (inner loop). Defaults shown effective (x MC_*RATE_K). ---
rate_rows = {
    rowSpec('P  (r p y)',             'MC_*RATE_P x MC_*RATE_K (effective)', ...
            @() rate_ctl.gain_p, @(v) setProp(rate_ctl, 'gain_p', v), ...
            0, 0.6, p.rate.gain_p .* p.rate.gain_k);
    rowSpec('I  (r p y)',             'MC_*RATE_I x MC_*RATE_K (effective)', ...
            @() rate_ctl.gain_i, @(v) setProp(rate_ctl, 'gain_i', v), ...
            0, 0.8, p.rate.gain_i .* p.rate.gain_k);
    rowSpec('D  (r p y)',             'MC_*RATE_D x MC_*RATE_K (effective)', ...
            @() rate_ctl.gain_d, @(v) setProp(rate_ctl, 'gain_d', v), ...
            0, 0.02, p.rate.gain_d .* p.rate.gain_k);
    rowSpec('FF (r p y)',             'MC_ROLLRATE_FF, MC_PITCHRATE_FF, MC_YAWRATE_FF', ...
            @() rate_ctl.gain_ff, @(v) setProp(rate_ctl, 'gain_ff', v), ...
            0, 0.5, p.rate.gain_ff);
    rowSpec('Int lim (r p y)',        'MC_RR_INT_LIM, MC_PR_INT_LIM, MC_YR_INT_LIM', ...
            @() rate_ctl.lim_int, @(v) setProp(rate_ctl, 'lim_int', v), ...
            0, 1, p.rate.int_lim);
};

% Panel heights weighted by row count (+ title/padding), outer -> inner.
rp = buildGroup(parent, [0.03 0.553 0.94 0.427], 'Position controller (outer loop)', pos_rows);
ra = buildGroup(parent, [0.03 0.340 0.94 0.205], 'Attitude controller',              att_rows);
ri = buildGroup(parent, [0.03 0.020 0.94 0.310], 'Rate controller (inner loop)',     rate_rows);
refresh = @() cellfun(@(f) f(), {rp, ra, ri});
end


% =========================================================================
% EKF2 tab: live measurement-noise / gate edits + the estimator-feed
% toggle (returned so the sim loop can read it). Measurement-noise and
% gate edits take effect on the next fusion; init covariances need Reset.
% =========================================================================
function est_cb = buildEkfTab(parent, est_bus)
ekf = est_bus.ekf;
d   = Ekf2Params();
rows = {
    rowSpec('Gyro proc noise',  'EKF2_GYR_NOISE (rad/s)', ...
            @() ekf.params.gyr_noise,  @(v) setEkfParam(ekf, 'gyr_noise', v),  0, 0.1,  d.gyr_noise);
    rowSpec('Accel proc noise', 'EKF2_ACC_NOISE (m/s^2)', ...
            @() ekf.params.acc_noise,  @(v) setEkfParam(ekf, 'acc_noise', v),  0, 1.0,  d.acc_noise);
    rowSpec('Gyro bias noise',  'EKF2_GYR_B_NOISE', ...
            @() ekf.params.gyr_b_noise, @(v) setEkfParam(ekf, 'gyr_b_noise', v), 0, 0.01, d.gyr_b_noise);
    rowSpec('Accel bias noise', 'EKF2_ACC_B_NOISE', ...
            @() ekf.params.acc_b_noise, @(v) setEkfParam(ekf, 'acc_b_noise', v), 0, 0.02, d.acc_b_noise);
    rowSpec('Baro noise (m)',   'EKF2_BARO_NOISE', ...
            @() ekf.params.baro_noise, @(v) setEkfParam(ekf, 'baro_noise', v), 0, 6,    d.baro_noise);
    rowSpec('GPS pos noise (m)', 'EKF2_GPS_P_NOISE', ...
            @() ekf.params.gps_p_noise, @(v) setEkfParam(ekf, 'gps_p_noise', v), 0, 3,  d.gps_p_noise);
    rowSpec('GPS vel noise (m/s)', 'EKF2_GPS_V_NOISE', ...
            @() ekf.params.gps_v_noise, @(v) setEkfParam(ekf, 'gps_v_noise', v), 0, 2,  d.gps_v_noise);
    rowSpec('Mag noise (G)',    'EKF2_MAG_NOISE', ...
            @() ekf.params.mag_noise,  @(v) setEkfParam(ekf, 'mag_noise', v),  0, 0.2,  d.mag_noise);
    rowSpec('Heading noise',    'EKF2_HEAD_NOISE (rad)', ...
            @() ekf.params.head_noise, @(v) setEkfParam(ekf, 'head_noise', v), 0, 1,    d.head_noise);
};
buildGroup(parent, [0.03 0.18 0.94 0.80], 'EKF2 noise parameters (live)', rows);

T = gcsTheme();
est_cb = uicontrol(parent, 'Style', 'checkbox', 'Units', 'normalized', ...
    'Position', [0.05 0.09 0.9 0.05], 'Value', 1, ...
    'ForegroundColor', T.good, ...
    'FontWeight', 'bold', 'String', 'Use EKF2 estimator (sensor-driven state feed)');
uicontrol(parent, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.05 0.02 0.9 0.06], 'ForegroundColor', T.sub, ...
    'HorizontalAlignment', 'left', 'FontSize', 8, ...
    'String', ['Noise/gate edits apply on the next fusion. Initial-covariance ' ...
               'and time-constant params take effect on Reset.']);
end


% =========================================================================
% Sensor-noise tab: edits apply to every chip of each type so whichever is
% voted primary uses the new value. All fields read live in measure().
% =========================================================================
function buildSensorTab(parent, est_bus)
imus  = est_bus.sensors.imu.sensors;
baros = est_bus.sensors.baro.sensors;
mags  = est_bus.sensors.mag.sensors;
gnss  = est_bus.sensors.gnss.sensors;
rows = {
    rowSpec('Gyro noise dens',  'IMU gyro_nd (rad/s/sqrt Hz)', ...
            @() imus{1}.gyro_nd,  @(v) setAll(imus, 'gyro_nd', v),  0, 5e-3, imus{1}.gyro_nd);
    rowSpec('Accel noise dens', 'IMU accel_nd (m/s^2/sqrt Hz)', ...
            @() imus{1}.accel_nd, @(v) setAll(imus, 'accel_nd', v), 0, 5e-3, imus{1}.accel_nd);
    rowSpec('Gyro vib gain',    'rad/s per unit vib_level', ...
            @() imus{1}.gyro_vib_gain,  @(v) setAll(imus, 'gyro_vib_gain', v),  0, 1,  imus{1}.gyro_vib_gain);
    rowSpec('Accel vib gain',   'm/s^2 per unit vib_level', ...
            @() imus{1}.accel_vib_gain, @(v) setAll(imus, 'accel_vib_gain', v), 0, 20, imus{1}.accel_vib_gain);
    rowSpec('Baro noise (m)',   'BaroSensor alt_noise_m', ...
            @() baros{1}.alt_noise_m, @(v) setAll(baros, 'alt_noise_m', v), 0, 5,   baros{1}.alt_noise_m);
    rowSpec('Mag noise (G)',    'MagSensor mag_noise_g', ...
            @() mags{1}.mag_noise_g,  @(v) setAll(mags, 'mag_noise_g', v),  0, 0.2, mags{1}.mag_noise_g);
    rowSpec('GPS pos H/V (m)',  'GNSS pos_h/pos_v noise', ...
            @() [gnss{1}.pos_h_noise_m; gnss{1}.pos_v_noise_m], ...
            @(v) setGnss(gnss, {'pos_h_noise_m', 'pos_v_noise_m'}, v), ...
            [0; 0], [5; 8], [gnss{1}.pos_h_noise_m; gnss{1}.pos_v_noise_m]);
    rowSpec('GPS vel H/V (m/s)', 'GNSS vel_h/vel_v noise', ...
            @() [gnss{1}.vel_h_noise_mps; gnss{1}.vel_v_noise_mps], ...
            @(v) setGnss(gnss, {'vel_h_noise_mps', 'vel_v_noise_mps'}, v), ...
            [0; 0], [2; 2], [gnss{1}.vel_h_noise_mps; gnss{1}.vel_v_noise_mps]);
};
buildGroup(parent, [0.03 0.03 0.94 0.95], ...
           'Sensor noise (applied to every chip of each type)', rows);
end


% =========================================================================
% Wind-model tab: steady NED + turbulence intensity/correlation, all bound
% to the live WindModel object.
% =========================================================================
function buildWindTab(parent, p, wind)
rows = {
    rowSpec('Steady N E D (m/s)', 'steady wind in NED', ...
            @() wind.steady_ned, @(v) setProp(wind, 'steady_ned', v), ...
            [-15; -15; -15], [15; 15; 15], p.wind.steady);
    rowSpec('Turb sigma (m/s)', 'turbulence 1-sigma per axis', ...
            @() wind.turb_sigma, @(v) setProp(wind, 'turb_sigma', v), 0, 5, p.wind.turb_sigma);
    rowSpec('Turb tau (s)', 'turbulence correlation time', ...
            @() wind.turb_tau, @(v) setProp(wind, 'turb_tau', v), 0.1, 10, p.wind.turb_tau);
};
buildGroup(parent, [0.03 0.18 0.94 0.80], 'Wind model (NED)', rows);
cb = uicontrol(parent, 'Style', 'checkbox', 'Units', 'normalized', ...
    'Position', [0.05 0.07 0.9 0.06], ...
    'Value', double(wind.turb_enable), 'FontWeight', 'bold', ...
    'String', 'Enable turbulence (Ornstein-Uhlenbeck)');
set(cb, 'Callback', @(src, ~) setWindEnable(wind, src));
end


% =========================================================================
% Cesium/Unity streaming tab: a single opt-in toggle. When checked, the sim
% loop streams the vehicle pose to the Cesium-Unity scene via CesiumBridge
% (geometry_msgs/PoseArray on /world/default/pose/info, 50 Hz frame rate).
% Only rosbridge_server is needed on the Unity side; PX4/Gazebo are not.
% =========================================================================
function cesium_cb = buildCesiumTab(parent)
T = gcsTheme();
cesium_cb = uicontrol(parent, 'Style', 'checkbox', 'Units', 'normalized', ...
    'Position', [0.05 0.91 0.9 0.05], 'Value', 1, ...
    'FontWeight', 'bold', 'String', 'Stream pose to Cesium/Unity (ROS 2)');
uicontrol(parent, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.05 0.56 0.9 0.30], 'ForegroundColor', T.sub, ...
    'HorizontalAlignment', 'left', 'FontSize', 9, ...
    'String', sprintf(['Pose stream: ground-truth pose as geometry_msgs/' ...
        'PoseArray on /world/default/pose/info (50 Hz). rosbridge_server is ' ...
        'auto-launched when run_interactive starts and stopped on Stop ' ...
        '(log: /tmp/px4_rosbridge.log).']));
end


% =========================================================================
% rosbridge_server lifecycle. Auto-launched on run_interactive start (for the
% Unity/Cesium WebSocket path) and stopped on Stop. Returns the launch PID, or
% [] if it was already running / could not be started (so we never kill a
% rosbridge we did not start). MATLAB's LD_LIBRARY_PATH is stripped so the
% ros2 CLI (python) does not load MATLAB's shared libs.
% =========================================================================
function pid = startRosbridge()
pid = [];
if ~isunix
    % Windows: everything Ubuntu-side runs manually in a WSL terminal
    % (see README.md). MATLAB only connects to ws://localhost:9090.
    fprintf(['rosbridge is started manually on Windows. In a WSL terminal:\n' ...
             '  source /opt/ros/humble/setup.bash && ' ...
             'ros2 launch rosbridge_server rosbridge_websocket_launch.xml\n']);
    return;
end
[~, running] = system('pgrep -f rosbridge_websocket');
if ~isempty(strtrim(running))
    fprintf('rosbridge_server already running; leaving it as-is.\n');
    return;   % pid stays [] -> stopRosbridge() will not touch it
end
% `exec` makes the backgrounded subshell BECOME ros2 launch, so $! is the
% ros2-launch PID (not a throwaway subshell) and SIGINT later reaches it.
cmd = ['bash -lc ''unset LD_LIBRARY_PATH; ' ...
       'source /opt/ros/humble/setup.bash && ' ...
       'exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml ' ...
       '>/tmp/px4_rosbridge.log 2>&1 & echo $!'''];
[st, out] = system(cmd);
pidnum = str2double(strtrim(out));
if st == 0 && isfinite(pidnum) && pidnum > 0
    pid = pidnum;
    fprintf('Started rosbridge_server (pid %d). Log: /tmp/px4_rosbridge.log\n', pid);
else
    warning(['Could not auto-launch rosbridge_server (see ' ...
             '/tmp/px4_rosbridge.log). Start it manually if you need Unity.']);
end
end

function stopRosbridge(pid)
if isempty(pid) || ~isfinite(pid) || pid <= 0, return; end
% Only SIGINT if the PID is STILL a rosbridge/ros2-launch process (guards
% against PID reuse). ros2 launch then shuts its child nodes down gracefully.
cmd = sprintf(['ps -p %d -o args= 2>/dev/null | grep -q rosbridge ' ...
               '&& kill -INT %d 2>/dev/null'], pid, pid);
[st, ~] = system(cmd);
if st == 0
    fprintf('Stopped rosbridge_server (pid %d).\n', pid);
end
end


% =========================================================================
% Mission tab: a north-up satellite map (700 m x 700 m) anchored at the
% Cesium/Unity georeference origin (Quba.unity CesiumGeoreference: lat
% 40.32214266903304, lon 49.59745, Baku) by default, re-anchored on the
% home position when a QGC .plan is loaded. Click the map to drop a
% waypoint at the clicked (East, North) using the altitude field (metres,
% +up); markers are colour coded launch=green, end=red, mid=yellow, each
% labelled with its lat/lon. Big buttons select flight mode; Save/Load
% read+write QGroundControl .plan files. All waypoint edits flow through the
% shared `waypoints` array (map_add_request / alt_edit_request /
% load_plan_request, applied by the sim loop) so the map and 3D view stay in
% sync. Waypoints are local NED offsets; the geodesy here places the basemap
% and converts to/from lat/lon for labels and .plan I/O. The anchor (lat0,
% lon0), half-extent, basemap image, and label handles live in appdata on
% the axes (mm is a value struct).
% =========================================================================
function mm = buildMissionTab(parent, fig, fmm)
T = gcsTheme();
lat0 = 40.32214266903304;   % Cesium origin (Baku), verified from Quba.unity
lon0 = 49.59745;
HALF = 350;                 % half-extent -> 700 m x 700 m map

% --- map axes (left), north-up local metres -------------------------------
axm = axes('Parent', parent, 'Units', 'normalized', ...
           'Position', [0.040 0.075 0.585 0.895]);
hold(axm, 'on'); box(axm, 'on'); axis(axm, 'equal');
set(axm, 'YDir', 'normal', 'FontSize', 8);        % North up
xlim(axm, [-HALF HALF]); ylim(axm, [-HALF HALF]);
xlabel(axm, 'EAST (m)'); ylabel(axm, 'NORTH (m)');
% persistent state for re-anchoring / labels / save (mm is a value struct).
setappdata(axm, 'lat0', lat0);  setappdata(axm, 'lon0', lon0);
setappdata(axm, 'HALF', HALF);  setappdata(axm, 'wp_labels', gobjects(0));

addSatelliteBasemap(axm, lat0, lon0, HALF);       % best-effort imagery

grid(axm, 'on');
set(axm, 'Layer', 'top', 'GridColor', [1 1 1], 'GridAlpha', 0.18);
plot(axm, 0, 0, '+', 'Color', 'w', 'MarkerSize', 12, 'LineWidth', 1.5, ...
     'HitTest', 'off', 'PickableParts', 'none');   % origin / home
% map-anchor caption (bottom-left corner; updated by setMissionAnchor)
anchor_lbl = text(axm, -HALF + 12, -HALF + 14, ...
    sprintf('ANCHOR %.5f, %.5f   \\cdot   CLICK MAP TO ADD WAYPOINT', lat0, lon0), ...
    'Color', T.text, 'FontName', T.mono, 'FontSize', 7.5, 'FontWeight', 'bold', ...
    'BackgroundColor', T.bg, 'Margin', 3, ...
    'HitTest', 'off', 'PickableParts', 'none');
setappdata(axm, 'anchor_lbl', anchor_lbl);

% waypoint graphics (data filled by updateMissionMap); HitTest off so clicks
% fall through to the axes ButtonDownFcn. The course line is magenta --
% the avionics color for the active navigation reference (FAA AC 25-11B).
hPath   = plot(axm, NaN, NaN, '-', 'Color', T.nav, 'LineWidth', 1.6, ...
               'HitTest', 'off', 'PickableParts', 'none');
hMid    = plot(axm, NaN, NaN, 'o', 'MarkerSize', 9,  'LineWidth', 1.0, ...
               'MarkerFaceColor', T.warn, 'MarkerEdgeColor', 'k', ...
               'HitTest', 'off', 'PickableParts', 'none');
hLaunch = plot(axm, NaN, NaN, 'o', 'MarkerSize', 11, 'LineWidth', 1.0, ...
               'MarkerFaceColor', T.good, 'MarkerEdgeColor', 'k', ...
               'HitTest', 'off', 'PickableParts', 'none');
hEnd    = plot(axm, NaN, NaN, 'o', 'MarkerSize', 11, 'LineWidth', 1.0, ...
               'MarkerFaceColor', T.bad, 'MarkerEdgeColor', 'k', ...
               'HitTest', 'off', 'PickableParts', 'none');

% Live 2D flown-path trails (filled by the sim loop every frame):
% ground truth (white), EKF (cyan). animatedline is incremental, so per-frame
% appends stay cheap.
% MaximumNumPoints caps the trail so the Mission map does not slow down over a
% long flight.
gtTrail  = animatedline(axm, 'Color', [1 1 1],     'LineWidth', 3.0, ...
                        'MaximumNumPoints', 6000, 'HitTest', 'off', 'PickableParts', 'none');
ekfTrail = animatedline(axm, 'Color', T.data,      'LineWidth', 3.0, ...
                        'MaximumNumPoints', 6000, 'HitTest', 'off', 'PickableParts', 'none');
% Clickable legend: click an entry to hide/show that trail (toggles its
% Visible). AutoUpdate off so it does not pick up the basemap/markers.
lgd = legend(axm, [gtTrail, ekfTrail], ...
             {'Ground truth', 'EKF'}, ...
             'Location', 'northeast', 'AutoUpdate', 'off', ...
             'TextColor', T.text, 'Color', T.panel, 'EdgeColor', T.edge, ...
             'FontSize', 9, 'Box', 'on');
lgd.ItemHitFcn = @legendToggleTrail;

% --- right column: instruments, mission plan, telemetry -------------------
inst_pan = uipanel(parent, 'Units', 'normalized', ...
    'Position', [0.655 0.545 0.335 0.435], 'BackgroundColor', T.panel);
pfd = buildPFD(inst_pan);

plan_pan = uipanel(parent, 'Units', 'normalized', ...
    'Position', [0.655 0.225 0.335 0.310], 'BackgroundColor', T.panel, ...
    'Title', ' MISSION PLAN ', 'FontWeight', 'bold', 'FontSize', 8);

uicontrol(plan_pan, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.04 0.870 0.58 0.100], 'ForegroundColor', T.sub, ...
    'HorizontalAlignment', 'left', 'FontSize', 8, ...
    'String', 'NEXT WAYPOINT ALTITUDE (m, +up)');
altEdit = uicontrol(plan_pan, 'Style', 'edit', 'Units', 'normalized', ...
    'Position', [0.66 0.865 0.30 0.115], 'String', '10', ...
    'BackgroundColor', T.field, 'ForegroundColor', T.text, ...
    'FontName', T.mono, 'FontWeight', 'bold');

% Takeoff target altitude (MIS_TAKEOFF_ALT role): applied to the next
% Takeoff command; the auto-transition to Hold happens at this height.
uicontrol(plan_pan, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.04 0.745 0.58 0.100], 'ForegroundColor', T.sub, ...
    'HorizontalAlignment', 'left', 'FontSize', 8, ...
    'String', 'TAKEOFF ALTITUDE (m, +up)');
uicontrol(plan_pan, 'Style', 'edit', 'Units', 'normalized', ...
    'Position', [0.66 0.740 0.30 0.115], ...
    'String', num2str(fmm.p.auto.takeoff_alt), ...
    'BackgroundColor', T.field, 'ForegroundColor', T.text, ...
    'FontName', T.mono, 'FontWeight', 'bold', ...
    'Callback', @(src, ~) onTakeoffAltEdit(src, fmm));

uicontrol(plan_pan, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.04 0.595 0.45 0.125], 'String', 'LOAD .PLAN', ...
    'FontWeight', 'bold', 'BackgroundColor', T.btn, ...
    'Callback', @(~,~) onLoadPlan(fig));
uicontrol(plan_pan, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.51 0.595 0.45 0.125], 'String', 'SAVE .PLAN', ...
    'FontWeight', 'bold', 'BackgroundColor', T.btn, ...
    'Callback', @(~,~) setappdata(fig, 'save_plan_request', true));
uicontrol(plan_pan, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.04 0.455 0.45 0.125], 'String', 'REMOVE LAST', ...
    'FontWeight', 'bold', 'BackgroundColor', T.btn, ...
    'Callback', @(~,~) setappdata(fig, 'pop_request', true));
uicontrol(plan_pan, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.51 0.455 0.45 0.125], 'String', 'CLEAR ALL', ...
    'FontWeight', 'bold', 'BackgroundColor', T.btn, 'ForegroundColor', T.warn, ...
    'Callback', @(~,~) setappdata(fig, 'clear_request', true));

tbl = uitable('Parent', plan_pan, 'Units', 'normalized', ...
    'Position', [0.04 0.030 0.92 0.400], ...
    'ColumnName', {'N (m)', 'E (m)', 'Alt (m)'}, ...
    'ColumnEditable', [false false true], ...
    'ColumnWidth', {70 70 70}, 'RowName', 'numbered', ...
    'BackgroundColor', [T.panel; T.btn], 'ForegroundColor', T.text, ...
    'FontName', T.mono, 'FontSize', 8, ...
    'CellEditCallback', @(~, ev) onAltEdit(fig, ev));

state_lbl = uicontrol(parent, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.655 0.035 0.335 0.175], 'BackgroundColor', T.field, ...
    'ForegroundColor', T.text, 'HorizontalAlignment', 'left', ...
    'FontName', T.mono, 'FontSize', 9, 'String', '');

% click handler: axes children have HitTest off, so the axes gets the click.
set(axm, 'ButtonDownFcn', @(src, ~) onMapClick(src, fig, altEdit, HALF));

mm = struct('ax', axm, 'path', hPath, 'launch', hLaunch, 'mid', hMid, ...
            'endp', hEnd, 'table', tbl, 'altEdit', altEdit, ...
            'state_lbl', state_lbl, 'pfd', pfd, ...
            'gt_trail', gtTrail, 'ekf_trail', ekfTrail);
end

% =========================================================================
% Attitude indicator + heading tape ("mini-PFD") on the Flight tab.
% Sky/ground rotate with roll and translate with pitch (hgtransform); the
% roll scale rotates with the horizon under a fixed bank pointer (so the
% tick under the pointer always reads the bank angle); the pitch ladder is
% marked every 5 deg, labelled every 10 deg. The heading tape below scrolls
% under a fixed amber lubber line with a digital readout.
% Colors per FAA AC 25-11B: cyan/blue sky, tan ground, white scales.
% =========================================================================
function P = buildPFD(parent)
T = gcsTheme();
k10 = 0.30;                 % ADI vertical units per 10 deg of pitch

% --- attitude indicator ----------------------------------------------------
axA = axes('Parent', parent, 'Units', 'normalized', ...
           'Position', [0.05 0.325 0.90 0.645]);
hold(axA, 'on');
set(axA, 'XTick', [], 'YTick', [], 'Box', 'on', 'Layer', 'top', ...
         'Color', T.field, 'XColor', T.edge, 'YColor', T.edge, ...
         'DataAspectRatio', [1 1 1]);
xlim(axA, [-1.30 1.30]); ylim(axA, [-1.02 1.02]);
disableDefaultInteractivity(axA);

tr = hgtransform('Parent', axA);                  % horizon: roll + pitch
patch('Parent', tr, 'XData', [-8 8 8 -8], 'YData', [0 0 8 8], ...
      'FaceColor', T.sky, 'EdgeColor', 'none', 'HitTest', 'off');
patch('Parent', tr, 'XData', [-8 8 8 -8], 'YData', [-8 -8 0 0], ...
      'FaceColor', T.gnd, 'EdgeColor', 'none', 'HitTest', 'off');
line('Parent', tr, 'XData', [-8 8], 'YData', [0 0], ...
     'Color', 'w', 'LineWidth', 1.8, 'HitTest', 'off');
% pitch ladder: one NaN-separated line + labels, all rolling with the horizon
xs = []; ys = [];
for d = 10:10:40                                  % major bars (10 deg)
    y = d/10 * k10;
    xs = [xs, -0.26, 0.26, NaN, -0.26, 0.26, NaN]; %#ok<AGROW>
    ys = [ys, y, y, NaN, -y, -y, NaN];             %#ok<AGROW>
end
for d = 5:10:35                                   % minor bars (5 deg)
    y = d/10 * k10;
    xs = [xs, -0.10, 0.10, NaN, -0.10, 0.10, NaN]; %#ok<AGROW>
    ys = [ys, y, y, NaN, -y, -y, NaN];             %#ok<AGROW>
end
line('Parent', tr, 'XData', xs, 'YData', ys, 'Color', 'w', ...
     'LineWidth', 1.0, 'HitTest', 'off');
for d = [-30 -20 -10 10 20 30]
    y = d/10 * k10;
    text('Parent', tr, 'Position', [-0.33 y 0], 'String', num2str(abs(d)), ...
        'Color', 'w', 'FontName', T.mono, 'FontSize', 7.5, ...
        'HorizontalAlignment', 'right', 'HitTest', 'off');
    text('Parent', tr, 'Position', [0.33 y 0], 'String', num2str(abs(d)), ...
        'Color', 'w', 'FontName', T.mono, 'FontSize', 7.5, ...
        'HorizontalAlignment', 'left', 'HitTest', 'off');
end

trR = hgtransform('Parent', axA);                 % roll scale: roll only
xs = []; ys = [];
for a = [-60 -45 -30 -20 -10 0 10 20 30 45 60]
    r0 = 0.78;
    r1 = r0 + 0.06 + 0.05 * any(a == [0 -30 30 -60 60]);
    th = pi/2 - deg2rad(a);          % bank-a tick sits under the pointer at bank a
    xs = [xs, r0*cos(th), r1*cos(th), NaN]; %#ok<AGROW>
    ys = [ys, r0*sin(th), r1*sin(th), NaN]; %#ok<AGROW>
end
line('Parent', trR, 'XData', xs, 'YData', ys, 'Color', 'w', ...
     'LineWidth', 1.0, 'HitTest', 'off');
patch('Parent', axA, 'XData', [-0.055 0.055 0], 'YData', [0.68 0.68 0.765], ...
      'FaceColor', T.acft, 'EdgeColor', 'none', 'HitTest', 'off');  % fixed pointer

% fixed aircraft symbol (yellow wings + centre dot)
line('Parent', axA, 'XData', [-0.55 -0.20 NaN 0.20 0.55], ...
     'YData', [0 0 NaN 0 0], 'Color', T.acft, 'LineWidth', 4, 'HitTest', 'off');
line('Parent', axA, 'XData', [-0.20 -0.20 NaN 0.20 0.20], ...
     'YData', [0 -0.09 NaN 0 -0.09], 'Color', T.acft, 'LineWidth', 4, 'HitTest', 'off');
line('Parent', axA, 'XData', 0, 'YData', 0, 'Marker', 'o', 'MarkerSize', 4.5, ...
     'MarkerFaceColor', T.acft, 'MarkerEdgeColor', T.acft, 'HitTest', 'off');

% --- heading tape -----------------------------------------------------------
axH = axes('Parent', parent, 'Units', 'normalized', ...
           'Position', [0.05 0.045 0.90 0.235]);
hold(axH, 'on');
set(axH, 'XTick', [], 'YTick', [], 'Box', 'on', 'Layer', 'top', ...
         'Color', T.field, 'XColor', T.edge, 'YColor', T.edge);
xlim(axH, [-45 45]); ylim(axH, [0 1]);
disableDefaultInteractivity(axH);

ticksH = line('Parent', axH, 'XData', NaN, 'YData', NaN, 'Color', 'w', ...
              'LineWidth', 1.0, 'HitTest', 'off');
lblH = gobjects(1, 5);
for i = 1:5
    lblH(i) = text('Parent', axH, 'Position', [0 0.47 0], 'String', '', ...
        'Color', 'w', 'FontName', T.mono, 'FontSize', 8, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center', 'HitTest', 'off');
end
line('Parent', axH, 'XData', [0 0], 'YData', [0 0.36], ...
     'Color', T.acft, 'LineWidth', 1.6, 'HitTest', 'off');   % lubber line
hdgTxt = text('Parent', axH, 'Position', [0 0.80 0], 'String', '---', ...
    'Color', T.text, 'FontName', T.mono, 'FontSize', 11, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'BackgroundColor', T.bg, ...
    'EdgeColor', T.edge, 'Margin', 2, 'HitTest', 'off');

H = struct('tr', tr, 'trR', trR, 'ticks', ticksH, 'lbl', lblH, ...
           'hdgTxt', hdgTxt, 'k10', k10);
P = struct('update', @(roll, pitch, hdg_deg) pfdUpdate(H, roll, pitch, hdg_deg));
end

% Per-frame PFD refresh. roll/pitch in rad, hdg in deg [0, 360).
function pfdUpdate(H, roll, pitch, hdg_deg)
pu = -pitch / deg2rad(10) * H.k10;                % pitch up -> horizon down
H.tr.Matrix  = makehgtform('zrotate', roll) * makehgtform('translate', [0 pu 0]);
H.trR.Matrix = makehgtform('zrotate', roll);

h5 = (ceil((hdg_deg - 44)/5) : floor((hdg_deg + 44)/5)) * 5;
x  = h5 - hdg_deg;
ht = 0.18 + 0.18 * (mod(h5, 10) == 0);            % tall every 10 deg
n  = numel(x);
xs = [x; x; nan(1, n)];
ys = [zeros(1, n); ht; nan(1, n)];
set(H.ticks, 'XData', xs(:), 'YData', ys(:));

names = {'N', '3', '6', 'E', '12', '15', 'S', '21', '24', 'W', '30', '33'};
base  = round(hdg_deg / 30);
for i = 1:5
    m  = base + i - 3;
    xo = m * 30 - hdg_deg;
    if abs(xo) <= 38, vis = 'on'; else, vis = 'off'; end
    set(H.lbl(i), 'Position', [xo 0.47 0], ...
        'String', names{mod(m, 12) + 1}, 'Visible', vis);
end
set(H.hdgTxt, 'String', sprintf('%03.0f', hdg_deg));
end


% =========================================================================
% Top telemetry strip: mode badge, primary flight numbers, health pills,
% mission clock. Built once; updateStatusStrip() refreshes it per frame.
% =========================================================================
function S = buildStatusStrip(fig)
T = gcsTheme();
pan = uipanel(fig, 'Units', 'normalized', 'Position', [0 0.930 1 0.070], ...
    'BackgroundColor', T.bg, 'BorderType', 'line', 'HighlightColor', T.edge);

uicontrol(pan, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.008 0.46 0.110 0.42], 'BackgroundColor', T.bg, ...
    'ForegroundColor', T.text, 'FontWeight', 'bold', 'FontSize', 13, ...
    'HorizontalAlignment', 'left', 'String', 'PX4 GCS');
uicontrol(pan, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.008 0.10 0.120 0.30], 'BackgroundColor', T.bg, ...
    'ForegroundColor', T.sub, 'FontSize', 7, ...
    'HorizontalAlignment', 'left', 'String', 'MATLAB SIL · PIXHAWK 6X · NEO-M9N');

stripCap(pan, 0.125, 0.085, 'MODE');
S.mode = stripVal(pan, 0.125, 0.085, T.good, 14);

caps = {'ALT  m', 'V/S  m/s', 'GS  m/s', 'HDG  deg', 'N  m', 'E  m'};
flds = {'alt', 'vs', 'gs', 'hdg', 'n', 'e'};
x0 = 0.225; w = 0.077;
for i = 1:numel(caps)
    x = x0 + (i-1) * w;
    stripCap(pan, x, w - 0.004, caps{i});
    S.(flds{i}) = stripVal(pan, x, w - 0.004, T.text, 13);
end

S.p_arm  = makePill(pan, 0.670);
S.p_ekf  = makePill(pan, 0.724);
S.p_gps  = makePill(pan, 0.778);
S.p_link = makePill(pan, 0.886);

stripCap(pan, 0.942, 0.055, 'MISSION TIME');
S.clock = stripVal(pan, 0.942, 0.055, T.text, 13);
end

% Small caption above a strip value.
function stripCap(pan, x, w, str)
T = gcsTheme();
uicontrol(pan, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [x 0.60 w 0.27], 'BackgroundColor', T.bg, ...
    'ForegroundColor', T.sub, 'FontSize', 7, ...
    'HorizontalAlignment', 'left', 'String', str);
end

% Large monospace strip value; returns the handle for per-frame updates.
function h = stripVal(pan, x, w, col, fs)
T = gcsTheme();
h = uicontrol(pan, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [x 0.06 w 0.52], 'BackgroundColor', T.bg, ...
    'ForegroundColor', col, 'FontName', T.mono, 'FontWeight', 'bold', ...
    'FontSize', fs, 'HorizontalAlignment', 'left', 'String', '--');
end

% Health pill (chip); setPill() drives label + state color.
function h = makePill(pan, x)
T = gcsTheme();
h = uicontrol(pan, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [x 0.28 0.050 0.44], 'BackgroundColor', T.panel, ...
    'ForegroundColor', T.sub, 'FontSize', 8, 'FontWeight', 'bold', ...
    'String', '--');
end

% Per-frame strip refresh (called from the sim loop).
function updateStatusStrip(S, mode, alt, vs, gs, hdg, N, E, t, ...
                           armed, use_est, link_on)
set(S.mode, 'String', upper(mode));
set(S.alt, 'String', sprintf('%7.1f', alt));
set(S.vs,  'String', sprintf('%+7.1f', vs));
set(S.gs,  'String', sprintf('%7.1f', gs));
set(S.hdg, 'String', sprintf('%03.0f', hdg));
set(S.n,   'String', sprintf('%+8.1f', N));
set(S.e,   'String', sprintf('%+8.1f', E));
set(S.clock, 'String', sprintf('T+%02d:%02d', floor(t/60), floor(mod(t, 60))));
if armed, setPill(S.p_arm, 'ARMED', 'bad');
else,     setPill(S.p_arm, 'DISARMED', 'off'); end
if use_est, setPill(S.p_ekf, 'EKF', 'good');
else,       setPill(S.p_ekf, 'GT FEED', 'warn'); end
setPill(S.p_gps, 'GPS', 'good');
if link_on, setPill(S.p_link, 'LINK', 'good');
else,       setPill(S.p_link, 'LINK', 'off'); end
end

% Set a pill's label + state color; skips the redraw when unchanged.
function setPill(h, str, lvl)
if strcmp(get(h, 'String'), str), return; end
T = gcsTheme();
switch lvl
    case 'good', fg = T.good; bgc = T.goodbg;
    case 'warn', fg = T.warn; bgc = T.warnbg;
    case 'bad',  fg = T.bad;  bgc = T.badbg;
    case 'nav',  fg = T.nav;  bgc = T.navbg;
    otherwise,   fg = T.sub;  bgc = T.panel;
end
set(h, 'String', str, 'ForegroundColor', fg, 'BackgroundColor', bgc);
end

% NaN-tolerant numeric formatting for setpoint readouts ('--' when the
% axis has no position setpoint, e.g. velocity-only manual control).
function s = n2s(v, fmt)
if isfinite(v)
    s = sprintf(fmt, v);
else
    s = '    -- ';
end
end


% =========================================================================
% Global bottom bar: flight-mode buttons (selected = green, the avionics
% "engaged" color) + Set Home / Reset / Stop. Lives outside the tab group
% so the pilot can change modes and stop from any tab.
% =========================================================================
function [mode_bg, arm_btn] = buildModeBar(fig, modes)
T = gcsTheme();
bar = uipanel(fig, 'Units', 'normalized', 'Position', [0 0 1 0.088], ...
    'BackgroundColor', T.bg, 'BorderType', 'line', 'HighlightColor', T.edge);

mode_bg = uibuttongroup(bar, 'Units', 'normalized', ...
    'Position', [0.006 0.10 0.625 0.80], 'BackgroundColor', T.bg, ...
    'BorderType', 'none', ...
    'SelectionChangedFcn', @(bg, ~) restyleModeButtons(bg));
nM = numel(modes);
bw = 1 / nM;
for i = 1:nM
    s = modes{i};
    uicontrol(mode_bg, 'Style', 'togglebutton', 'Units', 'normalized', ...
        'Position', [(i-1)*bw + 0.003, 0.06, bw - 0.006, 0.88], ...
        'String', upper(s), 'Tag', s, 'FontSize', 9, ...
        'BackgroundColor', T.panel, 'ForegroundColor', T.sub);
end
posBtn = findobj(mode_bg, 'Tag', 'position');   % default = Position
if ~isempty(posBtn), set(mode_bg, 'SelectedObject', posBtn(1)); end
restyleModeButtons(mode_bg);

arm_btn = uicontrol(bar, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.640 0.14 0.072 0.72], 'String', 'ARM', ...
    'FontWeight', 'bold', 'FontSize', 10, ...
    'BackgroundColor', T.btn, 'ForegroundColor', T.good, ...
    'Callback', @(~,~) setappdata(fig, 'arm_request', true));
uicontrol(bar, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.720 0.14 0.082 0.72], 'String', 'SET HOME', ...
    'FontWeight', 'bold', 'BackgroundColor', T.btn, ...
    'Callback', @(~,~) setappdata(fig, 'set_home_request', true));
uicontrol(bar, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.810 0.14 0.082 0.72], 'String', 'RESET', ...
    'FontWeight', 'bold', 'BackgroundColor', T.btn, 'ForegroundColor', T.warn, ...
    'Callback', @(~,~) setappdata(fig, 'reset_request', true));
uicontrol(bar, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.900 0.14 0.094 0.72], 'String', 'STOP', ...
    'FontWeight', 'bold', 'FontSize', 10, ...
    'BackgroundColor', T.badbg, 'ForegroundColor', T.bad, ...
    'Callback', @(~,~) setappdata(fig, 'running', false));
end

% Engaged mode reads green; the rest stay quiet. Called on user clicks
% (SelectionChangedFcn) and on auto-transitions (setModeButton).
function restyleModeButtons(bg)
if ~isvalid(bg), return; end
T = gcsTheme();
kids = findobj(bg, 'Style', 'togglebutton');
sel  = get(bg, 'SelectedObject');
set(kids, 'BackgroundColor', T.panel, 'ForegroundColor', T.sub, ...
    'FontWeight', 'normal');
if ~isempty(sel)
    set(sel, 'BackgroundColor', T.goodbg, 'ForegroundColor', T.good, ...
        'FontWeight', 'bold');
end
end


% =========================================================================
% GCS theme: one palette + font pair for every window. Colors follow the
% avionics conventions of FAA AC 25-11B (red = warning, amber = caution,
% green = normal/engaged, cyan sky / tan ground, magenta = active nav
% reference) on a dark blue-slate night-ops base.
% =========================================================================
function T = gcsTheme()
persistent Tc
if isempty(Tc)
    mono = pickFont({'Ubuntu Mono', 'JetBrains Mono', 'DejaVu Sans Mono', ...
                     'Liberation Mono', 'Consolas'}, 'FixedWidth');
    sans = pickFont({'Ubuntu', 'Noto Sans', 'DejaVu Sans', 'Segoe UI'}, ...
                    'Helvetica');
    bg   = hexrgb('0D1117');
    good = hexrgb('43D29A');
    warn = hexrgb('FFB02E');
    bad  = hexrgb('FF5252');
    nav  = hexrgb('E060E0');
    Tc = struct( ...
        'bg',    bg, ...                  % window chrome
        'panel', hexrgb('151C24'), ...    % tabs / cards
        'field', hexrgb('0A0E13'), ...    % input fields / instrument wells
        'btn',   hexrgb('1F2935'), ...    % buttons / table stripe
        'edge',  hexrgb('2B3947'), ...    % hairline borders
        'text',  hexrgb('E9EEF3'), ...    % primary text
        'sub',   hexrgb('8CA0B3'), ...    % captions / secondary text
        'good',  good, 'goodbg', mixc(good, bg, 0.82), ...
        'warn',  warn, 'warnbg', mixc(warn, bg, 0.82), ...
        'bad',   bad,  'badbg',  mixc(bad,  bg, 0.78), ...
        'nav',   nav,  'navbg',  mixc(nav,  bg, 0.82), ...
        'data',  hexrgb('6FD3FF'), ...    % cyan: data / EKF trail
        'sky',   hexrgb('2C5B86'), ...    % ADI sky
        'gnd',   hexrgb('5A4632'), ...    % ADI ground (tan)
        'acft',  hexrgb('FFD24D'), ...    % aircraft symbol / lubber yellow
        'font',  sans, 'mono', mono);
end
T = Tc;
end

% Apply the theme as figure-level defaults so every uicontrol / panel /
% axes created under figh inherits it unless explicitly overridden.
function applyThemeDefaults(figh, T)
set(figh, ...
    'DefaultUicontrolBackgroundColor', T.panel, ...
    'DefaultUicontrolForegroundColor', T.text, ...
    'DefaultUicontrolFontName', T.font, ...
    'DefaultUicontrolFontSize', 9, ...
    'DefaultUipanelBackgroundColor', T.panel, ...
    'DefaultUipanelForegroundColor', T.sub, ...
    'DefaultUipanelHighlightColor', T.edge, ...
    'DefaultUipanelShadowColor', T.bg, ...
    'DefaultUipanelBorderType', 'line', ...
    'DefaultUipanelFontName', T.font, ...
    'DefaultUibuttongroupBackgroundColor', T.panel, ...
    'DefaultUibuttongroupForegroundColor', T.sub, ...
    'DefaultUibuttongroupHighlightColor', T.edge, ...
    'DefaultUibuttongroupShadowColor', T.bg, ...
    'DefaultUibuttongroupBorderType', 'line', ...
    'DefaultUibuttongroupFontName', T.font, ...
    'DefaultAxesColor', T.field, ...
    'DefaultAxesXColor', T.sub, ...
    'DefaultAxesYColor', T.sub, ...
    'DefaultAxesGridColor', [1 1 1], ...
    'DefaultAxesGridAlpha', 0.12, ...
    'DefaultAxesFontName', T.font, ...
    'DefaultTextColor', T.text, ...
    'DefaultTextFontName', T.font);
end

% '#RRGGBB' (no #) -> [r g b] in [0, 1].
function c = hexrgb(h)
c = double([hex2dec(h(1:2)), hex2dec(h(3:4)), hex2dec(h(5:6))]) / 255;
end

% Blend a toward b: w = 0 -> a, w = 1 -> b. Used for the dim pill/chip fills.
function c = mixc(a, b, w)
c = (1 - w) * a + w * b;
end

% First installed font from the candidate list, else the fallback. MATLAB
% resolves 'FixedWidth' to a monospace font on any platform.
function f = pickFont(cands, fallback)
avail = listfonts;
for i = 1:numel(cands)
    if any(strcmpi(avail, cands{i})), f = cands{i}; return; end
end
f = fallback;
end


% Legend click handler: toggle the clicked trail's visibility on/off.
function legendToggleTrail(~, ev)
h = ev.Peer;
if strcmp(get(h, 'Visible'), 'on')
    set(h, 'Visible', 'off');
else
    set(h, 'Visible', 'on');
end
end

% Lay Esri 'satellite' tiles under the local-metre axes (best-effort).
function addSatelliteBasemap(axm, lat0, lon0, HALF)
Re = 6378137; d = pi / 180;
dLat = HALF / (Re * d);
dLon = HALF / (Re * d * cosd(lat0));
latlim = [lat0 - dLat, lat0 + dLat];
lonlim = [lon0 - dLon, lon0 + dLon];
try
    [A, RA] = readBasemapImage('satellite', latlim, lonlim);
    if isprop(RA, 'LatitudeLimits')           % geographic raster reference
        latImg = RA.LatitudeLimits;
        lonImg = RA.LongitudeLimits;
    else                                      % map raster ref (Web Mercator)
        lonImg = RA.XWorldLimits / (Re * d);
        latImg = (2 * atan(exp(RA.YWorldLimits / Re)) - pi/2) / d;
    end
    Eimg = (lonImg - lon0) * d * Re * cosd(lat0);
    Nimg = (latImg - lat0) * d * Re;
    % A row 1 = northern edge -> map it to the larger N (top, YDir normal).
    h = image(axm, 'XData', Eimg, 'YData', [Nimg(2) Nimg(1)], 'CData', A, ...
              'HitTest', 'off', 'PickableParts', 'none');
    uistack(h, 'bottom');                   % keep under markers/grid on reanchor
    setappdata(axm, 'basemap_img', h);
catch ME
    T = gcsTheme();
    set(axm, 'Color', T.field);
    setappdata(axm, 'basemap_img', gobjects(0));
    warning('Mission map: satellite basemap unavailable (%s). Grid only.', ...
            ME.message);
end
end

% Re-anchor the map on a new (lat0, lon0): refresh the basemap + caption.
function setMissionAnchor(mm, lat0, lon0)
if ~isstruct(mm) || ~isfield(mm, 'ax') || ~isvalid(mm.ax), return; end
HALF = getappdata(mm.ax, 'HALF');
setappdata(mm.ax, 'lat0', lat0);
setappdata(mm.ax, 'lon0', lon0);
old = getappdata(mm.ax, 'basemap_img');
if ~isempty(old) && all(isgraphics(old)), delete(old); end
addSatelliteBasemap(mm.ax, lat0, lon0, HALF);
albl = getappdata(mm.ax, 'anchor_lbl');
if ~isempty(albl) && isgraphics(albl)
    set(albl, 'String', sprintf( ...
        'ANCHOR %.5f, %.5f   \\cdot   CLICK MAP TO ADD WAYPOINT', lat0, lon0));
    uistack(albl, 'top');                % keep above the fresh basemap
end
end

% Select the mode toggle-button whose Tag matches modeStr (auto-mode
% transitions); restyle so the engaged mode reads green from any tab.
function setModeButton(mm, modeStr)
if ~isstruct(mm) || ~isfield(mm, 'mode_bg') || ~isvalid(mm.mode_bg), return; end
btn = findobj(mm.mode_bg, 'Tag', modeStr);
if ~isempty(btn)
    set(mm.mode_bg, 'SelectedObject', btn(1));
    restyleModeButtons(mm.mode_bg);
end
end

% Map click -> stash an [N E D] add request for the sim loop.
function onMapClick(axm, fig, altEdit, HALF)
cp = get(axm, 'CurrentPoint');
E = cp(1, 1); N = cp(1, 2);
if abs(E) > HALF || abs(N) > HALF, return; end   % ignore clicks outside box
alt = str2double(get(altEdit, 'String'));
if ~isfinite(alt), alt = 10; end
setappdata(fig, 'map_add_request', [N, E, -alt]);   % NED, D = -altitude
end

% Takeoff-altitude edit: write straight onto the live FlightModeManager
% params (used by the next Takeoff command). Invalid input reverts.
function onTakeoffAltEdit(src, fmm)
v = str2double(get(src, 'String'));
if isfinite(v) && v >= 1 && v <= 500
    fmm.p.auto.takeoff_alt = v;
else
    set(src, 'String', num2str(fmm.p.auto.takeoff_alt));
end
end

% Table Alt-column edit -> stash a [row alt_m] request for the sim loop.
function onAltEdit(fig, ev)
if isempty(ev.Indices), return; end
row = ev.Indices(1);
newAlt = ev.NewData;
if ischar(newAlt) || isstring(newAlt), newAlt = str2double(newAlt); end
setappdata(fig, 'alt_edit_request', [row, double(newAlt)]);
end

% Redraw the Mission-map markers + table from the waypoint list (NED metres).
function updateMissionMap(mm, wps)
if ~isstruct(mm) || ~isfield(mm, 'ax') || ~isvalid(mm.ax), return; end
old = getappdata(mm.ax, 'wp_labels');           % clear old coordinate labels
if ~isempty(old), delete(old(isgraphics(old))); end
if isempty(wps)
    set([mm.path mm.launch mm.mid mm.endp], 'XData', NaN, 'YData', NaN);
    set(mm.table, 'Data', {});
    setappdata(mm.ax, 'wp_labels', gobjects(0));
    return;
end
N = wps(:, 1); E = wps(:, 2); alt = -wps(:, 3);
n = size(wps, 1);
set(mm.path,   'XData', E,    'YData', N);
set(mm.launch, 'XData', E(1), 'YData', N(1));
if n >= 2, set(mm.endp, 'XData', E(end), 'YData', N(end));
else,      set(mm.endp, 'XData', NaN,    'YData', NaN); end
if n >= 3, set(mm.mid, 'XData', E(2:end-1), 'YData', N(2:end-1));
else,      set(mm.mid, 'XData', NaN,        'YData', NaN); end
set(mm.table, 'Data', num2cell([N, E, alt]));

% lat/lon coordinate label at each clicked point (current anchor)
lat0 = getappdata(mm.ax, 'lat0'); lon0 = getappdata(mm.ax, 'lon0');
Re = 6378137; d = pi / 180;
labels = gobjects(n, 1);
for i = 1:n
    lat = lat0 + N(i) / (Re * d);
    lon = lon0 + E(i) / (Re * d * cosd(lat0));
    labels(i) = text(mm.ax, E(i), N(i), ...
        sprintf('  %d: %.6f, %.6f', i, lat, lon), ...
        'Color', 'w', 'FontSize', 7, 'FontWeight', 'bold', ...
        'VerticalAlignment', 'bottom', 'Clipping', 'on', ...
        'HitTest', 'off', 'PickableParts', 'none');
end
setappdata(mm.ax, 'wp_labels', labels);
end


% =========================================================================
% QGroundControl .plan I/O (JSON). Waypoints are converted to/from lat/lon
% with a flat-earth model at the plan's home (load) or the map anchor (save).
% =========================================================================
function onLoadPlan(fig)
mdir = fullfile(fileparts(mfilename('fullpath')), 'missions');
[f, p] = uigetfile({'*.plan;*.json', 'QGC plan (*.plan, *.json)'}, ...
                   'Load mission .plan', [mdir filesep]);
if isequal(f, 0), return; end
try
    data = plan_load(fullfile(p, f));
catch ME
    errordlg(sprintf('Failed to parse plan:\n%s', ME.message), 'Load .plan');
    return;
end
setappdata(fig, 'load_plan_request', data);     % sim loop applies it
end

function savePlanDialog(mm, waypoints)
if isempty(waypoints)
    warndlg('No waypoints to save.', 'Save .plan'); return;
end
lat0 = getappdata(mm.ax, 'lat0'); lon0 = getappdata(mm.ax, 'lon0');
mdir = fullfile(fileparts(mfilename('fullpath')), 'missions');
[f, p] = uiputfile({'*.plan', 'QGC plan (*.plan)'}, 'Save mission .plan', ...
                   fullfile(mdir, 'mission.plan'));
if isequal(f, 0), return; end
try
    plan_save(fullfile(p, f), waypoints, lat0, lon0);
catch ME
    errordlg(sprintf('Failed to save plan:\n%s', ME.message), 'Save .plan');
end
end

% Parse a QGC .plan -> struct(waypoints[Nx3 NED], lat0, lon0). NAV_WAYPOINT
% (16) and TAKEOFF (22) items are converted from lat/lon to NED about the
% plan's plannedHomePosition; items without coordinates (e.g. RTL) are skipped.
function out = plan_load(path)
plan = jsondecode(fileread(path));
m = plan.mission;
home = m.plannedHomePosition;
lat0 = home(1); lon0 = home(2);
items = m.items;
if isstruct(items), items = num2cell(items); end   % homogeneous -> cell
Re = 6378137; d = pi / 180;
wps = zeros(0, 3);
for k = 1:numel(items)
    it = items{k};
    if ~isfield(it, 'command') || ~ismember(it.command, [16 22]), continue; end
    pr = it.params;
    if iscell(pr), lat = cellnum(pr{5}); lon = cellnum(pr{6}); alt = cellnum(pr{7});
    else,          lat = pr(5);          lon = pr(6);          alt = pr(7);        end
    if ~isfinite(lat) || ~isfinite(lon), continue; end
    if ~isfinite(alt) && isfield(it, 'Altitude'), alt = it.Altitude; end
    N = (lat - lat0) * d * Re;
    E = (lon - lon0) * d * Re * cosd(lat0);
    wps(end+1, :) = [N, E, -alt]; %#ok<AGROW>
end
out = struct('waypoints', wps, 'lat0', lat0, 'lon0', lon0);
end

function v = cellnum(x)
if isempty(x), v = NaN; else, v = double(x); end
end

% Write waypoints as a QGC .plan: item 1 = TAKEOFF (22), the rest =
% NAV_WAYPOINT (16), terminated by RTL (20). Frame 3 = global/rel-alt.
function plan_save(path, waypoints, lat0, lon0)
Re = 6378137; d = pi / 180;
n = size(waypoints, 1);
items = cell(1, n + 1);
for k = 1:n
    N = waypoints(k, 1); E = waypoints(k, 2); alt = -waypoints(k, 3);
    lat = lat0 + N / (Re * d);
    lon = lon0 + E / (Re * d * cosd(lat0));
    if k == 1, cmd = 22; else, cmd = 16; end
    items{k} = struct('AMSLAltAboveTerrain', [], 'Altitude', alt, ...
        'AltitudeMode', 1, 'autoContinue', true, 'command', cmd, ...
        'doJumpId', k, 'frame', 3, ...
        'params', {{0, 0, 0, [], lat, lon, alt}}, 'type', 'SimpleItem');
end
items{n + 1} = struct('autoContinue', true, 'command', 20, 'doJumpId', n + 1, ...
    'frame', 2, 'params', {{0, 0, 0, 0, 0, 0, 0}}, 'type', 'SimpleItem');
mission = struct('cruiseSpeed', 15, 'firmwareType', 12, ...
    'globalPlanAltitudeMode', 0, 'hoverSpeed', 5, 'items', {items}, ...
    'plannedHomePosition', [lat0, lon0, 0], 'vehicleType', 2, 'version', 2);
plan = struct('fileType', 'Plan', ...
    'geoFence', struct('circles', {{}}, 'polygons', {{}}, 'version', 2), ...
    'groundStation', 'QGroundControl', 'mission', mission, ...
    'rallyPoints', struct('points', {{}}, 'version', 2), 'version', 1);
txt = jsonencode(plan, 'PrettyPrint', true);
fid = fopen(path, 'w');
if fid < 0, error('cannot open %s for writing', path); end
fwrite(fid, txt); fclose(fid);
end


% =========================================================================
% Autotune tab: amplitude / rise-time inputs, the Start/Cancel button,
% a live status line, and a results panel. The sim loop owns the actual
% start/cancel logic (preconditions) and updates the labels.
% =========================================================================
function [at_btn, amp_edit, rise_edit, status_lbl, results_lbl] = ...
        buildAutotuneTab(parent, fig, at_opts)
T = gcsTheme();
pan = uipanel(parent, 'Units', 'normalized', 'Position', [0.03 0.55 0.94 0.43], ...
    'Title', ' SYSTEM-IDENTIFICATION AUTOTUNE ', 'FontWeight', 'bold', 'FontSize', 8);

uicontrol(pan, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.03 0.80 0.45 0.12], 'ForegroundColor', T.sub, ...
    'HorizontalAlignment', 'left', 'String', 'Inject amplitude MC_AT_SYSID_AMP (rad/s):');
amp_edit = uicontrol(pan, 'Style', 'edit', 'Units', 'normalized', ...
    'Position', [0.50 0.80 0.15 0.13], 'BackgroundColor', T.field, ...
    'FontName', T.mono, 'String', num2str(at_opts.sysid_amp));
uicontrol(pan, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.03 0.62 0.45 0.12], 'ForegroundColor', T.sub, ...
    'HorizontalAlignment', 'left', 'String', 'Desired rise time MC_AT_RISE_TIME (s):');
rise_edit = uicontrol(pan, 'Style', 'edit', 'Units', 'normalized', ...
    'Position', [0.50 0.62 0.15 0.13], 'BackgroundColor', T.field, ...
    'FontName', T.mono, 'String', num2str(at_opts.rise_time));

at_btn = uicontrol(pan, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.70 0.62 0.27 0.30], 'String', 'Start Autotune', ...
    'FontWeight', 'bold', 'BackgroundColor', T.goodbg, 'ForegroundColor', T.good, ...
    'Callback', @(~, ~) setappdata(fig, 'autotune_request', true));

uicontrol(pan, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.03 0.28 0.94 0.26], 'ForegroundColor', T.sub, ...
    'HorizontalAlignment', 'left', 'FontSize', 8, ...
    'String', ['Preconditions (checked at Start): airborne (>1.5 m), low speed, ' ...
               'position/hold mode, sticks centred. A roll/pitch stick deflection ' ...
               'aborts. On success the identified gains are applied to the live ' ...
               'controllers and mirrored onto the Controller tab.']);

status_lbl = uicontrol(pan, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.03 0.04 0.94 0.20], 'BackgroundColor', T.field, ...
    'ForegroundColor', T.warn, ...
    'HorizontalAlignment', 'left', 'FontName', T.mono, 'FontSize', 9, ...
    'String', 'autotune: idle');

results_lbl = uicontrol(parent, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.03 0.03 0.94 0.49], 'BackgroundColor', T.field, ...
    'HorizontalAlignment', 'left', 'FontName', T.mono, 'FontSize', 9, ...
    'String', 'Identified models and tuned gains will appear here after a run.');
end


% --- Setters for the parameter tabs ---
function setEkfParam(ekf, name, v)
ekf.params.(name) = v(1);
end

function setAll(cells, name, v)
for i = 1:numel(cells)
    cells{i}.(name) = v(1);
end
end

function setGnss(cells, names, v)
for i = 1:numel(cells)
    cells{i}.(names{1}) = v(1);
    cells{i}.(names{2}) = v(2);
end
end

function setWindEnable(wind, src)
wind.turb_enable = logical(get(src, 'Value'));
end


% Format the autotune result struct as a fixed-width text block.
function s = autotuneResultText(r)
ax = {'roll ', 'pitch', 'yaw  '};
lines = {sprintf('Final state: %s   (passed verification: %d)', r.state, r.success)};
lines{end+1} = 'Identified model  [b0 b1 b2 | a1 a2]   (scaled)';
for k = 1:3
    c = r.id_coeff(:, k);   % [a1;a2;b0;b1;b2]
    lines{end+1} = sprintf('  %s %+6.3f %+6.3f %+6.3f | %+6.3f %+6.3f', ...
        ax{k}, c(3), c(4), c(5), c(1), c(2)); %#ok<AGROW>
end
lines{end+1} = 'Designed gains   kc       ki       kd     att_p';
for k = 1:3
    lines{end+1} = sprintf('  %s %7.4f %8.4f %8.4f %7.3f', ...
        ax{k}, r.rate_k(k), r.rate_i(k), r.rate_d(k), r.att_p(k)); %#ok<AGROW>
end
s = strjoin(lines, newline);
end


% One tunable: label + N sliders. get() returns the current value(s) in
% display units; set(v) writes them back. lo/hi are slider bounds and def
% is the px4_params default (all in display units). Tip = source PX4 param.
function spec = rowSpec(label, tip, get, set, lo, hi, def)
spec = struct('label', label, 'tip', tip, 'get', get, 'set', set, ...
              'lo', lo, 'hi', hi, 'def', def);
end


% Lay a group of rows into a titled panel at normalized position `pos`,
% with a per-controller "Reset defaults" button along the top.
function group_refresh = buildGroup(parent, pos, title, rows)
T = gcsTheme();
panel = uipanel(parent, 'Units', 'normalized', 'Position', pos, ...
                'Title', [' ' upper(title) ' '], ...
                'FontWeight', 'bold', 'FontSize', 8);
nr          = numel(rows);
reset_fns   = cell(nr, 1);
refresh_fns = cell(nr, 1);

top  = 0.88;                 % rows start below the reset button strip
bot  = 0.015;
rowh = (top - bot) / nr;
for r = 1:nr
    [reset_fns{r}, refresh_fns{r}] = makeRow(panel, top - r * rowh, rowh, rows{r});
end

uicontrol(panel, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.70 0.905 0.28 0.085], 'String', 'RESET DEFAULTS', ...
    'FontSize', 7.5, 'BackgroundColor', T.btn, 'ForegroundColor', T.warn, ...
    'Callback', @(~,~) resetGroup(reset_fns));

% Re-read live values into this group's sliders/edits (no controller write).
group_refresh = @() resetGroup(refresh_fns);
end


% Build one label + N (slider + editable readout) columns inside `panel`.
% Returns a function handle that resets this row to its defaults.
%
% All sliders/edits are created first, THEN the callbacks are wired, so the
% closures capture the fully-populated handle arrays (wiring inside the
% build loop would snapshot still-unassigned GraphicsPlaceholders).
function [reset_fn, refresh_fn] = makeRow(panel, ybot, rowh, spec)
T = gcsTheme();
ncols = 3;                              % grid columns (3-axis params)
uicontrol(panel, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.02 ybot + 0.08*rowh 0.30 0.80*rowh], ...
    'ForegroundColor', T.text, 'HorizontalAlignment', 'left', ...
    'FontSize', 8.5, ...
    'String', spec.label, 'TooltipString', spec.tip);

vals = spec.get(); vals = vals(:);
n    = numel(vals);
lo   = expandToN(spec.lo, n);
hi   = expandToN(spec.hi, n);
ew   = 0.66 / ncols;
sliders = gobjects(n, 1);
edits   = gobjects(n, 1);
for c = 1:n
    x0 = 0.34 + (c-1)*ew;
    sliders(c) = uicontrol(panel, 'Style', 'slider', 'Units', 'normalized', ...
        'Position', [x0, ybot + 0.50*rowh, ew*0.92, 0.40*rowh], ...
        'BackgroundColor', T.field, ...
        'Min', lo(c), 'Max', hi(c), ...
        'Value', min(max(vals(c), lo(c)), hi(c)), 'TooltipString', spec.tip);
    edits(c) = uicontrol(panel, 'Style', 'edit', 'Units', 'normalized', ...
        'Position', [x0, ybot + 0.06*rowh, ew*0.92, 0.40*rowh], ...
        'BackgroundColor', T.field, 'FontName', T.mono, 'FontSize', 8, ...
        'String', num2str(vals(c), '%.4g'), 'TooltipString', spec.tip);
end
% Wire callbacks only after both arrays are fully built.
for c = 1:n
    addlistener(sliders(c), 'ContinuousValueChange', ...
                @(~,~) onSlider(sliders, edits, spec));
    set(sliders(c), 'Callback', @(~,~) onSlider(sliders, edits, spec));
    set(edits(c),   'Callback', @(~,~) onEdit(sliders, edits, spec));
end
reset_fn   = @() resetRow(sliders, edits, spec);
refresh_fn = @() refreshRow(sliders, edits, spec);
end


% Slider moved: read all sliders, mirror into the edit boxes, push to the
% controller. Runs continuously during a drag.
function onSlider(sliders, edits, spec)
n = numel(sliders);
v = zeros(n, 1);
for c = 1:n
    v(c) = get(sliders(c), 'Value');
    set(edits(c), 'String', num2str(v(c), '%.4g'));
end
spec.set(v);
end


% Edit box committed: accept any finite value (not just within the slider
% range) -- if it falls outside, the slider's Min/Max grow to include it so
% the thumb stays consistent. Push the row vector to the controller.
function onEdit(sliders, edits, spec)
n = numel(sliders);
v = zeros(n, 1);
for c = 1:n
    x = str2double(get(edits(c), 'String'));
    if ~isfinite(x)
        x = get(sliders(c), 'Value');           % invalid entry -> revert
    end
    lo = min(get(sliders(c), 'Min'), x);
    hi = max(get(sliders(c), 'Max'), x);
    if hi <= lo, hi = lo + eps; end
    set(sliders(c), 'Min', lo);                  % widen first (old value
    set(sliders(c), 'Max', hi);                  % stays inside [lo,hi]),
    set(sliders(c), 'Value', x);                 % then move the thumb.
    set(edits(c),   'String', num2str(x, '%.4g'));
    v(c) = x;
end
spec.set(v);
end


% Restore one row to its px4_params defaults (sliders, edits, controller).
function resetRow(sliders, edits, spec)
d = expandToN(spec.def, numel(sliders));
for c = 1:numel(sliders)
    lo = min(get(sliders(c), 'Min'), d(c));
    hi = max(get(sliders(c), 'Max'), d(c));
    set(sliders(c), 'Min', lo);
    set(sliders(c), 'Max', hi);
    set(sliders(c), 'Value', d(c));
    set(edits(c),   'String', num2str(d(c), '%.4g'));
end
spec.set(d);
end


% Re-read the row's CURRENT live value into its sliders/edits without
% writing back to the object (used to mirror autotuned gains onto the UI).
function refreshRow(sliders, edits, spec)
v = spec.get(); v = v(:);
for c = 1:numel(sliders)
    lo = min(get(sliders(c), 'Min'), v(c));
    hi = max(get(sliders(c), 'Max'), v(c));
    if hi <= lo, hi = lo + eps; end
    set(sliders(c), 'Min', lo);
    set(sliders(c), 'Max', hi);
    set(sliders(c), 'Value', v(c));
    set(edits(c),   'String', num2str(v(c), '%.4g'));
end
end


% Reset every row in a controller panel to defaults.
function resetGroup(reset_fns)
for i = 1:numel(reset_fns)
    reset_fns{i}();
end
end


% Broadcast a scalar bound/default to an N-vector; pass vectors through.
function out = expandToN(x, n)
x = x(:);
if isscalar(x)
    out = repmat(x, n, 1);
else
    out = x;
end
end


% --- Small setters for controller properties (used by tuning callbacks) ---
function setProp(obj, name, v)
obj.(name) = v(:);
end

function setVelLims(pc, v)
pc.lim_vel_horizontal = v(1);
pc.lim_vel_up         = v(2);
pc.lim_vel_down       = v(3);
end

function setThr(pc, v)
pc.thr_min      = v(1);
pc.hover_thrust = v(2);
pc.thr_max      = v(3);
end

% Attitude P is stored with yaw pre-divided by the yaw weight; reconstruct
% the nominal MC_*_P gains for display and re-apply via setProportionalGain.
function g = attPGet(att_ctl)
g = att_ctl.proportional_gain;
if att_ctl.yaw_w > 1e-4
    g(3) = g(3) * att_ctl.yaw_w;
end
end

function attYawSet(att_ctl, w)
att_ctl.setProportionalGain(attPGet(att_ctl), w);
end

function ctrl_fig_close(fig, ctrl_fig)
if ishandle(fig)
    setappdata(fig, 'running', false);
end
delete(ctrl_fig);
end
