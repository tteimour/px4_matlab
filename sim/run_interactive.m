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
% Tabbed GUI. The 3D scene was removed -- Unity/Cesium now provides the
% visualization, so the tabs are parameter editors plus the Mission map
% (live 2D trails) and the VIO control tab. Manual control lives in a
% separate floating window so you can fly from any tab.
% =====================================================================
fig = figure('Name', 'PX4 interactive — tabbed', ...
             'NumberTitle', 'off', 'Color', 'w', ...
             'Position', [90 70 1320 840]);

tg = uitabgroup(fig, 'Units', 'normalized', 'Position', [0 0 1 1]);
tab_ctrl = uitab(tg, 'Title', 'Controller');
tab_ekf  = uitab(tg, 'Title', 'EKF');
tab_sens = uitab(tg, 'Title', 'Sensors');
tab_wind = uitab(tg, 'Title', 'Wind');
tab_auto = uitab(tg, 'Title', 'Autotune');
tab_cesium = uitab(tg, 'Title', 'Cesium');
tab_vio = uitab(tg, 'Title', 'VIO');
tab_mission = uitab(tg, 'Title', 'Mission');

mode_strings = {'stabilized', 'altitude', 'position', 'hold', ...
                'mission', 'rtl', 'land', 'takeoff'};
default_mode_idx = 3;             % start in Position

% The in-GUI 3D scene was removed (Unity/Cesium renders the vehicle now).
% Ground truth / EKF / VIO are shown as live 2D trails on the Mission tab.

% =====================================================================
% The old "Flight control" cockpit panel was removed from the 3D-UAV tab.
% Mode selection (now big buttons), the waypoint editor, the live state
% readout, and mission save/load all live on the Mission tab instead.
% `mode_bg` / `state_lbl` are created there (buildMissionTab) and grabbed
% from the returned struct below.
% =====================================================================

% =====================================================================
% Manual control lives in a SEPARATE floating window (ctrl_fig) so you can
% fly the drone from ANY tab -- the joysticks used to sit on the 3D tab and
% vanished when you switched tabs. The drag callbacks are wired to ctrl_fig;
% the sim loop reads left_h/right_h regardless of which figure holds them.
% Reset/Stop are here too (they act on the main figure `fig`). Closing this
% window stops the sim.
% =====================================================================
ctrl_fig = figure('Name', 'Manual control (works on any tab)', ...
    'NumberTitle', 'off', 'MenuBar', 'none', 'ToolBar', 'none', ...
    'Color', 'w', 'Position', [60 90 480 300], ...
    'CloseRequestFcn', @(~,~) setappdata(fig, 'running', false));
ax_left = axes('Parent', ctrl_fig, 'Units', 'normalized', ...
               'Position', [0.06 0.32 0.40 0.60]);
ax_right = axes('Parent', ctrl_fig, 'Units', 'normalized', ...
                'Position', [0.54 0.32 0.40 0.60]);
[left_h, right_h] = makeJoysticks(ctrl_fig, ax_left, ax_right);

uicontrol(ctrl_fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.06 0.06 0.40 0.18], 'String', 'Reset', 'FontWeight', 'bold', ...
    'Callback', @(~,~) setappdata(fig, 'reset_request', true));
uicontrol(ctrl_fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.54 0.06 0.40 0.18], 'String', 'Stop', 'FontWeight', 'bold', ...
    'BackgroundColor', [0.95 0.85 0.85], ...
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
vio_ui = buildVioTab(tab_vio, fig);   % VIO control tab (enable, params, streams)
vio_cb = vio_ui.vio_cb;               % the loop drives VIO off this checkbox

% Mission tab: north-up satellite map at the Cesium origin (Baku); click to
% drop waypoints, set per-waypoint altitude. Writes the same `waypoints`
% array (via map_add_request / alt_edit_request) the cockpit editor uses.
mission_map = buildMissionTab(tab_mission, fig, mode_strings);
state_lbl = mission_map.state_lbl;   % live state readout now lives on Mission tab
mode_bg   = mission_map.mode_bg;     % flight-mode buttons (replace the dropdown)

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

imu_pub_hz = 200;            % OpenVINS IMU stream rate [Hz] (decimated from 1 kHz)
imu_pub_dt = 1 / imu_pub_hz;

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
setappdata(fig, 'autotune_request', false);
setappdata(fig, 'map_add_request', []);         % [N E D] from a Mission-map click
setappdata(fig, 'alt_edit_request', []);        % [row alt_m] from the Mission table
setappdata(fig, 'load_plan_request', []);       % struct(waypoints,lat0,lon0) from a .plan
setappdata(fig, 'save_plan_request', false);    % Save-.plan button
waypoints = zeros(0, 3);                        % Nx3 [N E D]
sticks = struct('left_x', 0, 'left_y', 0, 'right_x', 0, 'right_y', 0);

k     = 0;
t_sim = 0;
vio_ui_tick = 0;       % throttles the (heavy) VIO image decode so joysticks stay snappy
cesium_bridge  = [];   % lazily created when the Cesium toggle is first enabled
imu_bridge     = [];   % lazily created with cesium_bridge; streams IMU to OpenVINS
last_imu_pub_t = -inf; % sim-time of last published IMU sample (200 Hz decimation)
imu_stream_failed = false; % latched on IMU publish failure to stop recreate churn

% --- VIO comparison logging (Cesium-tab toggle) -----------------------
vio_node       = [];      % ROS 2 node for the OpenVINS odom subscriber
vio_sub        = [];      % subscriber to /ov_msckf/odomimu
vio_sub_raw    = [];      % lazy: /down_cam/image_raw (only while VIO tab open)
vio_sub_trk    = [];      % lazy: /ov_msckf/trackhist (only while VIO tab open)
openvins_pid   = [];      % OpenVINS launch PID (auto-started with the VIO toggle)
vio_align_R    = [];      % frozen SE3 rotation OpenVINS-global -> NED (map trail)
vio_align_t    = [];      % frozen SE3 translation (map trail)
vio_logging    = false;   % true while the VIO toggle is on
prev_vio_on    = false;   % edge detection for the VIO toggle
vio_idx        = 0;       % rows logged this VIO session
vio_last_stamp = -inf;    % last logged VIO message sim-time stamp (dedup)
vio_overflow_warned = false;  % warn-once when the VIO buffer fills
vio_log = struct('t', nan(max_log, 1), 'vt', nan(max_log, 1), ...
    'gt_pos',  nan(max_log, 3), 'gt_vel',  nan(max_log, 3), ...
    'ekf_pos', nan(max_log, 3), 'ekf_vel', nan(max_log, 3), ...
    'vio_pos', nan(max_log, 3), 'vio_vel', nan(max_log, 3));

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
        % restarts cleanly at 0 and fresh IMU samples flow again (required for
        % the VIO IMU bridge, and fixes the pre-existing estimator stall on Reset).
        est_bus.reset();
        % Re-inject the known bias after reset so post-Reset runs
        % have the same truth bias to estimate.
        est_bus.sensors.applyImuBias(true_gyro_bias, true_accel_bias);
        t_sim   = 0;
        last_imu_pub_t    = -inf;  % sim time restarts at 0; re-arm IMU decimation
        imu_stream_failed = false; % re-arm IMU bridge after a Reset
        if vio_logging             % sim clock restarts -> VIO can't span Reset
            vio_logging = false; set(vio_cb, 'Value', 0); prev_vio_on = false;
            if ~isempty(vio_node) && isvalid(vio_node), delete(vio_node); end
            vio_node = []; vio_sub = []; vio_sub_raw = []; vio_sub_trk = [];
            stopOpenvins(openvins_pid); openvins_pid = [];
            vio_align_R = []; vio_align_t = [];
            clearpoints(mission_map.gt_trail);  clearpoints(mission_map.ekf_trail);
            clearpoints(mission_map.vio_trail);
            fprintf('VIO logging stopped by Reset (sim clock restarted).\n');
        end
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
    % VIO streaming on? Read once per frame (the substep loop runs at ~1 kHz;
    % avoid a GUI read per substep). Gates the IMU publish inside the loop.
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

        % --- Stream IMU to OpenVINS at imu_pub_hz (sim-time stamped) --------
        % Decimate the 1 kHz voted IMU to ~200 Hz. imu.t is the sample's sim
        % time -- the same clock the Cesium pose (and thus the Unity camera)
        % carry -- so camera and IMU stay in one clock domain for VIO.
        if stream_on && ~isempty(imu_bridge)
            imu = est_bus.sensors.vehicleImu();
            if ~isempty(imu) && isfield(imu, 't') && ...
                    imu.t >= last_imu_pub_t + imu_pub_dt - 1e-9
                try
                    imu_bridge.publish(imu.gyro_b, imu.accel_b, imu.t);
                    last_imu_pub_t = imu.t;
                catch ME
                    warning('IMU bridge publish failed (%s). Disabling.', ME.message);
                    delete(imu_bridge);
                    imu_bridge = [];
                    imu_stream_failed = true;  % stop per-frame recreate churn
                end
            end
        end

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

        % Landed: ground + slow + commanded altitude target near ground.
        % Use ground truth so estimator noise doesn't cause hover flapping.
        landed = (s_truth.position_ned(3) > -0.05) && ...
                 (norm(s_truth.velocity_ned) < 0.3) && ...
                 (cmd.pos_sp(3) > -0.10);
        torque = rate_ctl.update(s.angular_vel_b, rate_sp_cmd, [0;0;0], dt_rate, landed);
        T_mag  = max(0, -thrust_body_z);
        [m, sat_pos, sat_neg] = alloc.allocate(torque, T_mag);
        rate_ctl.setSaturationStatus(sat_pos, sat_neg);
        m_last = m;

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
        set(at_btn, 'String', 'Cancel Autotune');
    else
        set(at_btn, 'String', 'Start Autotune');
    end
    set(at_status_lbl, 'String', ['autotune: ' autotune_status]);

    % --- State for the readout + bridges (no in-GUI 3D view; Unity renders) ---
    s = plant.state();
    eN = s.position_ned(1);
    eE = s.position_ned(2);
    eU = max(0, -s.position_ned(3));
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
        % IMU bridge to OpenVINS (native DDS, no rosbridge). Created with the
        % Cesium toggle so the VIO pipeline (Unity camera + MATLAB IMU) comes
        % up together; the substep loop above does the 200 Hz publishing.
        if isempty(imu_bridge) && ~imu_stream_failed
            try
                imu_bridge = ImuBridge();
                last_imu_pub_t = -inf;
                fprintf('IMU bridge: publishing to %s @ %d Hz\n', ...
                        imu_bridge.Topic, imu_pub_hz);
            catch ME
                warning('IMU bridge failed to start (%s). Disabling.', ME.message);
                imu_bridge = [];
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

    % --- VIO logging + comparison (Cesium-tab toggle) ---------------------
    % On enable, subscribe to /ov_msckf/odomimu and from this frame on record
    % ground truth, EKF and VIO (OpenVINS global frame) every frame. On
    % disable (or Stop, in teardown) plot GT vs EKF vs VIO. "Start from the
    % latest ground-truth point" = logging begins now and the VIO trajectory
    % is rigidly aligned to GT over the logged window at plot time.
    vio_on = ishandle(vio_cb) && get(vio_cb, 'Value') == 1;
    if vio_on && ~prev_vio_on                 % rising edge: start a session
        if isempty(openvins_pid)              % write params, then launch OpenVINS
            writeVioParams(vio_ui);           % apply the VIO-tab params to the config
            openvins_pid = startOpenvins();
        end
        if isempty(vio_node)                  % gate on node (sub may be stale)
            try
                vio_node = ros2node('/px4_matlab_vio', 0);
                vio_sub  = ros2subscriber(vio_node, '/ov_msckf/odomimu', ...
                    'nav_msgs/Odometry', @(m) onVioMessage(fig, m));
                % Image subscribers are created lazily below, only while the VIO
                % tab is open -- deserializing ~60 frames/s on MATLAB's single
                % thread is what froze the GUI when flying on other tabs.
            catch ME
                warning('VIO subscriber failed to start (%s). Disabling.', ME.message);
                if ~isempty(vio_node) && isvalid(vio_node), delete(vio_node); end
                vio_node = []; vio_sub = []; vio_idx = 0; set(vio_cb, 'Value', 0);
            end
        end
        if ~isempty(vio_sub)
            setappdata(fig, 'vio_latest', []);   % drop any stale message
            vio_idx = 0; vio_last_stamp = -inf; vio_overflow_warned = false;
            vio_logging = true;
            vio_align_R = []; vio_align_t = [];  % fresh map-trail alignment
            clearpoints(mission_map.gt_trail);  clearpoints(mission_map.ekf_trail);
            clearpoints(mission_map.vio_trail);
            fprintf('VIO logging started at t=%.2f s (anchored at ground truth).\n', t_sim);
        end
    elseif ~vio_on && prev_vio_on             % falling edge: stop, plot, free
        vio_logging = false;
        plotVioComparison(vio_log, vio_idx);
        if ~isempty(vio_node) && isvalid(vio_node), delete(vio_node); end
        vio_node = []; vio_sub = [];          % so re-enable rebuilds cleanly
        vio_sub_raw = []; vio_sub_trk = [];   % image subs dropped with the node
        stopOpenvins(openvins_pid); openvins_pid = [];
    end
    prev_vio_on = vio_on;

    % --- lazy image subscriptions: only while the VIO tab is open ----------
    % Deserializing two image streams (~60 msg/s of 512x512) on the main thread
    % is what froze the GUI. Subscribe only when you are looking at them.
    want_vio_imgs = vio_logging && ~isempty(vio_node) && isvalid(vio_node) && ...
                    ishandle(tg) && tg.SelectedTab == tab_vio;
    if want_vio_imgs && isempty(vio_sub_raw)
        try
            setappdata(fig, 'vio_img_raw', []); setappdata(fig, 'vio_img_trk', []);
            vio_sub_raw = ros2subscriber(vio_node, '/down_cam/image_raw', ...
                'sensor_msgs/Image', @(m) onVioImage(fig, 'vio_img_raw', m));
            vio_sub_trk = ros2subscriber(vio_node, '/ov_msckf/trackhist', ...
                'sensor_msgs/Image', @(m) onVioImage(fig, 'vio_img_trk', m));
        catch
            vio_sub_raw = []; vio_sub_trk = [];
        end
    elseif ~want_vio_imgs && ~isempty(vio_sub_raw)
        try, delete(vio_sub_raw); catch, end %#ok<NOCOM>
        try, delete(vio_sub_trk); catch, end %#ok<NOCOM>
        vio_sub_raw = []; vio_sub_trk = [];
    end

    if vio_logging && ~isempty(vio_sub)
        vmsg = getappdata(fig, 'vio_latest');
        if ~isempty(vmsg)
            vstamp = double(vmsg.header.stamp.sec) + ...
                     double(vmsg.header.stamp.nanosec) * 1e-9;
            vp = [vmsg.pose.pose.position.x, vmsg.pose.pose.position.y, ...
                  vmsg.pose.pose.position.z];
            vv = [vmsg.twist.twist.linear.x, vmsg.twist.twist.linear.y, ...
                  vmsg.twist.twist.linear.z];
            if vstamp > vio_last_stamp && all(isfinite([vstamp, vp, vv]))
                if vio_idx >= max_log
                    if ~vio_overflow_warned
                        warning('VIO log buffer full (%d samples); dropping the rest.', max_log);
                        vio_overflow_warned = true;
                    end
                else
                    % GT/EKF sampled now (t_sim); VIO carries its own sim-time
                    % stamp (vt) -- aligned by interpolation at plot time.
                    gtn  = plant.state();
                    estn = est_bus.stateOut();
                    vio_idx = vio_idx + 1;
                    vio_log.t(vio_idx)          = t_sim;
                    vio_log.vt(vio_idx)         = vstamp;
                    vio_log.gt_pos(vio_idx, :)  = gtn.position_ned';
                    vio_log.gt_vel(vio_idx, :)  = gtn.velocity_ned';
                    vio_log.ekf_pos(vio_idx, :) = estn.position_ned';
                    vio_log.ekf_vel(vio_idx, :) = estn.velocity_ned';
                    vio_log.vio_pos(vio_idx, :) = vp;
                    vio_log.vio_vel(vio_idx, :) = vv;
                    vio_last_stamp = vstamp;

                    % --- live 2D map trails (Mission tab) ------------------
                    % GT (white) + EKF (cyan) appended every sample; VIO (red)
                    % once a frozen GT alignment exists -- so VIO divergence
                    % shows as the red trail drifting off GT.
                    addpoints(mission_map.gt_trail,  gtn.position_ned(2),  gtn.position_ned(1));
                    addpoints(mission_map.ekf_trail, estn.position_ned(2), estn.position_ned(1));
                    if ~isempty(vio_align_R)
                        a = vio_align_R * vp.' + vio_align_t;
                        addpoints(mission_map.vio_trail, a(2), a(1));
                    elseif vio_idx >= 80 && mod(vio_idx, 10) == 0
                        gn = vio_log.gt_pos(1:vio_idx, 1:2);
                        if max(max(gn, [], 1) - min(gn, [], 1)) > 2   % moved enough to fix yaw
                            [vio_align_R, vio_align_t] = umeyama_align( ...
                                vio_log.vio_pos(1:vio_idx, :).', vio_log.gt_pos(1:vio_idx, :).');
                            al = (vio_align_R * vio_log.vio_pos(1:vio_idx, :).' + vio_align_t).';
                            clearpoints(mission_map.vio_trail);
                            addpoints(mission_map.vio_trail, al(:, 2), al(:, 1));
                        end
                    end
                end
            end
        end
    end

    % --- refresh the VIO tab (camera streams + live odom readout) ----------
    % Decoding two 512x512 frames every loop iteration starves MATLAB's single
    % thread and makes the joysticks lag. So only do it when the VIO tab is
    % actually visible, throttled to ~every 4th frame, flushing queued mouse
    % events right after the heavy decode.
    if vio_logging && ishandle(tg) && tg.SelectedTab == tab_vio
        vio_ui_tick = vio_ui_tick + 1;
        if mod(vio_ui_tick, 4) == 0
            updateVioImages(vio_ui, fig);
            updateVioReadout(vio_ui, getappdata(fig, 'vio_latest'), vio_idx, ~isempty(vio_align_R));
            drawnow limitrate;   % process queued joystick/mouse events after decode
        end
    end

    rpy    = quat_to_euler(s.attitude_q);
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

    set(state_lbl, 'String', sprintf( ...
        ['Mode    %s\n' ...
         'pos     N=%+6.2f  E=%+6.2f  Alt=%5.2f\n' ...
         'pos_sp  N=%+6.2f  E=%+6.2f  Alt=%5.2f\n' ...
         'vel     %5.2f m/s\n' ...
         'yaw     %+6.1f deg   yaw_sp %+6.1f\n' ...
         'sticks  L=(%+.2f,%+.2f)  R=(%+.2f,%+.2f)\n' ...
         '%s'], ...
        prev_mode, eN, eE, eU, eN_sp, eE_sp, eU_sp, ...
        norm(s.velocity_ned), rad2deg(rpy(3)), rad2deg(cmd.yaw_sp), ...
        sticks.left_x, sticks.left_y, sticks.right_x, sticks.right_y, ...
        autotune_status));

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
% Tear down the OpenVINS IMU ROS 2 node if it was started.
if ~isempty(imu_bridge) && isvalid(imu_bridge)
    delete(imu_bridge);
end

% VIO comparison: if logging was still on at Stop, plot before tearing down.
if vio_logging
    plotVioComparison(vio_log, vio_idx);
end
if ~isempty(vio_node) && isvalid(vio_node)
    delete(vio_node);     % also drops vio_sub
end
% Stop OpenVINS if the toggle was still on at Stop (no-op if we did not start it).
stopOpenvins(openvins_pid);
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

est_cb = uicontrol(parent, 'Style', 'checkbox', 'Units', 'normalized', ...
    'Position', [0.05 0.09 0.9 0.05], 'BackgroundColor', 'w', 'Value', 1, ...
    'FontWeight', 'bold', 'String', 'Use EKF2 estimator (sensor-driven state feed)');
uicontrol(parent, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.05 0.02 0.9 0.06], 'BackgroundColor', 'w', ...
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
    'Position', [0.05 0.07 0.9 0.06], 'BackgroundColor', 'w', ...
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
cesium_cb = uicontrol(parent, 'Style', 'checkbox', 'Units', 'normalized', ...
    'Position', [0.05 0.91 0.9 0.05], 'BackgroundColor', 'w', 'Value', 1, ...
    'FontWeight', 'bold', 'String', 'Stream pose to Cesium/Unity (ROS 2)');
uicontrol(parent, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.05 0.56 0.9 0.30], 'BackgroundColor', 'w', ...
    'HorizontalAlignment', 'left', 'FontSize', 9, ...
    'String', sprintf(['Pose stream: ground-truth pose as geometry_msgs/' ...
        'PoseArray on /world/default/pose/info (50 Hz). rosbridge_server is ' ...
        'auto-launched when run_interactive starts and stopped on Stop ' ...
        '(log: /tmp/px4_rosbridge.log).\n\n' ...
        'VIO logging + control moved to the VIO tab.']));
end


% =========================================================================
% VIO tab: enable checkbox, editable OpenVINS parameters (written to the
% config and applied when VIO is enabled -> OpenVINS relaunches), a live odom
% readout (/ov_msckf/odomimu), and the two image streams side by side
% (/down_cam/image_raw + /ov_msckf/trackhist). Returns a struct of handles;
% the sim loop drives everything off vio_ui.vio_cb.
% =========================================================================
function vio_ui = buildVioTab(parent, fig)
[estCfg, camCfg] = vioCfgPaths();

vio_cb = uicontrol(parent, 'Style', 'checkbox', 'Units', 'normalized', ...
    'Position', [0.02 0.945 0.6 0.04], 'BackgroundColor', 'w', 'Value', 0, ...
    'FontWeight', 'bold', 'FontSize', 11, ...
    'String', 'Enable VIO  (auto-launch OpenVINS + log + plot)');
uicontrol(parent, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.02 0.90 0.96 0.04], 'BackgroundColor', 'w', ...
    'HorizontalAlignment', 'left', 'FontSize', 8, ...
    'String', ['Params are written to the OpenVINS config and applied on Enable ' ...
        '(OpenVINS relaunches). Tip: fly up to ~30 m, THEN enable -- a wider ground ' ...
        'footprint = many more features (this is why Gazebo worked). Lower ' ...
        'fast_threshold and CLAHE find more features on smooth terrain.']);

% --- editable parameters (left panel) -------------------------------------
pan = uipanel(parent, 'Units', 'normalized', 'Position', [0.02 0.30 0.40 0.585], ...
    'Title', 'OpenVINS parameters (applied on Enable)', 'BackgroundColor', 'w', ...
    'FontWeight', 'bold');
ed = struct();
ed.num_pts = vioParamRow(pan, 0.88, 'num features (num\_pts)',  readYamlScalar(estCfg, 'num_pts'));
ed.fast    = vioParamRow(pan, 0.795,'fast\_threshold (lower=more)', readYamlScalar(estCfg, 'fast_threshold'));
ed.grid_x  = vioParamRow(pan, 0.71, 'grid\_x',        readYamlScalar(estCfg, 'grid_x'));
ed.grid_y  = vioParamRow(pan, 0.625,'grid\_y',        readYamlScalar(estCfg, 'grid_y'));
ed.min_px  = vioParamRow(pan, 0.54, 'min\_px\_dist',  readYamlScalar(estCfg, 'min_px_dist'));
intr = readYamlArray(camCfg, 'intrinsics');   % [fx fy cx cy]
if numel(intr) < 4, intr = [394.2 394.2 256 256]; end
ed.fx = vioParamRow(pan, 0.455, 'fx',  num2str(intr(1)));
ed.fy = vioParamRow(pan, 0.37,  'fy',  num2str(intr(2)));
ed.cx = vioParamRow(pan, 0.285, 'cx',  num2str(intr(3)));
ed.cy = vioParamRow(pan, 0.20,  'cy',  num2str(intr(4)));
% init method + histogram as dropdowns
uicontrol(pan, 'Style', 'text', 'Units', 'normalized', 'Position', [0.04 0.105 0.44 0.055], ...
    'BackgroundColor', 'w', 'HorizontalAlignment', 'left', 'FontSize', 9, 'String', 'init method');
initDyn = strcmpi(strtrim(readYamlScalar(estCfg, 'init_dyn_use')), 'true');
ed.init = uicontrol(pan, 'Style', 'popupmenu', 'Units', 'normalized', ...
    'Position', [0.50 0.11 0.44 0.06], 'String', {'dynamic', 'static'}, ...
    'Value', 1 + ~initDyn, 'BackgroundColor', 'w');
uicontrol(pan, 'Style', 'text', 'Units', 'normalized', 'Position', [0.04 0.02 0.44 0.055], ...
    'BackgroundColor', 'w', 'HorizontalAlignment', 'left', 'FontSize', 9, 'String', 'histogram');
hm = upper(strrep(strtrim(readYamlScalar(estCfg, 'histogram_method')), '"', ''));
hopts = {'NONE', 'HISTOGRAM', 'CLAHE'}; hidx = find(strcmp(hopts, hm), 1);
if isempty(hidx), hidx = 2; end
ed.hist = uicontrol(pan, 'Style', 'popupmenu', 'Units', 'normalized', ...
    'Position', [0.50 0.025 0.44 0.06], 'String', hopts, 'Value', hidx, 'BackgroundColor', 'w');

% --- live odom readout (left, below the panel) ----------------------------
readout = uicontrol(parent, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.02 0.04 0.40 0.24], 'BackgroundColor', [0.97 0.97 0.97], ...
    'HorizontalAlignment', 'left', 'FontName', 'Courier New', 'FontSize', 9, ...
    'String', 'VIO odom: (enable VIO to start)');

% --- two image streams side by side (right) -------------------------------
axRaw = axes('Parent', parent, 'Units', 'normalized', 'Position', [0.45 0.34 0.26 0.50]);
imgRaw = image(axRaw, zeros(2, 2, 3, 'uint8')); axis(axRaw, 'image', 'off');
title(axRaw, '/down\_cam/image\_raw', 'FontSize', 9);
axTrk = axes('Parent', parent, 'Units', 'normalized', 'Position', [0.72 0.34 0.26 0.50]);
imgTrk = image(axTrk, zeros(2, 2, 3, 'uint8')); axis(axTrk, 'image', 'off');
title(axTrk, '/ov\_msckf/trackhist (features)', 'FontSize', 9);
uicontrol(parent, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.45 0.04 0.53 0.26], 'BackgroundColor', 'w', ...
    'HorizontalAlignment', 'left', 'FontSize', 8, ...
    'String', ['Left: raw camera Unity sends OpenVINS. Right: OpenVINS'' tracked ' ...
        'features. If the right view has few/no points, the camera is feature-' ...
        'starved (fly higher, lower fast_threshold, or CLAHE).']);

vio_ui = struct('vio_cb', vio_cb, 'ed', ed, 'readout', readout, ...
    'axRaw', axRaw, 'imgRaw', imgRaw, 'axTrk', axTrk, 'imgTrk', imgTrk);
end

% One "label + edit" row inside the VIO parameter panel; returns the edit handle.
function h = vioParamRow(pan, y, lbl, val)
uicontrol(pan, 'Style', 'text', 'Units', 'normalized', 'Position', [0.04 y 0.44 0.055], ...
    'BackgroundColor', 'w', 'HorizontalAlignment', 'left', 'FontSize', 9, 'String', lbl);
h = uicontrol(pan, 'Style', 'edit', 'Units', 'normalized', 'Position', [0.50 y+0.005 0.44 0.06], ...
    'BackgroundColor', [0.99 0.99 0.97], 'String', val);
end

% Absolute paths of the two OpenVINS config files the launch reads.
function [estCfg, camCfg] = vioCfgPaths()
base = '/home/teymur/ytu_thesis/simulation/open_vins/config/matlab_unity';
estCfg = fullfile(base, 'estimator_config.yaml');
camCfg = fullfile(base, 'kalibr_imucam_chain.yaml');
end

% Read a scalar YAML value as a char (e.g. '200', 'true', '"CLAHE"'); '' if absent.
function v = readYamlScalar(file, key)
v = '';
if ~isfile(file), return; end
lines = readlines(file);
for i = 1:numel(lines)
    tok = regexp(lines(i), "^\s*" + key + ":\s*(\S+)", 'tokens', 'once');
    if ~isempty(tok), v = char(tok(1)); return; end
end
end

% Read a YAML array "key: [a, b, c]" as a numeric row vector; [] if absent.
function v = readYamlArray(file, key)
v = [];
if ~isfile(file), return; end
lines = readlines(file);
for i = 1:numel(lines)
    tok = regexp(lines(i), "^\s*" + key + ":\s*\[([^\]]*)\]", 'tokens', 'once');
    if ~isempty(tok), v = str2double(strsplit(char(tok(1)), ',')); return; end
end
end

% Replace the value of "key: VALUE  # comment" in-place, preserving the comment.
% val may be numeric or char. Only the first matching line is changed.
function setYamlScalar(file, key, val)
if isnumeric(val), valstr = num2str(val); else, valstr = char(val); end
lines = readlines(file);
for i = 1:numel(lines)
    if ~isempty(regexp(lines(i), "^\s*" + key + ":(\s|$)", 'once'))
        lines(i) = regexprep(lines(i), "^(\s*" + key + ":\s*)\S+", "$1" + valstr, 'once');
        writelines(lines, file);
        return;
    end
end
end

% Replace "key: [ ... ]" with the given numeric vector, preserving the comment.
function setYamlArray(file, key, vals)
s = "[" + strjoin(string(vals), ", ") + "]";
lines = readlines(file);
for i = 1:numel(lines)
    if ~isempty(regexp(lines(i), "^\s*" + key + ":\s*\[", 'once'))
        lines(i) = regexprep(lines(i), "(^\s*" + key + ":\s*)\[[^\]]*\]", "$1" + s, 'once');
        writelines(lines, file);
        return;
    end
end
end

% Write the VIO-tab parameter fields into the OpenVINS config files. Called on
% VIO enable, just before OpenVINS launches, so the values take effect.
function writeVioParams(vio_ui)
[estCfg, camCfg] = vioCfgPaths();
gn = @(h, d) vioFieldNum(h, d);
setYamlScalar(estCfg, 'num_pts',        gn(vio_ui.ed.num_pts, 200));
setYamlScalar(estCfg, 'fast_threshold', gn(vio_ui.ed.fast,    15));
setYamlScalar(estCfg, 'grid_x',         gn(vio_ui.ed.grid_x,  16));
setYamlScalar(estCfg, 'grid_y',         gn(vio_ui.ed.grid_y,  16));
setYamlScalar(estCfg, 'min_px_dist',    gn(vio_ui.ed.min_px,  12));
initStrs = get(vio_ui.ed.init, 'String');
isDyn = strcmp(initStrs{get(vio_ui.ed.init, 'Value')}, 'dynamic');
setYamlScalar(estCfg, 'init_dyn_use', char("" + string(isDyn)));   % 'true'/'false'
histStrs = get(vio_ui.ed.hist, 'String');
setYamlScalar(estCfg, 'histogram_method', ['"' histStrs{get(vio_ui.ed.hist, 'Value')} '"']);
setYamlArray(camCfg, 'intrinsics', [gn(vio_ui.ed.fx, 394.2), gn(vio_ui.ed.fy, 394.2), ...
    gn(vio_ui.ed.cx, 256), gn(vio_ui.ed.cy, 256)]);
fprintf('VIO params written to OpenVINS config.\n');
end

function v = vioFieldNum(h, def)
v = str2double(get(h, 'String'));
if ~isfinite(v), v = def; end
end

% Guarded image-subscriber callback: stash latest image, skip if fig is gone.
function onVioImage(fig, key, m)
if ishandle(fig), setappdata(fig, key, m); end
end

% sensor_msgs/Image struct -> HxWx3 uint8 (mono replicated to RGB).
function im = decodeRosImage(msg)
if isempty(msg) || ~isfield(msg, 'width') || double(msg.width) == 0
    im = zeros(2, 2, 3, 'uint8'); return;
end
try
    im = rosReadImage(msg);
    if size(im, 3) == 1, im = repmat(im, [1 1 3]); end
catch
    w = double(msg.width); h = double(msg.height); d = uint8(msg.data(:));
    g = reshape(d(1:min(w*h, numel(d))), w, []).';
    im = repmat(g, [1 1 3]);
end
end

% Refresh the two image axes from the latest cached frames. Decodes only when
% a NEW frame arrived (loop runs faster than the camera), keyed on the stamp.
function updateVioImages(vio_ui, fig)
refreshVioImage(fig, 'vio_img_raw', 'vio_img_raw_st', vio_ui.imgRaw, vio_ui.axRaw);
refreshVioImage(fig, 'vio_img_trk', 'vio_img_trk_st', vio_ui.imgTrk, vio_ui.axTrk);
end

function refreshVioImage(fig, key, stKey, imgH, axH)
m = getappdata(fig, key);
if isempty(m) || ~isfield(m, 'header'), return; end
st = double(m.header.stamp.sec) + double(m.header.stamp.nanosec) * 1e-9;
if isequal(st, getappdata(fig, stKey)), return; end   % unchanged -> skip decode
im = decodeRosImage(m);
set(imgH, 'CData', im);
set(axH, 'XLim', [0.5, size(im, 2) + 0.5], 'YLim', [0.5, size(im, 1) + 0.5]);
setappdata(fig, stKey, st);
end

% Refresh the odom readout from the latest /ov_msckf/odomimu message.
function updateVioReadout(vio_ui, vmsg, n, aligned)
if isempty(vmsg)
    set(vio_ui.readout, 'String', sprintf(['VIO odom (/ov_msckf/odomimu)\n' ...
        'waiting for OpenVINS to publish...\nsamples logged: %d'], n));
    return;
end
p = [vmsg.pose.pose.position.x, vmsg.pose.pose.position.y, vmsg.pose.pose.position.z];
v = [vmsg.twist.twist.linear.x, vmsg.twist.twist.linear.y, vmsg.twist.twist.linear.z];
if aligned, astr = 'locked'; else, astr = 'pending motion'; end
set(vio_ui.readout, 'String', sprintf(['VIO odom (/ov_msckf/odomimu)\n' ...
    'pos[global] x=%+8.2f\n            y=%+8.2f\n            z=%+8.2f m\n' ...
    'speed |v| = %6.2f m/s\n' ...
    'samples logged: %d\n' ...
    'map-trail align: %s'], p(1), p(2), p(3), norm(v), n, astr));
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
    warning('rosbridge auto-launch is wired for Linux only; start it manually.');
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
% OpenVINS lifecycle. Auto-launched when the VIO toggle is ticked (sources
% ROS 2 + the two OpenVINS overlays, then ros2 launch the matlab_unity
% pipeline) and SIGINT'd when it is unticked / on Stop. Clean-slate: kills any
% prior OpenVINS pipeline first -- a stray run_subscribe_msckf (e.g. an
% ov_msckf launched WITHOUT its image bridge) otherwise silently blocks the
% launch and starves the camera. Needs the IMU bridge (Cesium toggle) + Unity
% camera up, and motion + features to initialise before /ov_msckf/odomimu flows.
% =========================================================================
function pid = startOpenvins()
pid = [];
if ~isunix
    warning('OpenVINS auto-launch is wired for Linux only; launch it manually.');
    return;
end
% clean slate: drop any prior pipeline (strays block the relaunch + image bridge)
system(['pkill -KILL -f run_subscribe_msckf 2>/dev/null; ' ...
        'pkill -KILL -f unity_image_bridge 2>/dev/null; ' ...
        'pkill -KILL -f "ros2 launch openvins" 2>/dev/null; true']);
ws1 = '/home/teymur/ytu_thesis/simulation/open_vins/install/setup.bash';
ws2 = '/home/teymur/ytu_thesis/simulation/openvins_ws/install/setup.bash';
cmd = ['bash -lc ''unset LD_LIBRARY_PATH; ' ...
       'source /opt/ros/humble/setup.bash && source ' ws1 ' && source ' ws2 ' && ' ...
       'exec ros2 launch openvins_matlab_bridge openvins_matlab_unity.launch.py ' ...
       '>/tmp/px4_openvins.log 2>&1 & echo $!'''];
[st, out] = system(cmd);
pidnum = str2double(strtrim(out));
if st == 0 && isfinite(pidnum) && pidnum > 0
    pid = pidnum;
    fprintf('Started OpenVINS (pid %d). Log: /tmp/px4_openvins.log\n', pid);
    fprintf(['  needs Unity Quba playing (camera) + enough features: fly ~30 m,\n' ...
             '  reload Unity for 30 fps, lower fast_threshold / use CLAHE on the VIO tab.\n']);
else
    warning('Could not auto-launch OpenVINS (see /tmp/px4_openvins.log).');
end
end

function stopOpenvins(pid)
if isempty(pid) || ~isfinite(pid) || pid <= 0, return; end
% Only SIGINT if the PID is STILL the OpenVINS launch (guards PID reuse).
cmd = sprintf(['ps -p %d -o args= 2>/dev/null | grep -qE "openvins|ros2 launch" ' ...
               '&& kill -INT %d 2>/dev/null'], pid, pid);
[st, ~] = system(cmd);
if st == 0
    fprintf('Stopped OpenVINS (pid %d).\n', pid);
end
end

% Subscriber callback: stash the latest odom, guarding a possibly-deleted fig.
function onVioMessage(fig, m)
if ishandle(fig)
    setappdata(fig, 'vio_latest', m);
end
end

% Reuse (or create) a tagged figure so repeated VIO sessions don't pile up
% windows.
function f = namedFigure(tag, name)
f = findobj(0, 'Type', 'figure', 'Tag', tag);
if isempty(f)
    f = figure('Name', name, 'Tag', tag);
else
    f = f(1); clf(f); set(f, 'Name', name); figure(f);
end
end


% =========================================================================
% Plot the live VIO session: ground truth vs EKF vs VIO. The VIO trajectory
% (OpenVINS `global` frame, arbitrary yaw+origin) is rigidly SE3-aligned to
% ground truth over the logged window (Umeyama / standard ATE), which anchors
% it at the start ground-truth point. Velocity is compared as speed |v| (VIO
% twist is body-frame). L is the vio_log struct, n the row count.
% =========================================================================
function plotVioComparison(L, n)
if n < 10
    fprintf('VIO comparison: only %d sample(s) logged; nothing to plot.\n', max(n, 0));
    return;
end
t   = L.t(1:n);          vt   = L.vt(1:n);     % GT/EKF time, VIO message time
gt  = L.gt_pos(1:n, :);  gtv  = L.gt_vel(1:n, :);
ekf = L.ekf_pos(1:n, :); ekfv = L.ekf_vel(1:n, :);
vio = L.vio_pos(1:n, :); viov = L.vio_vel(1:n, :);
ok  = ~isnan(t) & ~isnan(vt) & all(~isnan(vio), 2) & ...
      all(~isnan(gt), 2) & all(~isnan(ekf), 2);
t = t(ok); vt = vt(ok); gt = gt(ok, :); gtv = gtv(ok, :);
ekf = ekf(ok, :); ekfv = ekfv(ok, :); vio = vio(ok, :); viov = viov(ok, :);
if size(vio, 1) < 10
    fprintf(['VIO comparison: <10 valid VIO samples (OpenVINS not ' ...
             'publishing /ov_msckf/odomimu?). Skipping plot.\n']);
    return;
end

% GT/EKF were sampled at frame time t; the VIO sample carries its own (lagged)
% sim-time vt. Interpolate GT/EKF onto vt so each VIO sample is compared to GT
% at the SAME instant, then plot against vt.
gt   = interp1(t, gt,   vt, 'linear', 'extrap');
ekf  = interp1(t, ekf,  vt, 'linear', 'extrap');
gtv  = interp1(t, gtv,  vt, 'linear', 'extrap');
ekfv = interp1(t, ekfv, vt, 'linear', 'extrap');

% SE3-align VIO -> ground truth (no scale; metric VIO).
[R, tt, ate] = umeyama_align(vio.', gt.');
vio_a = (R * vio.' + tt).';
err_ekf = vecnorm(ekf - gt, 2, 2);
err_vio = vecnorm(vio_a - gt, 2, 2);
% speed |v| is rotation-invariant, so VIO body-frame velocity norm == NED speed.
sp_gt = vecnorm(gtv, 2, 2); sp_ekf = vecnorm(ekfv, 2, 2); sp_vio = vecnorm(viov, 2, 2);
rmse_ekf = sqrt(mean(err_ekf.^2));

fprintf('\n=== VIO vs EKF vs ground truth (%d samples, %.1f s) ===\n', ...
        size(vio, 1), vt(end) - vt(1));
fprintf('EKF position RMSE: %.3f m  (mean %.3f, max %.3f)\n', ...
        rmse_ekf, mean(err_ekf), max(err_ekf));
fprintf('VIO position ATE : %.3f m  (mean %.3f, max %.3f) after SE3 align\n', ...
        ate, mean(err_vio), max(err_vio));

% Downsample for PLOTTING only (the metrics above use every sample). Rendering
% 8 line plots of every sample across 3 figures is what freezes the GUI on long
% flights; ~3000 points per line is visually identical and renders instantly.
np = numel(vt); ds = max(1, ceil(np / 3000)); di = 1:ds:np;
vt = vt(di); gt = gt(di, :); ekf = ekf(di, :); vio_a = vio_a(di, :);
err_ekf = err_ekf(di); err_vio = err_vio(di);
sp_gt = sp_gt(di); sp_ekf = sp_ekf(di); sp_vio = sp_vio(di);

lbl = {'North', 'East', 'Down'};
namedFigure('vio_cmp_pos', 'VIO/EKF/GT: position (NED)');
for i = 1:3
    subplot(3, 1, i); hold on; grid on;
    plot(vt, gt(:, i),    'k',   'LineWidth', 1.3);
    plot(vt, ekf(:, i),   'b',   'LineWidth', 1.0);
    plot(vt, vio_a(:, i), 'r--', 'LineWidth', 1.2);
    ylabel([lbl{i} ' [m]']);
    if i == 1, legend('ground truth', 'EKF', 'VIO (aligned)', 'Location', 'best'); end
end
xlabel('sim time [s]');

namedFigure('vio_cmp_err', 'VIO/EKF/GT: error + speed');
subplot(2, 1, 1); hold on; grid on;
plot(vt, err_ekf, 'b', 'LineWidth', 1.2);
plot(vt, err_vio, 'r', 'LineWidth', 1.2);
ylabel('position error vs GT [m]');
legend('EKF', 'VIO', 'Location', 'best');
title(sprintf('EKF RMSE %.3f m   |   VIO ATE %.3f m', rmse_ekf, ate));
subplot(2, 1, 2); hold on; grid on;
plot(vt, sp_gt,  'k',   'LineWidth', 1.3);
plot(vt, sp_ekf, 'b',   'LineWidth', 1.0);
plot(vt, sp_vio, 'r--', 'LineWidth', 1.2);
ylabel('speed [m/s]'); xlabel('sim time [s]');
legend('ground truth', 'EKF', 'VIO', 'Location', 'best');

namedFigure('vio_cmp_traj', 'VIO/EKF/GT: trajectory (top-down)');
hold on; grid on; axis equal;
plot(gt(:, 2),    gt(:, 1),    'k',   'LineWidth', 1.3);
plot(ekf(:, 2),   ekf(:, 1),   'b',   'LineWidth', 1.0);
plot(vio_a(:, 2), vio_a(:, 1), 'r--', 'LineWidth', 1.2);
xlabel('East [m]'); ylabel('North [m]'); title('Trajectory (top-down)');
legend('ground truth', 'EKF', 'VIO', 'Location', 'best');
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
function mm = buildMissionTab(parent, fig, modes)
lat0 = 40.32214266903304;   % Cesium origin (Baku), verified from Quba.unity
lon0 = 49.59745;
HALF = 350;                 % half-extent -> 700 m x 700 m map

% --- map axes (left), north-up local metres -------------------------------
axm = axes('Parent', parent, 'Units', 'normalized', ...
           'Position', [0.05 0.13 0.58 0.83]);
hold(axm, 'on'); box(axm, 'on'); axis(axm, 'equal');
set(axm, 'YDir', 'normal');                       % North up
xlim(axm, [-HALF HALF]); ylim(axm, [-HALF HALF]);
xlabel(axm, 'East (m)'); ylabel(axm, 'North (m)');
% persistent state for re-anchoring / labels / save (mm is a value struct).
setappdata(axm, 'lat0', lat0);  setappdata(axm, 'lon0', lon0);
setappdata(axm, 'HALF', HALF);  setappdata(axm, 'wp_labels', gobjects(0));
title(axm, sprintf('Mission map @ %.5f, %.5f — click to add a waypoint', ...
                   lat0, lon0));

addSatelliteBasemap(axm, lat0, lon0, HALF);       % best-effort imagery

grid(axm, 'on');
set(axm, 'Layer', 'top', 'GridColor', [1 1 1], 'GridAlpha', 0.3);
plot(axm, 0, 0, '+', 'Color', 'k', 'MarkerSize', 12, 'LineWidth', 1.5, ...
     'HitTest', 'off', 'PickableParts', 'none');   % origin / home

% waypoint graphics (data filled by updateMissionMap); HitTest off so clicks
% fall through to the axes ButtonDownFcn.
hPath   = plot(axm, NaN, NaN, '-', 'Color', 'w', 'LineWidth', 1.2, ...
               'HitTest', 'off', 'PickableParts', 'none');
hMid    = plot(axm, NaN, NaN, 'o', 'MarkerSize', 9,  'LineWidth', 1.0, ...
               'MarkerFaceColor', [1 0.85 0], 'MarkerEdgeColor', 'k', ...
               'HitTest', 'off', 'PickableParts', 'none');
hLaunch = plot(axm, NaN, NaN, 'o', 'MarkerSize', 11, 'LineWidth', 1.0, ...
               'MarkerFaceColor', [0 0.8 0], 'MarkerEdgeColor', 'k', ...
               'HitTest', 'off', 'PickableParts', 'none');
hEnd    = plot(axm, NaN, NaN, 'o', 'MarkerSize', 11, 'LineWidth', 1.0, ...
               'MarkerFaceColor', [0.9 0 0], 'MarkerEdgeColor', 'k', ...
               'HitTest', 'off', 'PickableParts', 'none');

% Live 2D flown-path trails (filled by the sim loop while VIO logging is on):
% ground truth (white), EKF (cyan), VIO aligned to GT (red). animatedline is
% incremental, so per-frame appends stay cheap.
% MaximumNumPoints caps the trail so the Mission map does not slow down over a
% long flight (the full path still goes to the comparison plot via vio_log).
gtTrail  = animatedline(axm, 'Color', [1 1 1],     'LineWidth', 3.0, ...
                        'MaximumNumPoints', 6000, 'HitTest', 'off', 'PickableParts', 'none');
ekfTrail = animatedline(axm, 'Color', [0 1 1],     'LineWidth', 3.0, ...
                        'MaximumNumPoints', 6000, 'HitTest', 'off', 'PickableParts', 'none');
vioTrail = animatedline(axm, 'Color', [1 0.2 0.2], 'LineStyle', '--', ...
                        'MaximumNumPoints', 6000, 'LineWidth', 3.0, ...
                        'HitTest', 'off', 'PickableParts', 'none');
% Clickable legend: click an entry to hide/show that trail (toggles its
% Visible). AutoUpdate off so it does not pick up the basemap/markers.
lgd = legend(axm, [gtTrail, ekfTrail, vioTrail], ...
             {'Ground truth', 'EKF', 'VIO'}, ...
             'Location', 'northeast', 'AutoUpdate', 'off', ...
             'TextColor', 'k', 'Color', 'w', 'FontSize', 9, 'Box', 'on');
lgd.ItemHitFcn = @legendToggleTrail;

% --- controls (right column) ----------------------------------------------
uicontrol(parent, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.66 0.915 0.33 0.035], 'BackgroundColor', 'w', ...
    'HorizontalAlignment', 'left', 'FontWeight', 'bold', ...
    'String', 'Altitude for next waypoint (m, +up):');
altEdit = uicontrol(parent, 'Style', 'edit', 'Units', 'normalized', ...
    'Position', [0.66 0.88 0.12 0.035], 'String', '10', ...
    'BackgroundColor', [0.99 0.99 0.97]);

uicontrol(parent, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.66 0.83 0.16 0.042], 'String', 'Load .plan', ...
    'FontWeight', 'bold', 'Callback', @(~,~) onLoadPlan(fig));
uicontrol(parent, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.83 0.83 0.16 0.042], 'String', 'Save .plan', ...
    'FontWeight', 'bold', 'Callback', @(~,~) setappdata(fig, 'save_plan_request', true));

uicontrol(parent, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.66 0.782 0.16 0.042], 'String', 'Set Home', ...
    'Callback', @(~,~) setappdata(fig, 'set_home_request', true));
uicontrol(parent, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.66 0.734 0.16 0.042], 'String', 'Remove last', ...
    'Callback', @(~,~) setappdata(fig, 'pop_request', true));
uicontrol(parent, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.83 0.734 0.16 0.042], 'String', 'Clear all', ...
    'Callback', @(~,~) setappdata(fig, 'clear_request', true));

uicontrol(parent, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.66 0.692 0.33 0.035], 'BackgroundColor', 'w', ...
    'HorizontalAlignment', 'left', 'FontSize', 8, ...
    'String', 'Green=launch  yellow=mid  red=end.  Edit Alt to change height.');

tbl = uitable('Parent', parent, 'Units', 'normalized', ...
    'Position', [0.66 0.40 0.33 0.285], ...
    'ColumnName', {'N (m)', 'E (m)', 'Alt (m)'}, ...
    'ColumnEditable', [false false true], ...
    'ColumnWidth', {70 70 70}, 'RowName', 'numbered', ...
    'CellEditCallback', @(~, ev) onAltEdit(fig, ev));

state_lbl = uicontrol(parent, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.66 0.13 0.33 0.255], 'BackgroundColor', 'w', ...
    'HorizontalAlignment', 'left', 'FontName', 'Courier New', ...
    'FontSize', 9, 'String', '');

% --- big flight-mode buttons (bottom strip) -------------------------------
mode_bg = uibuttongroup('Parent', parent, 'Units', 'normalized', ...
    'Position', [0.01 0.01 0.98 0.095], 'Title', 'Flight mode', ...
    'BackgroundColor', 'w', 'FontWeight', 'bold');
nM = numel(modes);
bw = 0.97 / nM;
for i = 1:nM
    s = modes{i};
    if strcmp(s, 'rtl'), disp_s = 'RTL'; else, disp_s = [upper(s(1)) s(2:end)]; end
    uicontrol(mode_bg, 'Style', 'togglebutton', 'Units', 'normalized', ...
        'Position', [0.015 + (i-1)*bw, 0.08, bw*0.95, 0.84], ...
        'String', disp_s, 'Tag', s, 'FontWeight', 'bold', 'FontSize', 10);
end
posBtn = findobj(mode_bg, 'Tag', 'position');   % default = Position
if ~isempty(posBtn), set(mode_bg, 'SelectedObject', posBtn(1)); end

% click handler: axes children have HitTest off, so the axes gets the click.
set(axm, 'ButtonDownFcn', @(src, ~) onMapClick(src, fig, altEdit, HALF));

mm = struct('ax', axm, 'path', hPath, 'launch', hLaunch, 'mid', hMid, ...
            'endp', hEnd, 'table', tbl, 'altEdit', altEdit, ...
            'mode_bg', mode_bg, 'state_lbl', state_lbl, ...
            'gt_trail', gtTrail, 'ekf_trail', ekfTrail, 'vio_trail', vioTrail);
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
    set(axm, 'Color', [0.15 0.18 0.20]);
    setappdata(axm, 'basemap_img', gobjects(0));
    warning('Mission map: satellite basemap unavailable (%s). Grid only.', ...
            ME.message);
end
end

% Re-anchor the map on a new (lat0, lon0): refresh the basemap + title.
function setMissionAnchor(mm, lat0, lon0)
if ~isstruct(mm) || ~isfield(mm, 'ax') || ~isvalid(mm.ax), return; end
HALF = getappdata(mm.ax, 'HALF');
setappdata(mm.ax, 'lat0', lat0);
setappdata(mm.ax, 'lon0', lon0);
old = getappdata(mm.ax, 'basemap_img');
if ~isempty(old) && all(isgraphics(old)), delete(old); end
set(mm.ax, 'Color', [1 1 1]);
addSatelliteBasemap(mm.ax, lat0, lon0, HALF);
title(mm.ax, sprintf('Mission map @ %.5f, %.5f — click to add a waypoint', ...
                     lat0, lon0));
end

% Select the mode toggle-button whose Tag matches modeStr.
function setModeButton(mm, modeStr)
if ~isstruct(mm) || ~isfield(mm, 'mode_bg') || ~isvalid(mm.mode_bg), return; end
btn = findobj(mm.mode_bg, 'Tag', modeStr);
if ~isempty(btn), set(mm.mode_bg, 'SelectedObject', btn(1)); end
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
pan = uipanel(parent, 'Units', 'normalized', 'Position', [0.03 0.55 0.94 0.43], ...
    'Title', 'System-identification autotune', 'BackgroundColor', 'w', 'FontWeight', 'bold');

uicontrol(pan, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.03 0.80 0.45 0.12], 'BackgroundColor', 'w', ...
    'HorizontalAlignment', 'left', 'String', 'Inject amplitude MC_AT_SYSID_AMP (rad/s):');
amp_edit = uicontrol(pan, 'Style', 'edit', 'Units', 'normalized', ...
    'Position', [0.50 0.80 0.15 0.13], 'BackgroundColor', [0.99 0.99 0.97], ...
    'String', num2str(at_opts.sysid_amp));
uicontrol(pan, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.03 0.62 0.45 0.12], 'BackgroundColor', 'w', ...
    'HorizontalAlignment', 'left', 'String', 'Desired rise time MC_AT_RISE_TIME (s):');
rise_edit = uicontrol(pan, 'Style', 'edit', 'Units', 'normalized', ...
    'Position', [0.50 0.62 0.15 0.13], 'BackgroundColor', [0.99 0.99 0.97], ...
    'String', num2str(at_opts.rise_time));

at_btn = uicontrol(pan, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.70 0.62 0.27 0.30], 'String', 'Start Autotune', ...
    'FontWeight', 'bold', 'BackgroundColor', [0.88 0.93 0.82], ...
    'Callback', @(~, ~) setappdata(fig, 'autotune_request', true));

uicontrol(pan, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.03 0.28 0.94 0.26], 'BackgroundColor', 'w', ...
    'HorizontalAlignment', 'left', 'FontSize', 8, ...
    'String', ['Preconditions (checked at Start): airborne (>1.5 m), low speed, ' ...
               'position/hold mode, sticks centred. A roll/pitch stick deflection ' ...
               'aborts. On success the identified gains are applied to the live ' ...
               'controllers and mirrored onto the Controller tab.']);

status_lbl = uicontrol(pan, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.03 0.04 0.94 0.20], 'BackgroundColor', [0.97 0.97 0.93], ...
    'HorizontalAlignment', 'left', 'FontName', 'Courier New', 'FontSize', 9, ...
    'String', 'autotune: idle');

results_lbl = uicontrol(parent, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.03 0.03 0.94 0.49], 'BackgroundColor', 'w', ...
    'HorizontalAlignment', 'left', 'FontName', 'Courier New', 'FontSize', 9, ...
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
panel = uipanel(parent, 'Units', 'normalized', 'Position', pos, ...
                'Title', title, 'BackgroundColor', 'w', ...
                'FontWeight', 'bold', 'FontSize', 9);
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
    'Position', [0.70 0.905 0.28 0.085], 'String', 'Reset defaults', ...
    'FontSize', 8, 'BackgroundColor', [0.95 0.90 0.85], ...
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
ncols = 3;                              % grid columns (3-axis params)
uicontrol(panel, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.02 ybot + 0.08*rowh 0.30 0.80*rowh], ...
    'BackgroundColor', 'w', 'HorizontalAlignment', 'left', ...
    'FontName', 'Courier New', 'FontSize', 8, ...
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
        'Min', lo(c), 'Max', hi(c), ...
        'Value', min(max(vals(c), lo(c)), hi(c)), 'TooltipString', spec.tip);
    edits(c) = uicontrol(panel, 'Style', 'edit', 'Units', 'normalized', ...
        'Position', [x0, ybot + 0.06*rowh, ew*0.92, 0.40*rowh], ...
        'BackgroundColor', [0.99 0.99 0.97], 'FontSize', 8, ...
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
