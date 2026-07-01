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
addpath(fullfile(root, 'src', 'io'));

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

fig = figure('Name', 'SYNAPLINE GCS — Flight Deck (MATLAB SIL)', ...
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

mode_strings = {'stabilized', 'altitude', 'position', 'hold', ...
                'mission', 'intercept', 'rtl', 'land', 'takeoff'};
default_mode_idx = 3;             % start in Position

% Global bottom bar: flight-mode buttons + Arm / Set Home / Reset / Stop,
% usable from any tab. The sim loop polls mode_bg's SelectedObject as before.
[mode_bg, arm_btn] = buildModeBar(fig, mode_strings);

% Manual-control sticks are embedded in the FLIGHT tab (built in
% buildMissionTab) -- they used to live in a separate floating window.
% RESET/STOP are on the bottom mode bar. left_h/right_h are pulled from
% mission_map below and read every frame; the physical pad (if any) and
% these on-screen sticks both drive them.

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

% Flight tab: north-up satellite map at the Cesium origin (Baku); click to
% drop waypoints, set per-waypoint altitude. Writes the same `waypoints`
% array (via map_add_request / alt_edit_request) the cockpit editor uses.
% Also hosts the attitude/heading instruments (pfd), updated per frame.
mission_map = buildMissionTab(tab_mission, fig, fmm);
mission_map.mode_bg = mode_bg;       % setModeButton() selects + restyles via this
left_h    = mission_map.left_h;      % on-screen manual sticks (Flight tab)
right_h   = mission_map.right_h;
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

% RC stick-override (PX4 COM_RC_OVERRIDE): while armed and airborne in an
% auto mode, moving the sticks hands control back to the pilot by switching
% to Position mode. Applies to every auto mode below, including Intercept.
override_det   = StickOverrideDetector(p.com.rc_stick_ov);
auto_modes     = {'mission', 'hold', 'rtl', 'land', 'takeoff', 'intercept'};

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

% Optional physical controller (e.g. PS4 pad). If a joystick is connected it
% drives the sticks and mirrors onto the on-screen caps; otherwise the mouse
% sticks are used. Confirm/adjust the axis map first with sim/test_joystick.m.
joystick = JoystickReader(1);   % never throws; hot-plug aware (auto-acquires)
if joystick.isConnected()
    fprintf(['run_interactive: physical joystick connected (%d axes) -- ' ...
             'using it for manual control.\n'], joystick.nAxes);
else
    fprintf(['run_interactive: no physical joystick yet (using on-screen ' ...
             'mouse sticks). Plug one in any time -- it is auto-detected ' ...
             'within ~1 s, no restart needed.\n']);
end
joy_was_connected = joystick.isConnected();   % track for hot-plug messages

k     = 0;
t_sim = 0;
ui_tick = 0;           % strip/readout refresh decimation (every 2nd frame)
cesium_bridge  = [];   % lazily created on the first sim frame (streaming is always on)
cesium_failed  = false; % latch: a bridge failure stops retries without per-frame spam

% Keep all ROS 2 traffic on loopback: every participant in this pipeline
% (MATLAB DDS nodes, rosbridge, Unity via rosbridge WebSocket) runs on this
% machine. rmw reads ROS_LOCALHOST_ONLY at participant creation, so set it
% BEFORE any ros2node / bridge is constructed. The launched subprocess gets
% its own export in startRosbridge (bash -lc re-reads the profile, so
% inheritance alone is not guaranteed).
setenv('ROS_LOCALHOST_ONLY', '1');

% Auto-launch rosbridge_server for the Unity/Cesium path. onCleanup guarantees
% it is stopped on any exit (Stop, window close, or an error in the loop).
rosbridge_pid = startRosbridge();
cleanup_rosbridge = onCleanup(@() stopRosbridge(rosbridge_pid));

% Intercept-mode guidance feed: the standalone C++ PN node publishes a NED
% acceleration setpoint on /guidance/acceleration_setpoint. Subscribe here (AFTER the
% ROS_LOCALHOST_ONLY=1 setenv above, so the DDS participant binds to the =1
% guidance node and rosbridge). The callback stashes the newest sample in
% appdata; the sim loop polls it while Intercept mode is active. Optional — the
% GUI still runs if ROS or the node isn't up (Intercept then just hovers).
setappdata(fig, 'guid_last', []);
intercept_prev_stamp = -1;
intercept_stale      = 0;
% Jamming-scenario state: GNSS-aiding config saved on Intercept lock and
% restored when Intercept ends (so the sim can be re-flown).
gnss_saved_ctrl   = est_bus.params.gps_ctrl;
gnss_saved_active = est_bus.ekf.gnss_active;
gnss_saved_vio    = est_bus.vio_enabled;
guid_sub = []; %#ok<NASGU>  kept in scope so the subscription stays alive
try
    guid_node = ros2node('matlab_intercept_listener'); %#ok<NASGU>
    guid_sub  = ros2subscriber(guid_node, '/guidance/acceleration_setpoint', ...
        'geometry_msgs/AccelStamped', ...
        @(m) setappdata(fig, 'guid_last', ...
            struct('a', [m.accel.linear.x; m.accel.linear.y; m.accel.linear.z], ...
                   'stamp', double(m.header.stamp.sec) + ...
                            double(m.header.stamp.nanosec) * 1e-9))); %#ok<NASGU>
    fprintf('Intercept guidance subscriber up on /guidance/acceleration_setpoint\n');
catch ME
    warning('Intercept guidance feed unavailable (%s). Intercept mode will coast (zero accel).', ...
            ME.message);
end

% Target-lock feed. A box list (/detection/boxes = [N, (cx,cy,w,h)*N] in slot
% order) drives the L1/R1/L2/R2 target lock: pressing a pad button publishes the
% chosen slot's box to /tracker/roi to seed hybrid_tracker_vpi.
% Optional: the GUI runs fine if the box source / ROS isn't up.
setappdata(fig, 'det_boxes', []);     % latest [N cx cy w h ...] vector
setappdata(fig, 'prev_lock_btn', []); % rising-edge state for the lock buttons
setappdata(fig, 'locked_slot', 0);    % last slot sent to /tracker/roi (0 = none)
det_box_sub = []; roi_pub = []; %#ok<NASGU>
try
    det_node = ros2node('matlab_detection_listener'); %#ok<NASGU>
    det_box_sub = ros2subscriber(det_node, '/detection/boxes', 'std_msgs/Float32MultiArray', ...
        @(m) setappdata(fig, 'det_boxes', double(m.data(:)))); %#ok<NASGU>
    roi_pub = ros2publisher(det_node, '/tracker/roi', 'sensor_msgs/RegionOfInterest');
    fprintf('Target-lock feed up (/detection/boxes); ROI publisher on /tracker/roi\n');
catch ME
    warning('Target-lock feed unavailable (%s). Target lock disabled.', ME.message);
end

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
        % Apply IMU noise scale before reset so initBias() sees it.
        imu_ns = 1.0;
        est_bus.sensors.setImuNoiseScale(imu_ns);
        est_bus.reset();
        % Re-inject the known bias after reset so post-Reset runs
        % have the same truth bias to estimate.
        est_bus.sensors.applyImuBias(true_gyro_bias * imu_ns, true_accel_bias * imu_ns);
        t_sim   = 0;
        % Clear the GT/EKF map trails + heading arrows on Reset.
        clearpoints(mission_map.gt_trail);  clearpoints(mission_map.ekf_trail);
        for ha = [mission_map.gt_arrow, mission_map.ekf_arrow]
            set(ha, 'XData', NaN, 'YData', NaN);
            ud = get(ha, 'UserData'); ud.hdg = NaN; set(ha, 'UserData', ud);
        end
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
        override_det.reset();   % start stick-movement detection fresh

        % --- GPS + radio jamming bubble on target lock (scenario) ----------
        % Entering Intercept models the UAV locking onto the target inside the
        % ~500 m jamming bubble: GNSS pos/vel aiding is denied and it flies
        % autonomously on the onboard EKF. The EKF is deliberately NOT reset —
        % it coasts from the healthy GPS-aided estimate held during manual
        % flight, so attitude stays observable (gravity + mag) while pos/vel
        % dead-reckon and drift. Leaving Intercept restores GNSS aiding so the
        % sim can be re-flown.
        if strcmp(new_mode, 'intercept')
            gnss_saved_ctrl   = est_bus.params.gps_ctrl;
            gnss_saved_active = est_bus.ekf.gnss_active;
            gnss_saved_vio    = est_bus.vio_enabled;
            est_bus.params.gps_ctrl = 0;      % EKF2_GNSS_CTRL=0: no GNSS pos/vel fusion
            est_bus.ekf.gnss_active = false;  % baro = sole height ref (denied behaviour)
            est_bus.vio_enabled     = false;  % no VIO pos/vel aid either
            fprintf(['INTERCEPT lock: entered GPS+radio jamming bubble. GNSS ' ...
                     'pos/vel aid DISABLED (gps_ctrl %d->0); flying autonomously ' ...
                     'on the onboard EKF (attitude-only).\n'], gnss_saved_ctrl);
        elseif strcmp(prev_mode, 'intercept')
            est_bus.params.gps_ctrl = gnss_saved_ctrl;
            est_bus.ekf.gnss_active = gnss_saved_active;
            est_bus.vio_enabled     = gnss_saved_vio;
            fprintf('INTERCEPT off: GNSS pos/vel aid RESTORED (gps_ctrl=%d).\n', ...
                    gnss_saved_ctrl);
        end

        prev_mode = new_mode;
    end

    % --- Read sticks (each frame) -----------------------------------------
    % Both inputs work: the physical pad drives by default and mirrors onto
    % the on-screen caps; grabbing a cap with the mouse (drag_active) takes
    % over while held and the pad resumes on release. The pad is read once
    % here for both sticks and buttons (buttons drive the target lock below).
    pad_sticks = []; joy_buttons = [];
    if ~isempty(joystick)
        [pad_sticks, joy_buttons] = joystick.read();
        % Announce hot-plug connect/disconnect transitions once.
        if joystick.isConnected() && ~joy_was_connected
            fprintf(['run_interactive: joystick connected (%d axes) -- pad ' ...
                     'now driving manual control.\n'], joystick.nAxes);
        elseif ~joystick.isConnected() && joy_was_connected
            fprintf(['run_interactive: joystick disconnected -- back to ' ...
                     'on-screen mouse sticks.\n']);
        end
        joy_was_connected = joystick.isConnected();
    end
    drag_active = ~isempty(getappdata(fig, 'drag_target'));
    if ~isempty(pad_sticks) && ~drag_active
        sticks = pad_sticks;
        if ishandle(left_h) && ishandle(right_h)
            set(left_h,  'XData', sticks.left_x,  'YData', sticks.left_y);
            set(right_h, 'XData', sticks.right_x, 'YData', sticks.right_y);
        end
    elseif ishandle(left_h) && ishandle(right_h)
        sticks.left_x  = get(left_h,  'XData');
        sticks.left_y  = get(left_h,  'YData');
        sticks.right_x = get(right_h, 'XData');
        sticks.right_y = get(right_h, 'YData');
    end

    % --- Target lock: L1/R1/L2/R2 -> publish slot box to /tracker/roi ------
    if ~isempty(joy_buttons) && ~isempty(roi_pub)
        handleTargetLock(fig, joy_buttons, roi_pub);
    end

    % --- RC stick override (PX4 COM_RC_OVERRIDE) ------------------------
    % Armed + airborne + in an auto mode + sticks moving => pilot takes over,
    % switch to Position mode (Commander.cpp:2895-2939). Selecting the button
    % routes through the normal mode-change block, so Intercept's GNSS restore
    % and the lead-filter resets happen as on a manual mode change. The
    % detector only runs in auto modes; reset() on each mode entry avoids a
    % spurious trigger from the entry transient.
    if bitand(p.com.rc_override, 1) && armed && ~landed_latch && ...
       any(strcmp(fmm.mode, auto_modes))
        if override_det.update(sticks, dt_frame)
            setModeButton(mission_map, 'position');
            fprintf('Pilot took over using sticks (%s -> position).\n', fmm.mode);
        end
    end

    % Wind parameters are set directly by the Wind tab callbacks.

    % --- Advance physics by dt_frame in dt_rate substeps ---
    n_steps = max(1, round(dt_frame / dt_rate));
    use_est = logical(get(est_cb, 'Value'));
    % Jamming scenario: while Intercept is active the controller MUST fly on the
    % onboard estimator (real hardware has no ground-truth feed), regardless of
    % the SENSORS-tab toggle. The EKF is GNSS-denied (set on lock above).
    if strcmp(fmm.mode, 'intercept')
        use_est = true;
    end
    % Cesium/Unity streaming is always on; LINK pill is green once the
    % bridge is up (drives the status strip).
    stream_on = ~isempty(cesium_bridge);
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
            % Intercept mode: feed the newest external guidance setpoint into
            % the FMM. Freshness = the heartbeat stamp advancing; if it stalls
            % (node down) mark it invalid and the mode coasts (zero accel).
            if strcmp(fmm.mode, 'intercept')
                gl = getappdata(fig, 'guid_last');
                if isempty(gl)
                    fmm.setInterceptAccel([0; 0; 0], false);
                else
                    if gl.stamp ~= intercept_prev_stamp
                        intercept_prev_stamp = gl.stamp;
                        intercept_stale = 0;
                    else
                        intercept_stale = intercept_stale + 1;
                    end
                    fmm.setInterceptAccel(gl.a, intercept_stale < round(0.3 / dt_pos));
                end
            end
            cmd = fmm.update(s, sticks, dt_pos);
            % Mode may have auto-transitioned (Takeoff -> Hold). Reflect in
            % the mode buttons and reset lead state across the discontinuity.
            if ~strcmp(cmd.mode, prev_mode)
                setModeButton(mission_map, cmd.mode);
                prev_mode = cmd.mode;
                pos_lead.reset();
                vel_lead.reset();
                att_lead.reset();
                override_det.reset();   % fresh detection after auto-transition
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
            elseif strcmp(cmd.kind, 'accel')
                % Velocity/position-estimation-free Intercept: invert the NED
                % kinematic acceleration setpoint straight to attitude + thrust.
                % Closes on ATTITUDE only — s.position_ned / s.velocity_ned are
                % deliberately NOT read here.
                [q_sp, thrust_body_z] = pos_ctl.accelToAttitude(cmd.acc_sp, cmd.yaw_sp);
                yawspeed_sp = cmd.yawspeed_sp;
                vel_sp_used = nan(3, 1);
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

    % --- Stream pose to Cesium/Unity (always on) --------------------------
    % Ground-truth pose, converted NED/FRD->ENU/FLU and published as
    % geometry_msgs/PoseArray. Streaming is a permanent part of the GCS (no
    % toggle); a failure latches cesium_failed so we stop retrying without
    % per-frame warning spam, and the sim keeps running.
    if ~cesium_failed
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
                cesium_bridge = []; cesium_failed = true;
            end
        end
        if ~isempty(cesium_bridge)
            try
                cesium_bridge.publish(s.position_ned, s.attitude_q, t_sim);
            catch ME
                warning('Cesium bridge publish failed (%s). Disabling.', ME.message);
                delete(cesium_bridge); cesium_bridge = []; cesium_failed = true;
            end
        end
    end

    % --- Always-on GT and EKF map trails ----------------------------------
    % Updated every outer-loop frame (~20-30 Hz).
    gtn_map  = plant.state();
    estn_map = est_bus.stateOut();
    addpoints(mission_map.gt_trail,  gtn_map.position_ned(2),  gtn_map.position_ned(1));
    addpoints(mission_map.ekf_trail, estn_map.position_ned(2), estn_map.position_ned(1));
    % Arrows point along the vehicle's actual HEADING (yaw) — where the nose
    % points — NOT course over ground, so a crabbing/sliding UAV still reads
    % correctly. GT arrow uses ground-truth yaw; EKF arrow uses the
    % estimator's own yaw, so their divergence is visible.
    gt_rpy  = quat_to_euler(gtn_map.attitude_q);
    ekf_rpy = quat_to_euler(estn_map.attitude_q);
    updateHeadingArrow(mission_map.gt_arrow,  gtn_map.position_ned(2),  gtn_map.position_ned(1),  gt_rpy(3));
    updateHeadingArrow(mission_map.ekf_arrow, estn_map.position_ned(2), estn_map.position_ned(1), ekf_rpy(3));
    % Dynamic map: smoothly drag the viewport to keep the vehicle in view.
    mapFollow(mission_map.ax, gtn_map.position_ned(2), gtn_map.position_ned(1));

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
    hdg = mod(rad2deg(rpy_d(3)), 360);            % displayed/EKF heading
    gt_hdg = mod(rad2deg(gt_rpy(3)), 360);        % ground-truth heading
    pfd.update(rpy_d(1), rpy_d(2), hdg, gt_hdg, wind_ned);
    ui_tick = ui_tick + 1;
    if mod(ui_tick, 2) == 0
        vs  = -s_disp.velocity_ned(3);            % climb rate, +up
        gs  = norm(s_disp.velocity_ned(1:2));     % ground speed
        updateStatusStrip(strip, prev_mode, eU, gs, vs, t_sim, ...
                          armed, use_est, stream_on);
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

if ~isempty(joystick)
    joystick.close();            % release the physical controller
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

setappdata(fig, 'drag_target', '');
% Stash the handles so the combined figure handler can drive the drag
% without threading them through every callback.
setappdata(fig, 'joy', struct('axl', ax_l, 'axr', ax_r, ...
    'lh', left_h, 'rh', right_h, 'lt', left_t, 'rt', right_t));
end

function startDrag(fig, which_)
setappdata(fig, 'drag_target', which_);
end

% Combined figure mouse handlers: manual-stick drag AND map pan. Each acts
% only when its own drag is active, so they coexist on the one figure.
function figMotion(fig, axm)
joyMotion(fig);
if ishandle(axm), onMapPanMotion(fig, axm); end
end

function figButtonUp(fig, axm)
joyEndDrag(fig);
if ishandle(axm), onMapPanEnd(fig); end
end

function joyMotion(fig)
target = getappdata(fig, 'drag_target');
if isempty(target), return; end
j = getappdata(fig, 'joy'); if isempty(j), return; end
switch target
    case 'left'
        cp = get(j.axl, 'CurrentPoint');
        x = max(-1, min(1, cp(1, 1))); y = max(-1, min(1, cp(1, 2)));
        set(j.lh, 'XData', x, 'YData', y); set(j.lt, 'XData', [0 x], 'YData', [0 y]);
    case 'right'
        cp = get(j.axr, 'CurrentPoint');
        x = max(-1, min(1, cp(1, 1))); y = max(-1, min(1, cp(1, 2)));
        set(j.rh, 'XData', x, 'YData', y); set(j.rt, 'XData', [0 x], 'YData', [0 y]);
end
end

function joyEndDrag(fig)
target = getappdata(fig, 'drag_target');
if isempty(target), return; end
j = getappdata(fig, 'joy');
if ~isempty(j)
    switch target
        case 'left',  set(j.lh, 'XData', 0, 'YData', 0); set(j.lt, 'XData', [0 0], 'YData', [0 0]);
        case 'right', set(j.rh, 'XData', 0, 'YData', 0); set(j.rt, 'XData', [0 0], 'YData', [0 0]);
    end
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
    rowSpec('Position P  (N E D)',    'MPC_XY_P, MPC_XY_P, MPC_Z_P', ...
            @() pos_ctl.gain_pos_p, @(v) setProp(pos_ctl, 'gain_pos_p', v), ...
            0, 3, p.pos.gain_pos_p);
    rowSpec('Velocity P  (N E D)',    'MPC_XY_VEL_P_ACC / MPC_Z_VEL_P_ACC', ...
            @() pos_ctl.gain_vel_p, @(v) setProp(pos_ctl, 'gain_vel_p', v), ...
            0, 8, p.pos.gain_vel_p);
    rowSpec('Velocity I  (N E D)',    'MPC_XY_VEL_I_ACC / MPC_Z_VEL_I_ACC', ...
            @() pos_ctl.gain_vel_i, @(v) setProp(pos_ctl, 'gain_vel_i', v), ...
            0, 5, p.pos.gain_vel_i);
    rowSpec('Velocity D  (N E D)',    'MPC_XY_VEL_D_ACC / MPC_Z_VEL_D_ACC', ...
            @() pos_ctl.gain_vel_d, @(v) setProp(pos_ctl, 'gain_vel_d', v), ...
            0, 2, p.pos.gain_vel_d);
    rowSpec('Velocity max (xy up dn) m/s', 'MPC_XY_VEL_MAX, MPC_Z_VEL_MAX_UP, _DN', ...
            @() [pos_ctl.lim_vel_horizontal; pos_ctl.lim_vel_up; pos_ctl.lim_vel_down], ...
            @(v) setVelLims(pos_ctl, v), ...
            [0;0;0], [25;10;10], [p.pos.vel_xy_max; p.pos.vel_z_up; p.pos.vel_z_down]);
    rowSpec('Tilt max (deg)',         'MPC_TILTMAX_AIR', ...
            @() rad2deg(pos_ctl.lim_tilt), @(v) setProp(pos_ctl, 'lim_tilt', deg2rad(v)), ...
            0, 80, rad2deg(p.pos.tilt_max));
    rowSpec('Thrust  (min hov max)',  'MPC_THR_MIN, MPC_THR_HOVER, MPC_THR_MAX', ...
            @() [pos_ctl.thr_min; pos_ctl.hover_thrust; pos_ctl.thr_max], ...
            @(v) setThr(pos_ctl, v), ...
            [0;0;0], [0.5;1;1], [p.pos.thr_min; p.pos.thr_hover; p.pos.thr_max]);
};

% --- Attitude controller ---
att_rows = {
    rowSpec('Attitude P  (r p y)',    'MC_ROLL_P, MC_PITCH_P, MC_YAW_P', ...
            @() attPGet(att_ctl), @(v) att_ctl.setProportionalGain(v(:), att_ctl.yaw_w), ...
            0, 12, p.att.gain_p);
    rowSpec('Yaw weight',             'MC_YAW_WEIGHT (0..1)', ...
            @() att_ctl.yaw_w, @(v) attYawSet(att_ctl, v), ...
            0, 1, p.att.yaw_weight);
    rowSpec('Rate max  (r p y) deg/s', 'MC_ROLLRATE_MAX, MC_PITCHRATE_MAX, MC_YAWRATE_MAX', ...
            @() rad2deg(att_ctl.rate_limit), @(v) setProp(att_ctl, 'rate_limit', deg2rad(v)), ...
            0, 360, rad2deg(p.att.rate_max));
};

% --- Rate controller (inner loop). Defaults shown effective (x MC_*RATE_K). ---
rate_rows = {
    rowSpec('Rate P  (r p y)',        'MC_*RATE_P x MC_*RATE_K (effective)', ...
            @() rate_ctl.gain_p, @(v) setProp(rate_ctl, 'gain_p', v), ...
            0, 0.6, p.rate.gain_p .* p.rate.gain_k);
    rowSpec('Rate I  (r p y)',        'MC_*RATE_I x MC_*RATE_K (effective)', ...
            @() rate_ctl.gain_i, @(v) setProp(rate_ctl, 'gain_i', v), ...
            0, 0.8, p.rate.gain_i .* p.rate.gain_k);
    rowSpec('Rate D  (r p y)',        'MC_*RATE_D x MC_*RATE_K (effective)', ...
            @() rate_ctl.gain_d, @(v) setProp(rate_ctl, 'gain_d', v), ...
            0, 0.02, p.rate.gain_d .* p.rate.gain_k);
    rowSpec('Rate FF  (r p y)',       'MC_ROLLRATE_FF, MC_PITCHRATE_FF, MC_YAWRATE_FF', ...
            @() rate_ctl.gain_ff, @(v) setProp(rate_ctl, 'gain_ff', v), ...
            0, 0.5, p.rate.gain_ff);
    rowSpec('Rate int limit  (r p y)', 'MC_RR_INT_LIM, MC_PR_INT_LIM, MC_YR_INT_LIM', ...
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
    fprintf(['rosbridge_server already running; leaving it as-is.\n' ...
             '  NOTE: if it was started before the localhost-only change, it is\n' ...
             '  still bound to all interfaces — kill it and rerun to apply.\n']);
    return;   % pid stays [] -> stopRosbridge() will not touch it
end
% `exec` makes the backgrounded subshell BECOME ros2 launch, so $! is the
% ros2-launch PID (not a throwaway subshell) and SIGINT later reaches it.
% Loopback only: ROS_LOCALHOST_ONLY pins DDS to lo; address:=127.0.0.1 pins
% the WebSocket listener (default '' = all interfaces) so Unity must be local.
cmd = ['bash -lc ''unset LD_LIBRARY_PATH; export ROS_LOCALHOST_ONLY=1; ' ...
       'source /opt/ros/humble/setup.bash && ' ...
       'exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml ' ...
       'address:=127.0.0.1 ' ...
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
HALF = 1000;                % half-extent -> 2 km x 2 km map

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
setappdata(axm, 'follow', true);   % dynamic-map follow (off on manual pan/zoom)

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

% Live 2D flown-path trails (always active):
% ground truth (green), EKF (red). MaximumNumPoints caps each trail so the
% Mission map does not slow down over a long flight.
gtTrail  = animatedline(axm, 'Color', [0.20 0.90 0.00], 'LineWidth', 3.0, ...
                        'MaximumNumPoints', 6000, 'HitTest', 'off', 'PickableParts', 'none');
ekfTrail = animatedline(axm, 'Color', [0.95 0.10 0.10], 'LineWidth', 3.0, ...
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

% Pro flight-plan list (custom-drawn). Click a row to edit its altitude in
% the editor below; altitude flows back via the same alt_edit_request path.
planAx = axes('Parent', plan_pan, 'Units', 'normalized', ...
    'Position', [0.04 0.095 0.92 0.345], 'XLim', [0 1], 'YLim', [0 1], 'Color', T.field);
hold(planAx, 'on'); axis(planAx, 'off');
disableDefaultInteractivity(planAx);
setappdata(planAx, 'sel', 0);
altLbl = uicontrol(plan_pan, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.04 0.022 0.56 0.050], 'BackgroundColor', T.panel, ...
    'ForegroundColor', T.sub, 'HorizontalAlignment', 'left', ...
    'FontName', T.mono, 'FontSize', 8, 'String', 'CLICK A WAYPOINT TO EDIT ALT');
altCell = uicontrol(plan_pan, 'Style', 'edit', 'Units', 'normalized', ...
    'Position', [0.62 0.018 0.34 0.058], 'BackgroundColor', T.field, ...
    'ForegroundColor', T.acft, 'FontName', T.mono, 'FontWeight', 'bold', ...
    'FontSize', 9, 'String', '', 'Enable', 'off');

% Manual-control sticks, embedded here (was a floating console). Both the
% physical pad and these on-screen sticks drive the same left_h/right_h.
uicontrol(parent, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.655 0.205 0.335 0.018], 'BackgroundColor', T.panel, ...
    'ForegroundColor', T.sub, 'HorizontalAlignment', 'left', ...
    'FontName', T.mono, 'FontSize', 7.5, 'FontWeight', 'bold', ...
    'String', 'MANUAL CONTROL  ·  DRAG OR USE PAD');
ax_left  = axes('Parent', parent, 'Units', 'normalized', 'Position', [0.658 0.035 0.158 0.170]);
ax_right = axes('Parent', parent, 'Units', 'normalized', 'Position', [0.834 0.035 0.158 0.170]);
[left_h, right_h] = makeJoysticks(fig, ax_left, ax_right);

% click handler: left-click adds waypoint, right-click starts pan drag.
% Scroll-wheel zooms around the cursor; double-click resets the view.
set(axm, 'ButtonDownFcn', @(src, ~) onMapButton(src, fig, altEdit, HALF));
set(fig, 'WindowScrollWheelFcn',  @(~, ev) onMapScroll(axm, ev));
% Combined motion/up handler: dispatches to BOTH the manual-stick drag and
% the map pan (each acts only when its own drag is active), so embedding the
% sticks in this figure doesn't clobber the map's pan handlers.
set(fig, 'WindowButtonMotionFcn', @(~, ~) figMotion(fig, axm));
set(fig, 'WindowButtonUpFcn',     @(~, ~) figButtonUp(fig, axm));
setappdata(fig, 'map_pan_start', []);

% Heading arrows: filled triangle at the tip of each trail showing flight
% direction. Drawn on top of trails; UserData caches the last valid heading
% so the arrow holds direction when the vehicle is stationary.
ARROW_SZ = 28;   % arrow length in map metres
gtArrow  = patch(axm, NaN, NaN, [0.20 0.90 0.00], 'EdgeColor', 'none', ...
                 'HitTest', 'off', 'PickableParts', 'none', ...
                 'UserData', struct('hdg', NaN, 'sz', ARROW_SZ));
ekfArrow = patch(axm, NaN, NaN, [0.95 0.10 0.10], 'EdgeColor', 'none', ...
                 'HitTest', 'off', 'PickableParts', 'none', ...
                 'UserData', struct('hdg', NaN, 'sz', ARROW_SZ));

% Link each trail to its arrow via AppData so legendToggleTrail can sync visibility.
setappdata(gtTrail,  'arrow', gtArrow);
setappdata(ekfTrail, 'arrow', ekfArrow);

mm = struct('ax', axm, 'path', hPath, 'launch', hLaunch, 'mid', hMid, ...
            'endp', hEnd, 'planAx', planAx, 'altCell', altCell, 'altLbl', altLbl, ...
            'fig', fig, 'altEdit', altEdit, 'pfd', pfd, ...
            'left_h', left_h, 'right_h', right_h, ...
            'gt_trail', gtTrail, 'ekf_trail', ekfTrail, ...
            'gt_arrow', gtArrow, 'ekf_arrow', ekfArrow);

% Row select (edit altitude) — wired after mm is built so the closures see it.
set(planAx, 'ButtonDownFcn', @(~,~) onPlanClick(mm));
set(altCell, 'Callback',     @(~,~) onAltCellEdit(mm));
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


% Rising edge of DS4 L1/R1/L2/R2 (1-based buttons 5/6/7/8) -> publish that
% slot's detection box to /tracker/roi (sensor_msgs/RegionOfInterest). The
% box list is /detection/boxes = [N, (cx,cy,w,h)*N] in slot order.
function handleTargetLock(fig, buttons, roi_pub)
LOCK_BTN = [5 6 7 8];                       % L1 R1 L2 R2 -> slot 1..4
b    = double(buttons(:))';
prev = getappdata(fig, 'prev_lock_btn');
% Seed from the CURRENT state on the first call / when the button count
% changes (e.g. pad reconnect), so a button already held is not seen as a
% fresh press (avoids a spurious lock at launch / on reconnect).
if numel(prev) ~= numel(b), prev = b; end
boxes  = getappdata(fig, 'det_boxes');      % [N cx cy w h ...]
b_save = b;                                 % what we record as "previous" next frame
for slot = 1:numel(LOCK_BTN)
    bi = LOCK_BTN(slot);
    rising = bi <= numel(b) && b(bi) > 0.5 && prev(bi) < 0.5;
    if ~rising, continue; end
    % Service the press only if that slot has a usable box this frame.
    off = 2 + (slot-1)*4;                    % cx cy w h
    serviced = ~isempty(boxes) && boxes(1) >= slot && numel(boxes) >= off + 3;
    if serviced
        cx = boxes(off); cy = boxes(off+1); w = boxes(off+2); h = boxes(off+3);
        roi = ros2message('sensor_msgs/RegionOfInterest');
        roi.x_offset = uint32(max(0, round(cx - w/2)));
        roi.y_offset = uint32(max(0, round(cy - h/2)));
        roi.width    = uint32(max(1, round(w)));
        roi.height   = uint32(max(1, round(h)));
        send(roi_pub, roi);
        setappdata(fig, 'locked_slot', slot);
        fprintf('Target LOCK: slot %d (button %d) -> ROI x=%d y=%d w=%d h=%d\n', ...
                slot, bi, roi.x_offset, roi.y_offset, roi.width, roi.height);
    else
        % Rising edge with no box yet: don't latch it, so a still-held press
        % locks as soon as the target appears in that slot.
        b_save(bi) = 0;
    end
end
setappdata(fig, 'prev_lock_btn', b_save);
end


% =========================================================================
function P = buildPFD(parent)
T = gcsTheme();
R = 0.97; kP = R * 0.150;          % disk radius; vertical units per 10 deg pitch

% ===== Attitude indicator (round glass ADI) ============================
axA = axes('Parent', parent, 'Units', 'normalized', ...
           'Position', [0.05 0.345 0.90 0.625]);
hold(axA, 'on'); axis(axA, 'off');
set(axA, 'XLim', [-1.36 1.36], 'YLim', [-1.18 1.18], ...
         'DataAspectRatio', [1 1 1], 'Color', T.bg);
disableDefaultInteractivity(axA);

th = linspace(0, 2*pi, 220);
patch(axA, R*cos(th), R*sin(th), T.gnd, 'EdgeColor', 'none', 'HitTest', 'off'); % ground disk
A.sky     = patch(axA, NaN, NaN, T.sky, 'EdgeColor', 'none', 'HitTest', 'off'); % sky cap (per frame)
A.horizon = line(axA, NaN, NaN, 'Color', 'w', 'LineWidth', 2.2, 'HitTest', 'off');
A.ladder  = line(axA, NaN, NaN, 'Color', 'w', 'LineWidth', 1.4, 'HitTest', 'off');
A.lblL = gobjects(1, 6); A.lblR = gobjects(1, 6);
for i = 1:6
    A.lblL(i) = text(axA, NaN, NaN, '', 'Color', 'w', 'FontName', T.mono, 'FontSize', 8, ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'HitTest', 'off');
    A.lblR(i) = text(axA, NaN, NaN, '', 'Color', 'w', 'FontName', T.mono, 'FontSize', 8, ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'HitTest', 'off');
end
A.bank = line(axA, NaN, NaN, 'Color', 'w', 'LineWidth', 1.2, 'HitTest', 'off');  % bank scale (rolls)
yb = R*0.86;
patch(axA, [-0.05 0.05 0], [yb+0.07 yb+0.07 yb], T.acft, 'EdgeColor', 'none', 'HitTest', 'off'); % fixed bank pointer
% fixed aircraft symbol (yellow wings + centre dot)
line(axA, [-0.42 -0.12 NaN 0.12 0.42], [0 0 NaN 0 0], 'Color', T.acft, 'LineWidth', 4, 'HitTest', 'off');
line(axA, [-0.12 -0.12 NaN 0.12 0.12], [0 -0.07 NaN 0 -0.07], 'Color', T.acft, 'LineWidth', 4, 'HitTest', 'off');
line(axA, 0, 0, 'Marker', 'o', 'MarkerSize', 4.5, 'MarkerFaceColor', T.acft, 'MarkerEdgeColor', T.acft, 'HitTest', 'off');
line(axA, R*cos(th), R*sin(th), 'Color', T.edge, 'LineWidth', 2, 'HitTest', 'off');   % bezel ring
% digital ROLL / PITCH readouts
rectangle(axA, 'Position', [-1.34 0.92 0.62 0.22], 'Curvature', 0.25, 'FaceColor', T.field, 'EdgeColor', T.edge);
rectangle(axA, 'Position', [ 0.72 0.92 0.62 0.22], 'Curvature', 0.25, 'FaceColor', T.field, 'EdgeColor', T.edge);
text(axA, -1.30, 1.09, 'ROLL',  'Color', T.sub, 'FontName', T.mono, 'FontSize', 7, 'HitTest', 'off');
text(axA,  0.76, 1.09, 'PITCH', 'Color', T.sub, 'FontName', T.mono, 'FontSize', 7, 'HitTest', 'off');
A.rollTxt  = text(axA, -0.76, 0.99, '--', 'Color', T.text, 'FontName', T.mono, 'FontSize', 12, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'right', 'HitTest', 'off');
A.pitchTxt = text(axA,  1.30, 0.99, '--', 'Color', T.text, 'FontName', T.mono, 'FontSize', 12, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'right', 'HitTest', 'off');
A.R = R; A.kP = kP;

% ===== Heading indicator (heading-up HSI arc) ==========================
axH = axes('Parent', parent, 'Units', 'normalized', ...
           'Position', [0.05 0.045 0.90 0.265]);
hold(axH, 'on'); axis(axH, 'off');
set(axH, 'XLim', [-1.30 1.30], 'YLim', [0.10 1.05], 'Color', T.field);
disableDefaultInteractivity(axH);
A.Hc = -1.54; A.Rc = 2.40; A.Hf = 0.675;     % arc circle centre-y, radius, deg->screen factor
A.hsiTicks = line(axH, NaN, NaN, 'Color', 'w', 'LineWidth', 1.0, 'HitTest', 'off');
A.hsiLbl = gobjects(1, 11);
for i = 1:11
    A.hsiLbl(i) = text(axH, NaN, NaN, '', 'Color', 'w', 'FontName', T.mono, 'FontSize', 9, ...
        'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'HitTest', 'off');
end
A.gtTick = line(axH, NaN, NaN, 'Color', T.good, 'LineWidth', 3, 'HitTest', 'off');
A.gtLbl  = text(axH, NaN, NaN, 'GT', 'Color', T.good, 'FontName', T.mono, 'FontSize', 8, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'HitTest', 'off');
yl = A.Hc + A.Rc;                            % arc top
patch(axH, [-0.05 0.05 0], [yl+0.12 yl+0.12 yl+0.02], 'w', 'EdgeColor', 'none', 'HitTest', 'off'); % lubber
rectangle(axH, 'Position', [-0.17 0.80 0.34 0.20], 'Curvature', 0.25, 'FaceColor', T.field, 'EdgeColor', T.acft, 'LineWidth', 1.4);
A.hdgTxt = text(axH, 0, 0.90, '---', 'Color', T.text, 'FontName', T.mono, 'FontSize', 13, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'HitTest', 'off');
text(axH, 0.22, 0.90, 'MAG', 'Color', T.good, 'FontName', T.mono, 'FontSize', 8, 'FontWeight', 'bold', 'HitTest', 'off');
% wind cell (bottom-left): digital direction + speed
rectangle(axH, 'Position', [-1.26 0.16 0.74 0.30], 'Curvature', 0.18, 'FaceColor', T.field, 'EdgeColor', T.data, 'LineWidth', 1.0);
A.windDir = text(axH, -1.20, 0.36, '---\circ', 'Color', T.text, 'FontName', T.mono, 'FontSize', 11, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'left', 'Interpreter', 'tex', 'HitTest', 'off');
A.windSpd = text(axH, -1.20, 0.23, '-- m/s', 'Color', T.data, 'FontName', T.mono, 'FontSize', 9, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'left', 'HitTest', 'off');
text(axH, -0.66, 0.40, 'WIND', 'Color', T.sub, 'FontName', T.mono, 'FontSize', 7, 'FontWeight', 'bold', 'HitTest', 'off');
% GT / EKF digital (bottom-right)
A.gtHdgTxt  = text(axH, 1.26, 0.40, 'GT  ---', 'Color', T.good, 'FontName', T.mono, 'FontSize', 9, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'right', 'HitTest', 'off');
A.ekfHdgTxt = text(axH, 1.26, 0.24, 'EKF ---', 'Color', T.bad, 'FontName', T.mono, 'FontSize', 9, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'right', 'HitTest', 'off');

P = struct('update', @(roll, pitch, ekf_hdg, gt_hdg, wind_ned) ...
                     pfdUpdate(A, roll, pitch, ekf_hdg, gt_hdg, wind_ned));
end

% Per-frame PFD refresh. roll/pitch in rad; ekf_hdg/gt_hdg in deg [0,360);
% wind_ned = [N;E;D] m/s.
function pfdUpdate(A, roll, pitch, ekf_hdg, gt_hdg, wind_ned)
R = A.R; kP = A.kP;
u = [-sin(roll); cos(roll)]; t = [cos(roll); sin(roll)];
pu = -pitch / deg2rad(10) * kP;                  % pitch up -> horizon down

% --- ADI: sky cap + horizon ---
Psky = halfDiskPoly(R, u, pu);
if isempty(Psky), set(A.sky, 'XData', NaN, 'YData', NaN);
else,             set(A.sky, 'XData', Psky(1,:), 'YData', Psky(2,:)); end
if abs(pu) < R
    Lh = sqrt(R^2 - pu^2); c0 = pu*u; a1 = c0 + Lh*t; b1 = c0 - Lh*t;
    set(A.horizon, 'XData', [a1(1) b1(1)], 'YData', [a1(2) b1(2)]);
else
    set(A.horizon, 'XData', NaN, 'YData', NaN);
end
% pitch ladder + labels
dvals = [-30 -20 -10 10 20 30]; xs = []; ys = [];
for i = 1:6
    d = dvals(i); off = pu + d/10*kP; c = off*u; a1 = c + 0.15*t; b1 = c - 0.15*t;
    xs = [xs a1(1) b1(1) NaN]; ys = [ys a1(2) b1(2) NaN]; %#ok<AGROW>
    e = (a1-b1)/norm(a1-b1); Lp = a1 + e*0.07; Rp = b1 - e*0.07;
    set(A.lblL(i), 'Position', [Lp(1) Lp(2) 0], 'String', num2str(abs(d)), 'Rotation', -rad2deg(roll));
    set(A.lblR(i), 'Position', [Rp(1) Rp(2) 0], 'String', num2str(abs(d)), 'Rotation', -rad2deg(roll));
end
for d = [-35 -25 -15 -5 5 15 25 35]
    off = pu + d/10*kP; c = off*u; a1 = c + 0.06*t; b1 = c - 0.06*t;
    xs = [xs a1(1) b1(1) NaN]; ys = [ys a1(2) b1(2) NaN]; %#ok<AGROW>
end
set(A.ladder, 'XData', xs, 'YData', ys);
% bank scale (rolls with roll)
bx = []; by = [];
for a = [-60 -45 -30 -20 -10 0 10 20 30 45 60]
    big = any(a == [0 -30 30 -60 60]); r0 = R*0.86; r1 = r0 + 0.05 + 0.04*big;
    aa = pi/2 - deg2rad(a) + roll;
    bx = [bx r0*cos(aa) r1*cos(aa) NaN]; by = [by r0*sin(aa) r1*sin(aa) NaN]; %#ok<AGROW>
end
set(A.bank, 'XData', bx, 'YData', by);
set(A.rollTxt,  'String', sprintf('%+03d', round(rad2deg(roll))));
set(A.pitchTxt, 'String', sprintf('%+03d', round(rad2deg(pitch))));

% --- HSI: heading-up arc ---
cur = ekf_hdg; Hc = A.Hc; Rc = A.Rc; Hf = A.Hf;
xs = []; ys = []; li = 0;
for b = (cur-46):(cur+46)
    if mod(round(b),5) ~= 0, continue; end
    big = mod(round(b),10) == 0; aa = pi/2 - deg2rad(b-cur)*Hf;
    r0 = Rc - (0.10 + 0.10*big);
    xs = [xs r0*cos(aa) Rc*cos(aa) NaN]; ys = [ys Hc+r0*sin(aa) Hc+Rc*sin(aa) NaN]; %#ok<AGROW>
    if big && li < numel(A.hsiLbl)
        li = li + 1; rl = Rc - 0.30;
        set(A.hsiLbl(li), 'Position', [rl*cos(aa) Hc+rl*sin(aa) 0], ...
            'String', sprintf('%02d', mod(round(b/10),36)), 'Visible', 'on');
    end
end
set(A.hsiTicks, 'XData', xs, 'YData', ys);
for j = li+1:numel(A.hsiLbl), set(A.hsiLbl(j), 'Visible', 'off'); end
% ground-truth heading tick
dg = mod(gt_hdg - cur + 180, 360) - 180;
if abs(dg) <= 46
    aa = pi/2 - deg2rad(dg)*Hf; r0 = Rc - 0.22;
    set(A.gtTick, 'XData', [r0*cos(aa) Rc*cos(aa)], 'YData', [Hc+r0*sin(aa) Hc+Rc*sin(aa)], 'Visible', 'on');
    rl = Rc - 0.40; set(A.gtLbl, 'Position', [rl*cos(aa) Hc+rl*sin(aa) 0], 'Visible', 'on');
else
    set(A.gtTick, 'Visible', 'off'); set(A.gtLbl, 'Visible', 'off');
end
set(A.hdgTxt, 'String', sprintf('%03d', round(mod(cur,360))));
% wind (digital)
spd = hypot(wind_ned(1), wind_ned(2));
wfrom = mod(rad2deg(atan2(-wind_ned(2), -wind_ned(1))), 360);
set(A.windDir, 'String', sprintf('%03d\\circ', round(wfrom)));
set(A.windSpd, 'String', sprintf('%.1f m/s', spd));
set(A.gtHdgTxt,  'String', sprintf('GT  %03d', round(mod(gt_hdg,360))));
set(A.ekfHdgTxt, 'String', sprintf('EKF %03d', round(mod(cur,360))));
end

% Polygon of the disk (radius R, centre origin) on the +u side of the line
% offset h along u -- the sky cap for the round ADI.
function P = halfDiskPoly(R, u, h)
P = [];
if h >= R, return; end
t = [u(2); -u(1)];
th = linspace(0, 2*pi, 240); circ = [R*cos(th); R*sin(th)];
pu_ = u'*circ; pt_ = t'*circ;
keep = pu_ >= h;
if ~any(keep), return; end
a = atan2(pt_(keep), pu_(keep)); pts = circ(:, keep);
[~, ord] = sort(a); P = pts(:, ord);
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
    'HorizontalAlignment', 'left', 'String', 'SYNAPLINE GCS');
uicontrol(pan, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.008 0.10 0.120 0.30], 'BackgroundColor', T.bg, ...
    'ForegroundColor', T.sub, 'FontSize', 7, ...
    'HorizontalAlignment', 'left', 'String', 'MATLAB SIL · PIXHAWK 6X · NEO-M9N');

stripCap(pan, 0.125, 0.085, 'MODE');
S.mode = stripVal(pan, 0.125, 0.085, T.good, 14);

% Option-B strip: ALT / GS / V-S annunciators only. HDG, N, E dropped --
% heading lives in the HSI, N/E in the waypoint table.
caps = {'ALT  m', 'GS  m/s', 'V/S  m/s'};
flds = {'alt', 'gs', 'vs'};
x0 = 0.225; w = 0.095;
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
function updateStatusStrip(S, mode, alt, gs, vs, t, ...
                           armed, use_est, link_on)
set(S.mode, 'String', upper(mode));
set(S.alt, 'String', sprintf('%7.1f', alt));
set(S.gs,  'String', sprintf('%7.1f', gs));
set(S.vs,  'String', sprintf('%+7.1f', vs));
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
newVis = 'off';
if strcmp(get(h, 'Visible'), 'on'), newVis = 'off'; else, newVis = 'on'; end
set(h, 'Visible', newVis);
arrow = getappdata(h, 'arrow');
if ~isempty(arrow) && ishandle(arrow)
    set(arrow, 'Visible', newVis);
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

% Left-click: add waypoint. Right-click: start pan drag. Double-click: reset view.
function onMapButton(axm, fig, altEdit, HALF)
cp = get(axm, 'CurrentPoint');
E = cp(1,1); N = cp(1,2);
switch get(ancestor(axm,'figure'), 'SelectionType')
    case 'normal'   % left single-click -> add waypoint
        if abs(E) > HALF || abs(N) > HALF, return; end
        alt = str2double(get(altEdit, 'String'));
        if ~isfinite(alt), alt = 10; end
        setappdata(fig, 'map_add_request', [N, E, -alt]);
    case 'alt'      % right-click -> start pan; store click position + current limits
        setappdata(fig, 'map_pan_start', [E, N, xlim(axm), ylim(axm)]);
        setappdata(axm, 'follow', false);   % manual pan takes over from follow
    case 'open'     % double-click -> reset to full extent + re-engage follow
        HALF2 = getappdata(axm, 'HALF');
        xlim(axm, [-HALF2 HALF2]); ylim(axm, [-HALF2 HALF2]);
        setappdata(axm, 'follow', true);
end
end

% Scroll-wheel zoom centred on the cursor, only when over the map axes.
function onMapScroll(axm, ev)
if ~ishandle(axm), return; end
hfig = ancestor(axm, 'figure');
fp = get(hfig, 'CurrentPoint');
ap = getpixelposition(axm, true);
if fp(1) < ap(1) || fp(1) > ap(1)+ap(3) || fp(2) < ap(2) || fp(2) > ap(2)+ap(4)
    return;
end
factor = 1.15 ^ (-ev.VerticalScrollCount);   % scroll up = zoom in
cp = get(axm, 'CurrentPoint');
cx = cp(1,1); cy = cp(1,2);
xlim(axm, cx + (xlim(axm) - cx) * factor);
ylim(axm, cy + (ylim(axm) - cy) * factor);
setappdata(axm, 'follow', false);            % manual zoom takes over from follow
end

% Dynamic-map follow: smoothly drag the viewport to keep the vehicle in view.
% While follow is on, if the vehicle leaves a centred dead-zone (inner DZ of
% the current view) the view is lerped toward re-centring on it -- so the map
% "drags" near the edges but holds still in the middle. Zoom is preserved.
% Manual pan/zoom disables follow; double-click reset re-engages it.
function mapFollow(axm, E, N)
if ~ishandle(axm) || ~getappdata(axm, 'follow'), return; end
xl = xlim(axm); yl = ylim(axm);
cx = mean(xl); cy = mean(yl); wx = diff(xl); wy = diff(yl);
DZ = 0.55;                                    % dead-zone = inner 55% of view
if abs(E - cx) <= DZ*wx/2 && abs(N - cy) <= DZ*wy/2
    return;                                   % inside dead-zone: hold still
end
a = 0.12;                                     % per-frame smoothing toward centre
cx = cx + a*(E - cx); cy = cy + a*(N - cy);
xlim(axm, [cx - wx/2, cx + wx/2]);
ylim(axm, [cy - wy/2, cy + wy/2]);
end

% Pan drag: called on every mouse move; acts only while a right-click is held.
function onMapPanMotion(fig, axm)
ps = getappdata(fig, 'map_pan_start');
if isempty(ps) || ~ishandle(axm), return; end
cp = get(axm, 'CurrentPoint');
dx = cp(1,1) - ps(1);  dy = cp(1,2) - ps(2);
xlim(axm, ps(3:4) - dx);
ylim(axm, ps(5:6) - dy);
end

% Clear pan state when any mouse button is released.
function onMapPanEnd(fig)
setappdata(fig, 'map_pan_start', []);
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

% Draw the pro flight-plan list on mm.planAx from the waypoint list (NED m).
% Columns: # | TYPE | N | E | ALT | DIST | BRG + totals footer. First WP =
% START (green), last = END (red), rest = WP (amber) -- same colour
% semantics as the map markers. Row geometry + wps are stashed in appdata
% for the click-to-edit handler. All children HitTest-off so clicks reach
% the axes ButtonDownFcn.
function drawPlanList(mm, wps)
T = gcsTheme(); ax = mm.planAx;
if ~isvalid(ax), return; end
cla(ax); set(ax, 'XLim', [0 1], 'YLim', [0 1]);
sel = getappdata(ax, 'sel'); n = size(wps, 1);
setappdata(ax, 'wps', wps);
if n == 0
    setappdata(ax, 'sel', 0); setappdata(ax, 'rowtops', []); setappdata(ax, 'rowh', 0);
    text(ax, 0.5, 0.5, 'NO WAYPOINTS', 'Color', T.sub, 'FontName', T.mono, 'FontSize', 9, ...
        'HorizontalAlignment', 'center', 'HitTest', 'off');
    set(mm.altCell, 'String', '', 'Enable', 'off');
    set(mm.altLbl, 'String', 'CLICK A WAYPOINT TO EDIT ALT');
    return;
end
if sel > n, sel = 0; setappdata(ax, 'sel', 0); end
cx = [0.03 0.10 0.34 0.50 0.64 0.78 0.92];
hdr = {'#','TYPE','N','E','ALT','DIST','BRG'}; hy = 0.95;
line(ax, [0.01 0.99], [hy-0.04 hy-0.04], 'Color', T.edge, 'LineWidth', 0.8, 'HitTest', 'off');
for c = 1:7
    text(ax, cx(c), hy, hdr{c}, 'Color', T.sub, 'FontName', T.mono, 'FontSize', 7.5, ...
        'FontWeight', 'bold', 'HorizontalAlignment', 'left', 'HitTest', 'off');
end
top = hy - 0.07; bot = 0.12; rowh = min(0.13, (top-bot)/n);
tops = zeros(n,1); tot = 0;
for i = 1:n
    yc = top - (i-0.5)*rowh; tops(i) = top - (i-1)*rowh;
    if i == sel
        rectangle(ax, 'Position', [0.01 tops(i)-rowh 0.98 rowh], 'FaceColor', mixc(T.acft,T.bg,0.84), ...
            'EdgeColor', T.acft, 'LineWidth', 0.8, 'HitTest', 'off');
    end
    Nn = wps(i,1); Ee = wps(i,2); alt = -wps(i,3);
    if i==1,     tp='START'; bc=T.good;
    elseif i==n, tp='END';   bc=T.bad;
    else,        tp='WP';    bc=T.acft; end
    if i>=2, leg = hypot(wps(i,1)-wps(i-1,1), wps(i,2)-wps(i-1,2));
             brg = mod(atan2d(wps(i,2)-wps(i-1,2), wps(i,1)-wps(i-1,1)),360); tot=tot+leg;
    else,    leg = NaN; brg = NaN; end
    text(ax, cx(1), yc, sprintf('%02d',i), 'Color', T.text, 'FontName', T.mono, 'FontSize', 8, 'FontWeight','bold','HitTest','off');
    rectangle(ax, 'Position', [cx(2) yc-0.035 0.20 0.07], 'Curvature', 0.5, 'FaceColor', mixc(bc,T.bg,0.80), 'EdgeColor', bc, 'LineWidth', 0.8, 'HitTest','off');
    text(ax, cx(2)+0.10, yc, tp, 'Color', bc, 'FontName', T.mono, 'FontSize', 6.5, 'FontWeight','bold','HorizontalAlignment','center','HitTest','off');
    text(ax, cx(3), yc, sprintf('%.0f',Nn),  'Color', T.text, 'FontName', T.mono, 'FontSize', 8, 'HitTest','off');
    text(ax, cx(4), yc, sprintf('%.0f',Ee),  'Color', T.text, 'FontName', T.mono, 'FontSize', 8, 'HitTest','off');
    text(ax, cx(5), yc, sprintf('%.0f',alt), 'Color', T.acft, 'FontName', T.mono, 'FontSize', 8, 'FontWeight','bold','HitTest','off');
    if i>=2
        text(ax, cx(6), yc, sprintf('%.0f',leg),  'Color', T.sub, 'FontName', T.mono, 'FontSize', 7.5, 'HitTest','off');
        text(ax, cx(7), yc, sprintf('%03.0f',brg), 'Color', T.sub, 'FontName', T.mono, 'FontSize', 7.5, 'HitTest','off');
    else
        text(ax, cx(6), yc, '--', 'Color', T.sub, 'FontName', T.mono, 'FontSize', 7.5, 'HitTest','off');
        text(ax, cx(7), yc, '--', 'Color', T.sub, 'FontName', T.mono, 'FontSize', 7.5, 'HitTest','off');
    end
end
setappdata(ax, 'rowtops', tops); setappdata(ax, 'rowh', rowh);
line(ax, [0.01 0.99], [0.10 0.10], 'Color', T.edge, 'LineWidth', 0.8, 'HitTest','off');
text(ax, 0.03, 0.05, sprintf('TOTAL %.0f m', tot), 'Color', T.good, 'FontName', T.mono, 'FontSize', 8, 'FontWeight','bold','HitTest','off');
text(ax, 0.97, 0.05, sprintf('%d WP', n), 'Color', T.sub, 'FontName', T.mono, 'FontSize', 8, 'FontWeight','bold','HorizontalAlignment','right','HitTest','off');
end

% Click a plan row -> select it and load its altitude into the editor.
function onPlanClick(mm)
ax = mm.planAx; wps = getappdata(ax, 'wps');
if isempty(wps), return; end
tops = getappdata(ax, 'rowtops'); rowh = getappdata(ax, 'rowh');
cp = get(ax, 'CurrentPoint'); dy = cp(1,2); sel = 0;
for i = 1:numel(tops)
    if dy <= tops(i) && dy > tops(i)-rowh, sel = i; break; end
end
if sel == 0, return; end
setappdata(ax, 'sel', sel);
set(mm.altCell, 'String', sprintf('%.4g', -wps(sel,3)), 'Enable', 'on');
set(mm.altLbl, 'String', sprintf('EDIT ALT OF WP %02d (m, +up)', sel));
drawPlanList(mm, wps);
end

% Commit the editor value as the selected waypoint's altitude (same path as
% the old table: alt_edit_request = [row alt_m], consumed by the sim loop).
function onAltCellEdit(mm)
sel = getappdata(mm.planAx, 'sel');
if sel < 1, return; end
v = str2double(get(mm.altCell, 'String'));
if isfinite(v), setappdata(mm.fig, 'alt_edit_request', [sel, v]); end
end

% Redraw the Mission-map markers + table from the waypoint list (NED metres).
function updateMissionMap(mm, wps)
if ~isstruct(mm) || ~isfield(mm, 'ax') || ~isvalid(mm.ax), return; end
old = getappdata(mm.ax, 'wp_labels');           % clear old coordinate labels
if ~isempty(old), delete(old(isgraphics(old))); end
if isempty(wps)
    set([mm.path mm.launch mm.mid mm.endp], 'XData', NaN, 'YData', NaN);
    drawPlanList(mm, []);
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
drawPlanList(mm, wps);

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
                'BackgroundColor', T.panel, 'BorderType', 'none');

% Variant-B "HUD console" decoration layer drawn behind the controls:
% faint scanlines, corner brackets, accent header. PickableParts off so it
% never intercepts clicks meant for the sliders/edits on top.
axd = axes('Parent', panel, 'Units', 'normalized', 'Position', [0 0 1 1], ...
           'XLim', [0 1], 'YLim', [0 1], 'Color', 'none', ...
           'HandleVisibility', 'off', 'PickableParts', 'none');
hold(axd, 'on'); axis(axd, 'off');
for yy = 0.05:0.05:0.85
    line(axd, [0.015 0.985], [yy yy], 'Color', mixc(T.panel, T.bg, 0.45), ...
         'LineWidth', 0.2, 'HitTest', 'off');
end
groupBrackets(axd, T.data);
text(axd, 0.03, 0.95, ['\diamondsuit  ' upper(title)], 'Color', T.data, ...
     'FontName', T.mono, 'FontSize', 9, 'FontWeight', 'bold', ...
     'Interpreter', 'tex', 'HitTest', 'off');
line(axd, [0.015 0.985], [0.905 0.905], 'Color', T.edge, 'LineWidth', 0.8, 'HitTest', 'off');

nr          = numel(rows);
reset_fns   = cell(nr, 1);
refresh_fns = cell(nr, 1);

top  = 0.86;                 % rows start below the header
bot  = 0.015;
rowh = (top - bot) / nr;
for r = 1:nr
    [reset_fns{r}, refresh_fns{r}] = makeRow(panel, top - r * rowh, rowh, rows{r});
end

uicontrol(panel, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.70 0.915 0.28 0.072], 'String', 'RESTORE DEFAULTS', ...
    'FontSize', 7, 'FontName', T.mono, 'FontWeight', 'bold', ...
    'BackgroundColor', T.btn, 'ForegroundColor', T.warn, ...
    'Callback', @(~,~) resetGroup(reset_fns));

% Re-read live values into this group's sliders/edits (no controller write).
group_refresh = @() resetGroup(refresh_fns);
end

% Four L-shaped corner brackets around a group (variant-B frame).
function groupBrackets(axd, col)
L = 0.05; x0 = 0.012; x1 = 0.988; y0 = 0.015; y1 = 0.985;
P = {[x0 x0+L; y0 y0], [x0 x0; y0 y0+L], [x1-L x1; y0 y0], [x1 x1; y0 y0+L], ...
     [x0 x0+L; y1 y1], [x0 x0; y1-L y1], [x1-L x1; y1 y1], [x1 x1; y1-L y1]};
for i = 1:numel(P)
    line(axd, P{i}(1,:), P{i}(2,:), 'Color', col, 'LineWidth', 1.6, 'HitTest', 'off');
end
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
axisCols = {T.data, T.acft, T.nav};     % per-axis LCD colour (variant B)
uicontrol(panel, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.02 ybot + 0.08*rowh 0.30 0.80*rowh], ...
    'BackgroundColor', T.panel, 'ForegroundColor', T.text, ...
    'HorizontalAlignment', 'left', 'FontName', T.mono, 'FontSize', 8, ...
    'String', upper(spec.label), 'TooltipString', spec.tip);

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
        'BackgroundColor', T.btn, ...
        'Min', lo(c), 'Max', hi(c), ...
        'Value', min(max(vals(c), lo(c)), hi(c)), 'TooltipString', spec.tip);
    edits(c) = uicontrol(panel, 'Style', 'edit', 'Units', 'normalized', ...
        'Position', [x0, ybot + 0.06*rowh, ew*0.92, 0.40*rowh], ...
        'BackgroundColor', T.field, 'ForegroundColor', axisCols{min(c,3)}, ...
        'FontName', T.mono, 'FontSize', 8.5, 'FontWeight', 'bold', ...
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

function updateHeadingArrow(h, E, N, hdg)
% Update a filled triangle at (E,N) pointing along the vehicle HEADING hdg
% (rad, 0 = North, + = East) — i.e. where the nose points, NOT course over
% ground. Light complex-mean smoothing (handles wrap) removes per-frame yaw
% jitter, mainly from the EKF feed, while still tracking turns promptly.
if ~ishandle(h), return; end
ud = get(h, 'UserData');
if ~isnan(hdg)
    if isnan(ud.hdg)
        ud.hdg = hdg;
    else
        alpha = 0.30;           % ~3-frame time constant at 20-30 Hz
        z = (1-alpha)*exp(1j*ud.hdg) + alpha*exp(1j*hdg);
        ud.hdg = angle(z);
    end
    set(h, 'UserData', ud);
end
if isnan(ud.hdg), return; end
sz = ud.sz;
% Unit direction vector in map coords [E, N]
dx = sin(ud.hdg); dy = cos(ud.hdg);
% Perpendicular (left side of arrow)
px = -dy; py = dx;
W = sz * 0.40;   % half-width of arrow base
% Triangle: tip, base-left, base-right
set(h, 'XData', [E,               E - sz*dx + W*px,  E - sz*dx - W*px], ...
       'YData', [N,               N - sz*dy + W*py,  N - sz*dy - W*py]);
end
