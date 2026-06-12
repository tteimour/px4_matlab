function diag_ekf_oscillation()
% Headless closed-loop hover diagnostic for the "roll oscillation with EKF
% feed" symptom. No GUI. Reuses the real EstimatorBus + controllers exactly
% as run_interactive wires them, but lets us MIX truth vs EKF signals into
% the inner loop so we can localize the oscillation to the attitude-estimate
% path or the rate-estimate path, then probe the implicated path.
%
% Scenario knobs (per cfg):
%   att, rate, posvel \in {'truth','ekf'}  -- source of each controller input
%   att_gain                                -- override OutputPredictor.att_gain
%   gyr_b_noise_scale                       -- scale EKF gyro-bias process noise
%   init_accel_b_var                        -- override EKF accel-bias init var
%
% A roll-rate impulse (kick) is applied at t_kick to measure ring-down
% (overshoot-and-return). Default config = GNSS aiding, zero true IMU bias,
% zero wind -- matching run_interactive's defaults so gravity fusion is
% suppressed (gnss_origin_set) exactly as in the reported case.

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(fullfile(root, 'src', 'params'));
addpath(fullfile(root, 'src', 'math'));
addpath(fullfile(root, 'src', 'plant'));
addpath(fullfile(root, 'src', 'controllers'));
addpath(fullfile(root, 'src', 'sensors'));
addpath(fullfile(root, 'src', 'sensors', 'imu'));
addpath(fullfile(root, 'src', 'sensors', 'baro'));
addpath(fullfile(root, 'src', 'sensors', 'mag'));
addpath(fullfile(root, 'src', 'sensors', 'gnss'));
addpath(fullfile(root, 'src', 'sensors', 'voter'));
addpath(fullfile(root, 'src', 'estimator'));

p     = px4_params();
earth = EarthModel(47.39773, 8.54559, 488.0);

T_total = 20.0;     % s
t_kick  = 10.0;     % s   roll-rate impulse time (after EKF convergence)
kick    = 0.40;     % rad/s

base = struct('att','truth','rate','truth','posvel','truth', ...
              'T_total',T_total,'t_kick',t_kick,'kick',kick);

efull = setfields(base, 'name','ekf_all', 'att','ekf','rate','ekf','posvel','ekf');
% --- round 6: reproduce the "step in roll/pitch while climbing" ---
% climb_to: step the altitude setpoint to this NED z at climb_t (m).
% --- round 7: gyro low-pass (PX4 IMU_GYRO_CUTOFF) effect on rate-loop jitter ---
% Pure hover (no kick) so the metric is clean actuator activity.
hov = setfields(base, 't_kick', 0);
S = {};
S{end+1} = setfields(hov,  'name','truth',      'att','truth','rate','truth','posvel','truth');
S{end+1} = setfields(efull,'name','ekf_lpf_off','t_kick',0,'gyro_lpf_hz',0);   % raw gyro (pre-fix)
S{end+1} = setfields(efull,'name','ekf_lpf_40', 't_kick',0,'gyro_lpf_hz',40);  % PX4 default
S{end+1} = setfields(efull,'name','ekf_lpf_20', 't_kick',0,'gyro_lpf_hz',20);

logs = cell(size(S));
for i = 1:numel(S)
    fprintf('--- running scenario: %s ---\n', S{i}.name);
    logs{i} = run_one(p, earth, S{i});
end

report(S, logs, t_kick);
makeplot(S, logs, t_kick, fullfile(here, 'diag_ekf_oscillation.png'));
fprintf('\nSaved figure -> %s\n', fullfile(here, 'diag_ekf_oscillation.png'));
end

% =========================================================================
function L = run_one(p, earth, cfg)
plant    = QuadrotorDynamics(p); plant.reset([0;0;-10], 0);
pos_ctl  = PositionController(p);
att_ctl  = AttitudeController(p);
rate_ctl = RateController(p);
alloc    = ControlAllocator(p);
est_bus  = EstimatorBus(earth);
est_bus.sensors.applyImuBias([0;0;0], [0;0;0]);   % match run_interactive defaults

if isfield(cfg,'att_gain'),          est_bus.output_pred.att_gain = cfg.att_gain; end
if isfield(cfg,'gyr_b_noise_scale'), est_bus.ekf.params.gyr_b_noise = est_bus.ekf.params.gyr_b_noise * cfg.gyr_b_noise_scale; end
needs_reset = false;
if isfield(cfg,'init_accel_b_var'),  est_bus.ekf.params.init_accel_b_var = cfg.init_accel_b_var; needs_reset = true; end
if isfield(cfg,'init_att_var'),      est_bus.ekf.params.init_att_var = cfg.init_att_var; needs_reset = true; end
% fusion-knockout knobs (EstimatorBus.params drives dispatch/gating;
% Ekf2.params drives in-fusion gates -- set both where relevant).
if isfield(cfg,'gyro_lpf_hz'), est_bus.setGyroCutoff(cfg.gyro_lpf_hz); end
if isfield(cfg,'mag_type'),  est_bus.params.mag_type = cfg.mag_type; end
if isfield(cfg,'mag_gate'),  est_bus.ekf.params.mag_gate = cfg.mag_gate; end
if isfield(cfg,'mag_noise'), est_bus.ekf.params.mag_noise = cfg.mag_noise; end
if isfield(cfg,'gps_ctrl'),  est_bus.params.gps_ctrl = cfg.gps_ctrl; est_bus.ekf.params.gps_ctrl = cfg.gps_ctrl; end
% force_gravity: keep GNSS pos/vel aiding (EstimatorBus.params bit0 set) but
% clear bit0 in the EKF's own copy so the gravity-fusion guard (Ekf2.m:413)
% does NOT fire -> gravity fuses continuously, restoring tilt observability.
if isfield(cfg,'force_gravity') && cfg.force_gravity
    est_bus.params.gps_ctrl     = 7;   % EstimatorBus still fuses GNSS pos+vel
    est_bus.ekf.params.gps_ctrl = 6;   % guard sees bit0=0 -> gravity not skipped
end
if needs_reset, est_bus.ekf.reset(); end   % re-init covariance with new diag

dt    = 1 / p.rate_hz.rate;
n_att = round(p.rate_hz.rate / p.rate_hz.attitude);
n_pos = round(p.rate_hz.rate / p.rate_hz.position);
N     = round(cfg.T_total / dt);

pos_sp        = [0;0;-10];  yaw_sp = 0;
q_sp          = [1;0;0;0];
thrust_body_z = -p.pos.thr_hover;
rate_sp       = [0;0;0];
m_last        = zeros(4,1);
kicked        = false;

climbing = isfield(cfg,'climb_to');
L.t        = (0:N-1)' * dt;
L.roll_t   = zeros(N,1);   % TRUE roll (deg) -- the physical oscillation
L.roll_e   = zeros(N,1);   % EKF-estimated roll (deg)
L.pitch_t  = zeros(N,1);   % TRUE pitch (deg)
L.pitch_e  = zeros(N,1);   % EKF-estimated pitch (deg)
L.p_t      = zeros(N,1);   % TRUE roll rate (deg/s)
L.gb_roll  = zeros(N,1);   % EKF gyro-bias, roll axis (deg/s)
L.posz_t   = zeros(N,1);
L.tau_roll = zeros(N,1);   % roll torque command (normalized) -- actuator activity

for k = 1:N
    t = (k-1) * dt;
    s_truth = plant.state();
    s_truth.vib_level = norm(m_last);

    est_bus.step(t, s_truth);
    s_ekf = est_bus.stateOut();

    % --- assemble controller-facing state by mixing sources ---
    if strcmp(cfg.posvel,'ekf')
        pos = s_ekf.position_ned; vel = s_ekf.velocity_ned; acc = s_ekf.acceleration_ned;
    else
        pos = s_truth.position_ned; vel = s_truth.velocity_ned; acc = s_truth.acceleration_ned;
    end
    if strcmp(cfg.att,'ekf'),  att_q = s_ekf.attitude_q;     else, att_q = s_truth.attitude_q;     end
    if strcmp(cfg.rate,'ekf'), omega = s_ekf.angular_vel_b;  else, omega = s_truth.angular_vel_b;  end

    if climbing && t >= cfg.climb_t
        pos_sp(3) = cfg.climb_to;   % step the altitude setpoint -> commanded climb
    end
    if mod(k-1, n_pos) == 0
        [q_sp, thrust_body_z] = pos_ctl.update(pos, vel, acc, pos_sp, yaw_sp, [], [], n_pos*dt);
    end
    if mod(k-1, n_att) == 0
        rate_sp = att_ctl.update(att_q, q_sp, 0);
    end
    torque = rate_ctl.update(omega, rate_sp, [0;0;0], dt, false);
    T_mag  = max(0, -thrust_body_z);
    [m, sp, sn] = alloc.allocate(torque, T_mag);
    rate_ctl.setSaturationStatus(sp, sn);
    m_last = m;

    plant.step(m, dt, [0;0;0]);

    if ~kicked && ~climbing && cfg.t_kick > 0 && t >= cfg.t_kick
        plant.omega_b(1) = plant.omega_b(1) + cfg.kick;   % roll-rate impulse
        kicked = true;
    end

    rpy_t = quat_to_euler(s_truth.attitude_q);
    rpy_e = quat_to_euler(s_ekf.attitude_q);
    L.roll_t(k)  = rad2deg(rpy_t(1));
    L.roll_e(k)  = rad2deg(rpy_e(1));
    L.pitch_t(k) = rad2deg(rpy_t(2));
    L.pitch_e(k) = rad2deg(rpy_e(2));
    L.p_t(k)     = rad2deg(s_truth.angular_vel_b(1));
    L.gb_roll(k) = rad2deg(est_bus.ekf.gyro_b(1));
    L.posz_t(k)  = s_truth.position_ned(3);
    L.tau_roll(k)= torque(1);
end
end

% =========================================================================
function report(S, logs, ~)
fprintf('\n==================== ATTITUDE + ACTUATOR METRICS ====================\n');
fprintf('%-14s | %8s | %8s | %10s | %10s\n', ...
        'scenario','roll_sd','pitch_sd','tau_sd','tau_jitter');
fprintf('%s\n', repmat('-',1,62));
for i = 1:numel(S)
    L = logs{i};  t = L.t;
    w = t >= 0.4 * t(end);            % steady window (last 60%)
    roll_sd  = std(L.roll_t(w));
    pitch_sd = std(L.pitch_t(w));
    tau_sd   = std(L.tau_roll(w));            % roll torque command spread
    tau_jit  = std(diff(L.tau_roll(w)));      % step-to-step change = HF actuator jitter
    fprintf('%-14s | %8.3f | %8.3f | %10.5f | %10.6f\n', ...
            S{i}.name, roll_sd, pitch_sd, tau_sd, tau_jit);
end
fprintf('%s\n', repmat('-',1,62));
fprintf(['roll_sd/pitch_sd : std of TRUE angle [deg], steady window\n' ...
         'tau_sd     : std of roll torque command (normalized) -- overall actuator activity\n' ...
         'tau_jitter : std of step-to-step torque change -- HIGH-FREQUENCY twitch the gyro LPF targets\n']);
end

% =========================================================================
function makeplot(S, logs, t_kick, outpath)
f = figure('Visible','off','Color','w','Position',[100 100 1200 900]);
co = lines(numel(S));
names = cellfun(@(s) s.name, S, 'uni', 0);

subplot(3,1,1); hold on; grid on;
for i = 1:numel(S)
    plot(logs{i}.t, logs{i}.roll_t, 'Color', co(i,:), 'LineWidth', 1.1);
end
ylabel('TRUE roll [deg]'); title('Roll vs time (all scenarios)');
legend(names, 'Interpreter','none','Location','eastoutside');

subplot(3,1,2); hold on; grid on;
for i = 1:numel(S)
    plot(logs{i}.t, logs{i}.pitch_t, 'Color', co(i,:), 'LineWidth', 1.1);
end
ylabel('TRUE pitch [deg]'); title('Pitch vs time (all scenarios)');
legend(names, 'Interpreter','none','Location','eastoutside');

% altitude profile + a representative ekf scenario's est-vs-true roll
idx = find(contains(names,'ekf'),1);  if isempty(idx), idx = 1; end
subplot(3,1,3); hold on; grid on;
yyaxis left;
plot(logs{idx}.t, -logs{idx}.posz_t, 'k-', 'LineWidth',1.2);
ylabel('altitude [m]');
yyaxis right;
plot(logs{idx}.t, logs{idx}.roll_t, 'b-', 'LineWidth',1.0);
plot(logs{idx}.t, logs{idx}.pitch_t,'r-', 'LineWidth',1.0);
ylabel('attitude [deg]');
xlabel('t [s]');
title(sprintf('%s: altitude (black), TRUE roll (blue), TRUE pitch (red)', names{idx}), 'Interpreter','none');
legend({'altitude','roll true','pitch true'},'Location','eastoutside');

exportgraphics(f, outpath, 'Resolution', 120);
close(f);
end

% =========================================================================
function c = setname(c, nm),  c.name = nm; end
function c = setfields(c, varargin)
for i = 1:2:numel(varargin), c.(varargin{i}) = varargin{i+1}; end
end
