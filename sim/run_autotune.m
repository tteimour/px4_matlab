function results = run_autotune(do_plot)
% Drive the PX4 system-identification attitude auto-tuner against the
% MATLAB QuadrotorDynamics plant, end to end.
%
% Mirrors the PX4 in-flight procedure: the multicopter holds position
% (hover) while McAutotuneAttitudeControl injects a decreasing-period
% square-wave rate setpoint on one body axis at a time, identifies the
% torque->rate model online (ArxRls / SystemIdentification), and designs
% rate-loop PID gains with the GMVC law (pid_design_gmvc). The attitude P
% gain follows the PX4 "60 deg error -> max output" rule.
%
% This is the MATLAB equivalent of running `MC_AT_START = 1` on a PX4
% vehicle, with the identification done against our own plant model.
%
%   results = run_autotune()        % run + plot + print
%   results = run_autotune(false)   % run + print, no figures
%
% Returns the struct produced by McAutotuneAttitudeControl.getResults().
%
% Gyro model. The tuner consumes a *modeled* body-rate signal, not the
% noise-free ground truth, exactly as PX4 consumes vehicle_angular_velocity
% from the gyro. This matters: PX4's recursive least-squares relies on the
% persistent broadband excitation of a real (vibration-rich) gyro to make
% the autoregressive part of the model observable. A noise-free rate makes
% consecutive samples y(k-1), y(k-2) collinear, so the [a1, a2] covariance
% never converges. The noise here uses the project's own ICM-45686 model
% (thermal density gyro_nd and motor-vibration gain gyro_vib_gain, both
% cited from the datasheet in src/sensors/imu/ImuICM45686.m), applied with
% the same formula as ImuSensor.measure(). The inner rate loop keeps using
% the ground-truth rate so the hover stays clean and the identification is
% isolated to what the tuner sees.

if nargin < 1, do_plot = true; end

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(fullfile(root, 'src', 'params'));
addpath(fullfile(root, 'src', 'math'));
addpath(fullfile(root, 'src', 'plant'));
addpath(fullfile(root, 'src', 'controllers'));
addpath(fullfile(root, 'src', 'autotune'));
addpath(fullfile(root, 'src', 'sensors'));
addpath(fullfile(root, 'src', 'sensors', 'imu'));

p = px4_params();

% =====================================================================
% Sim modules (ground-truth state feed; the tuner uses the true body
% rate as PX4 would use the filtered gyro).
% =====================================================================
hover_alt = 5.0;                       % m above ground
plant    = QuadrotorDynamics(p);
plant.reset([0; 0; -hover_alt], 0);
pos_ctl  = PositionController(p);
att_ctl  = AttitudeController(p);
rate_ctl = RateController(p);
alloc    = ControlAllocator(p);

% Auto-tuner. apply_mode = 0 -> identify + design + report only (the
% safe "logging" mode). IMU_GYRO_CUTOFF = 40 Hz is the PX4 default.
at_opts.apply_mode  = 0;
at_opts.gyro_cutoff = 40.0;            % IMU_GYRO_CUTOFF default
at_opts.sysid_amp   = 0.7;             % MC_AT_SYSID_AMP default
at_opts.rise_time   = 0.14;            % MC_AT_RISE_TIME default
at_opts.log_enable  = true;
at = McAutotuneAttitudeControl(p, at_opts);

% Gyro noise model (see header). gyro_nd / gyro_vib_gain come straight
% from the ICM-45686 chip model; the discrete std follows
% ImuSensor.measure(): thermal = gyro_nd/sqrt(dt), vibration scales with
% motor activity (vib_level ~ norm(motor_cmd)).
imu        = ImuICM45686([], [], []);
gyro_sig_th  = imu.gyro_nd / sqrt(1 / p.rate_hz.rate);   % rad/s, thermal
gyro_vib_gain = imu.gyro_vib_gain;                       % rad/s per unit vib
rng(1, 'twister');                                       % reproducible run

% =====================================================================
% Loop rates (match px4_params).
% =====================================================================
dt_rate = 1 / p.rate_hz.rate;          % 1 kHz inner loop
n_att   = round(p.rate_hz.rate / p.rate_hz.attitude);
n_pos   = round(p.rate_hz.rate / p.rate_hz.position);
dt_pos  = n_pos * dt_rate;

pos_sp = [0; 0; -hover_alt];
yaw_sp = 0;

q_sp          = [1; 0; 0; 0];
thrust_body_z = -p.pos.thr_hover;
rate_sp       = [0; 0; 0];

settle_time = 2.0;                      % hover stabilization before tuning
t_max       = 90.0;                     % hard stop (s)

n_max = round(t_max / dt_rate);
log.t       = nan(n_max, 1);
log.omega   = nan(n_max, 3);
log.inj     = nan(n_max, 3);
log.torque  = nan(n_max, 3);
log.state   = nan(n_max, 1);
li = 0;

fprintf('Hovering %.0f s to settle, then starting autotune...\n', settle_time);

k = 0;
t_sim = 0;
started = false;
m = [0; 0; 0; 0];

while t_sim < t_max
    s = plant.state();
    omega_true = s.angular_vel_b;
    q          = s.attitude_q;

    % Modeled gyro the tuner sees: ground truth + thermal + vibration
    % noise (vibration scales with current motor activity).
    vib        = norm(m);
    omega_meas = omega_true ...
        + gyro_sig_th * randn(3, 1) ...
        + gyro_vib_gain * vib * randn(3, 1);

    % Kick off the identification once the hover has settled.
    if ~started && t_sim >= settle_time
        at.start();
        started = true;
        fprintf('t=%.1fs: autotune started.\n', t_sim);
    end

    % --- Position loop (hold hover) at 50 Hz ---
    if mod(k, n_pos) == 0
        [q_sp, thrust_body_z, ~, ~, ~] = pos_ctl.update( ...
            s.position_ned, s.velocity_ned, s.acceleration_ned, ...
            pos_sp, yaw_sp, [], [], dt_pos);
    end

    % --- Attitude loop at 250 Hz ---
    if mod(k, n_att) == 0
        rate_sp = att_ctl.update(q, q_sp, NaN);
    end

    % --- Inject the identification signal and close the rate loop ---
    inj = at.injection();                       % held excitation (3x1)
    rate_sp_total = rate_sp + inj;

    landed = false;                             % hovering at altitude
    torque = rate_ctl.update(omega_true, rate_sp_total, [0; 0; 0], dt_rate, landed);
    T_mag  = max(0, -thrust_body_z);
    [m, sat_pos, sat_neg] = alloc.allocate(torque, T_mag);
    rate_ctl.setSaturationStatus(sat_pos, sat_neg);

    plant.step(m, dt_rate, [0; 0; 0]);

    % --- Feed the measurement to the tuner (causal: torque/gyro that
    %     produced this step) ---
    if started
        at.step(dt_rate, torque, omega_meas, true, [0; 0]);
    end

    % --- Log ---
    li = li + 1;
    log.t(li)        = t_sim;
    log.omega(li, :) = omega_meas';
    log.inj(li, :)   = inj';
    log.torque(li, :) = torque';
    log.state(li)    = at.state;

    k = k + 1;
    t_sim = t_sim + dt_rate;

    if started && at.isDone()
        fprintf('t=%.1fs: autotune finished (state = %s).\n', t_sim, at.stateName());
        break;
    end
end

% Trim logs.
fn = fieldnames(log);
for i = 1:numel(fn)
    log.(fn{i}) = log.(fn{i})(1:li, :);
end

results = at.getResults();
printResults(results, at);

if do_plot
    plotAutotune(at, log, here);
end
end


% =====================================================================
function printResults(r, at) %#ok<INUSD>
axes_lbl = {'roll ', 'pitch', 'yaw  '};

fprintf('\n=====================================================================\n');
fprintf(' PX4 system-identification autotune — results\n');
fprintf(' final state: %s   (gains pass verification: %d)\n', r.state, r.success);
fprintf('=====================================================================\n');

fprintf('\nIdentified discrete-time models  G(q^-1) = B(q^-1)/A(q^-1)\n');
fprintf('  (coefficients scaled by input_scale, as used for design)\n');
fprintf('  %-6s | %-9s %-9s %-9s | %-9s %-9s\n', 'axis', 'b0', 'b1', 'b2', 'a1', 'a2');
for ax = 1:3
    c = r.id_coeff(:, ax);    % [a1; a2; b0; b1; b2]
    fprintf('  %-6s | %+9.4f %+9.4f %+9.4f | %+9.4f %+9.4f\n', ...
        axes_lbl{ax}, c(3), c(4), c(5), c(1), c(2));
end

fprintf('\nDesigned gains (standard form: u = kc*(1 + ki*dt + kd/dt)*e)\n');
fprintf('  %-6s | %-9s %-9s %-9s | %-9s\n', 'axis', 'kc', 'ki', 'kd', 'att_p');
for ax = 1:3
    fprintf('  %-6s | %9.4f %9.4f %9.4f | %9.4f\n', ...
        axes_lbl{ax}, r.rate_k(ax), r.rate_i(ax), r.rate_d(ax), r.att_p(ax));
end

t = r.tuned;
fprintf('\nNominal vs tuned PX4 parameters (parallel form, MC_*RATE_K = 1):\n');
fprintf('  %-16s %-12s %-12s\n', 'param', 'nominal', 'tuned');
rows = {
    'MC_ROLLRATE_P',  r.nom_rate_p(1),                 t.MC_ROLLRATE_P
    'MC_ROLLRATE_I',  r.nom_rate_p(1)*r.nom_rate_i(1), t.MC_ROLLRATE_I
    'MC_ROLLRATE_D',  r.nom_rate_p(1)*r.nom_rate_d(1), t.MC_ROLLRATE_D
    'MC_ROLL_P',      r.nom_att_p(1),                  t.MC_ROLL_P
    'MC_PITCHRATE_P', r.nom_rate_p(2),                 t.MC_PITCHRATE_P
    'MC_PITCHRATE_I', r.nom_rate_p(2)*r.nom_rate_i(2), t.MC_PITCHRATE_I
    'MC_PITCHRATE_D', r.nom_rate_p(2)*r.nom_rate_d(2), t.MC_PITCHRATE_D
    'MC_PITCH_P',     r.nom_att_p(2),                  t.MC_PITCH_P
    'MC_YAWRATE_P',   r.nom_rate_p(3),                 t.MC_YAWRATE_P
    'MC_YAWRATE_I',   r.nom_rate_p(3)*r.nom_rate_i(3), t.MC_YAWRATE_I
    'MC_YAWRATE_D',   r.nom_rate_p(3)*r.nom_rate_d(3), t.MC_YAWRATE_D
    'MC_YAW_P',       r.nom_att_p(3),                  t.MC_YAW_P };
for i = 1:size(rows, 1)
    fprintf('  %-16s %-12.4f %-12.4f\n', rows{i, 1}, rows{i, 2}, rows{i, 3});
end

if ~r.success
    fprintf(['\nNOTE: the gain set did not pass PX4''s areGainsGood() bounds.\n' ...
             '      The per-axis gains above are still the design output and are\n' ...
             '      reported for inspection (PX4 would mark the run as FAIL).\n']);
end
fprintf('\n');
end


% =====================================================================
function plotAutotune(at, log, outdir)
h = at.hist;
if isempty(h.t)
    warning('No autotune history logged; skipping plots.');
    return;
end

% ---- Figure 1: identification convergence (coefficients + variances) ----
f1 = figure('Name', 'Autotune — identification', 'Color', 'w', ...
            'Position', [80 80 1100 720]);

subplot(2, 2, 1);
plot(h.t, h.coeff', 'LineWidth', 1.0); grid on;
xlabel('time (s)'); ylabel('coefficient value');
title('Identified ARX coefficients (scaled)');
legend({'a_1', 'a_2', 'b_0', 'b_1', 'b_2'}, 'Location', 'best');

subplot(2, 2, 2);
semilogy(h.t, h.var', 'LineWidth', 1.0); grid on; hold on;
yline(at.converged_thr, 'k--', 'conv. thr');
xlabel('time (s)'); ylabel('parameter variance');
title('RLS covariance diagonal (convergence)');

subplot(2, 2, 3);
plot(h.t, h.fitness, 'LineWidth', 1.0); grid on;
xlabel('time (s)'); ylabel('fitness');
title('Fitness (low-passed |\Delta\theta|/dt)');

subplot(2, 2, 4);
plot(h.t, [h.kc; h.ki; h.kd]', 'LineWidth', 1.0); grid on; hold on;
plot(h.t, h.att_p, 'LineWidth', 1.2);
xlabel('time (s)'); ylabel('gain');
title('GMVC-designed gains (current axis)');
legend({'kc', 'ki', 'kd', 'att\_p'}, 'Location', 'best');

saveFig(f1, fullfile(outdir, 'autotune_identification.png'));

% ---- Figure 2: excitation vs response per axis ----
f2 = figure('Name', 'Autotune — excitation/response', 'Color', 'w', ...
            'Position', [120 120 1100 720]);
lbl = {'roll', 'pitch', 'yaw'};
for ax = 1:3
    subplot(3, 1, ax);
    plot(log.t, log.inj(:, ax), 'LineWidth', 1.0); hold on; grid on;
    plot(log.t, log.omega(:, ax), 'LineWidth', 1.0);
    xlabel('time (s)'); ylabel(sprintf('%s (rad/s)', lbl{ax}));
    legend({'injected rate sp', 'measured body rate'}, 'Location', 'best');
    title(sprintf('%s-axis excitation and response', lbl{ax}));
end
saveFig(f2, fullfile(outdir, 'autotune_excitation.png'));

fprintf('Saved plots to:\n  %s\n  %s\n', ...
    fullfile(outdir, 'autotune_identification.png'), ...
    fullfile(outdir, 'autotune_excitation.png'));
end


function saveFig(f, path)
try
    exportgraphics(f, path, 'Resolution', 120);
catch
    print(f, '-dpng', '-r120', path);
end
end
