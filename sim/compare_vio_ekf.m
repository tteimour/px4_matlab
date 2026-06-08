function out = compare_vio_ekf(bagPath, simLog)
%COMPARE_VIO_EKF  Compare OpenVINS VIO against the MATLAB EKF2 estimate.
%
% Aligns the OpenVINS trajectory (recorded from /ov_msckf/odomimu) to the
% MATLAB EKF2 / output-predictor estimate (sim_log.est_pos / est_vel, NED)
% and plots position (N/E/D + ATE error), velocity (N/E/D + speed), and the
% top-down / altitude trajectory.
%
% Why alignment: with init_dyn_use, OpenVINS' "global" frame is gravity
% aligned but its yaw and origin are arbitrary (unobservable gauge freedoms).
% A rigid SE3 (no scale) alignment removes them before error is computed --
% the standard ATE method (same as ov_eval). The IMU makes VIO metric, so no
% scale is fitted. VIO twist.linear is in the IMU body frame
% (ROS2Visualizer.cpp:301), so it is compared as speed |v| (frame- and
% alignment-invariant); the N/E/D velocity overlay is differentiated from the
% aligned VIO position (so it lands in NED without OpenVINS' JPL quaternion).
%
% Usage:
%   1) During the flight, record VIO in a terminal (ROS sourced):
%        ros2 bag record -o vio_run /ov_msckf/odomimu
%   2) After the run, with sim_log in the base workspace
%      (run_interactive estimator feed must be ON so est_pos/est_vel exist):
%        compare_vio_ekf('vio_run')
%   Optionally pass the log explicitly: compare_vio_ekf('vio_run', sim_log)
%
% Returns a struct with the aligned series and error metrics.
%
% Requires ROS Toolbox (ros2bagreader) and src/math on the path.

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'src', 'math'));   % umeyama_align

if nargin < 2 || isempty(simLog)
    simLog = evalin('base', 'sim_log');         % errors clearly if missing
end

% ---- EKF2 estimate (NED) -------------------------------------------------
t_e = simLog.t(:);
p_e = simLog.est_pos;          % Nx3 NED
v_e = simLog.est_vel;          % Nx3 NED
if isempty(p_e) || all(isnan(p_e(:)))
    error(['sim_log.est_pos is empty/all-NaN -- the run used the ' ...
           'ground-truth feed, not the estimator. Re-run run_interactive ' ...
           'with the estimator feed ON.']);
end
ok  = all(~isnan(p_e), 2) & ~isnan(t_e);
t_e = t_e(ok); p_e = p_e(ok, :); v_e = v_e(ok, :);

% ---- VIO from bag (OpenVINS global frame) --------------------------------
[t_v, p_v, spd_v] = read_odomimu(bagPath);

% ---- Common time window (shared sim clock) -------------------------------
t0 = max(t_e(1),   t_v(1));
t1 = min(t_e(end), t_v(end));
if t1 <= t0
    error(['No overlapping time window between sim_log (%.2f..%.2f s) and ' ...
           'VIO bag (%.2f..%.2f s). Were they recorded in the same run?'], ...
          t_e(1), t_e(end), t_v(1), t_v(end));
end
m = t_v >= t0 & t_v <= t1;
t_v = t_v(m); p_v = p_v(m, :); spd_v = spd_v(m);

% interpolate the EKF estimate onto the VIO timestamps
p_ei  = interp1(t_e, p_e, t_v, 'linear');
v_ei  = interp1(t_e, v_e, t_v, 'linear');
spd_e = vecnorm(v_ei, 2, 2);

% ---- Rigid alignment: VIO(global) -> EKF(NED) ----------------------------
[R, t, ate] = umeyama_align(p_v.', p_ei.');     % 3xN in, 3xN frame
p_va = (R * p_v.' + t).';                        % aligned VIO, NED (Nx3)
err  = vecnorm(p_va - p_ei, 2, 2);

% VIO NED velocity via derivative of the aligned position (light smoothing)
v_va = zeros(size(p_va));
for i = 1:3
    v_va(:, i) = gradient(p_va(:, i), t_v);
end
v_va = movmean(v_va, 7, 1);

% ---- Report --------------------------------------------------------------
fprintf('\n=== VIO vs EKF2 estimate ===\n');
fprintf('Overlap: %.2f .. %.2f s  (%d VIO samples)\n', t0, t1, numel(t_v));
fprintf('Position ATE (RMSE after SE3 align): %.3f m\n', ate);
fprintf('Position error  mean %.3f  max %.3f m\n', mean(err), max(err));
fprintf('Speed RMSE: %.3f m/s\n', sqrt(mean((spd_v - spd_e).^2)));
fprintf('Alignment t = [%.2f %.2f %.2f] m\n', t);

% ---- Plots ---------------------------------------------------------------
lbl = {'North', 'East', 'Down'};

figure('Name', 'VIO vs EKF2: position (NED)');
for i = 1:3
    subplot(3, 1, i); hold on; grid on;
    plot(t_v, p_ei(:, i), 'b',   'LineWidth', 1.2);
    plot(t_v, p_va(:, i), 'r--', 'LineWidth', 1.2);
    ylabel([lbl{i} ' [m]']);
    if i == 1, legend('EKF2 est', 'VIO (aligned)', 'Location', 'best'); end
end
xlabel('sim time [s]');

figure('Name', 'VIO vs EKF2: velocity (NED)');
for i = 1:3
    subplot(3, 1, i); hold on; grid on;
    plot(t_v, v_ei(:, i), 'b',   'LineWidth', 1.2);
    plot(t_v, v_va(:, i), 'r--', 'LineWidth', 1.2);
    ylabel(['v_' lower(lbl{i}(1)) ' [m/s]']);
    if i == 1
        legend('EKF2 est', 'VIO (d/dt aligned pos)', 'Location', 'best');
    end
end
xlabel('sim time [s]');

figure('Name', 'VIO vs EKF2: error + speed');
subplot(2, 1, 1); grid on;
plot(t_v, err, 'k', 'LineWidth', 1.2);
ylabel('pos error |p| [m]');
title(sprintf('Position ATE RMSE = %.3f m  (mean %.3f, max %.3f)', ...
              ate, mean(err), max(err)));
subplot(2, 1, 2); hold on; grid on;
plot(t_v, spd_e, 'b',   'LineWidth', 1.2);
plot(t_v, spd_v, 'r--', 'LineWidth', 1.2);     % VIO's own filtered speed
ylabel('speed [m/s]'); xlabel('sim time [s]');
legend('EKF2 |v|', 'VIO |v| (filter)', 'Location', 'best');

figure('Name', 'VIO vs EKF2: trajectory');
subplot(1, 2, 1); hold on; grid on; axis equal;
plot(p_ei(:, 2), p_ei(:, 1), 'b',   'LineWidth', 1.2);
plot(p_va(:, 2), p_va(:, 1), 'r--', 'LineWidth', 1.2);
xlabel('East [m]'); ylabel('North [m]'); title('Top-down');
legend('EKF2', 'VIO', 'Location', 'best');
subplot(1, 2, 2); hold on; grid on;
plot(t_v, -p_ei(:, 3), 'b',   'LineWidth', 1.2);
plot(t_v, -p_va(:, 3), 'r--', 'LineWidth', 1.2);
xlabel('sim time [s]'); ylabel('Up [m]'); title('Altitude');
legend('EKF2', 'VIO', 'Location', 'best');

% ---- Return --------------------------------------------------------------
out = struct('t', t_v, 'p_ekf', p_ei, 'p_vio', p_va, ...
             'v_ekf', v_ei, 'v_vio', v_va, ...
             'pos_err', err, 'ate_rmse', ate, ...
             'speed_ekf', spd_e, 'speed_vio', spd_v, ...
             'R', R, 't', t);
end


function [t, p, spd] = read_odomimu(bagPath)
%READ_ODOMIMU  Pull t (sim-time header stamp), position, and speed from a bag.
bag  = ros2bagreader(bagPath);
sel  = select(bag, 'Topic', '/ov_msckf/odomimu');
msgs = readMessages(sel);
if isempty(msgs)
    error('No /ov_msckf/odomimu messages found in bag "%s".', bagPath);
end
n = numel(msgs);
t = zeros(n, 1); p = zeros(n, 3); spd = zeros(n, 1);
for k = 1:n
    mk = msgs{k};
    t(k)   = double(mk.header.stamp.sec) + double(mk.header.stamp.nanosec) * 1e-9;
    p(k, :) = [mk.pose.pose.position.x, ...
               mk.pose.pose.position.y, ...
               mk.pose.pose.position.z];
    spd(k)  = norm([mk.twist.twist.linear.x, ...
                    mk.twist.twist.linear.y, ...
                    mk.twist.twist.linear.z]);
end
[t, idx] = sort(t);          % guard monotonic time for interp1
p = p(idx, :); spd = spd(idx);
end
