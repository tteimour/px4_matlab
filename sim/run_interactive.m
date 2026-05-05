function run_interactive()
% Interactive PX4 position-controller demo.
%
% A 3D view of the drone with X (North), Y (East), Altitude, and Yaw
% sliders. Drag any slider and the drone retargets and flies there in
% real time using the full PX4 cascade:
%
%   slider -> pos_sp -> PositionController -> AttitudeController
%             -> RateController -> ControlAllocator -> QuadrotorDynamics
%
% Frame conventions: NED internally (PX4 native). Altitude is exposed
% to the user as a positive height; the slider has a hard minimum of 0,
% so the setpoint never goes below ground (NED z = -altitude, capped
% at 0). The plant also has a soft ground floor at altitude = 0.
%
% The drone starts at the origin (NED [0;0;0], altitude = 0).

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(fullfile(root, 'src', 'params'));
addpath(fullfile(root, 'src', 'math'));
addpath(fullfile(root, 'src', 'plant'));
addpath(fullfile(root, 'src', 'controllers'));

p = px4_params();

range_xy  = 50;        % +/- m for North/East sliders
range_alt = 30;        % 0..m for altitude slider (minimum is 0)

% =====================================================================
% Figure / axes
% =====================================================================
fig = figure('Name', 'PX4 interactive position controller', ...
             'NumberTitle', 'off', 'Color', 'w', ...
             'Position', [120 80 1180 920]);

ax = axes('Parent', fig, 'Units', 'normalized', ...
          'Position', [0.06 0.46 0.66 0.50]);
hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on'); axis(ax, 'equal');
view_half = 8;                                    % initial half-width (m) of the camera box
xlim(ax, [-view_half view_half]);
ylim(ax, [-view_half view_half]);
zlim(ax, [0 view_half * 1.2]);
xlabel(ax, 'East (m)'); ylabel(ax, 'North (m)'); zlabel(ax, 'Altitude (m)');
title(ax, 'PX4 cascaded controller — drag the sliders to retarget');
view(ax, 35, 25);

% Ground plane (visual only) — large enough to span any slider position
gp = range_xy;
ground = patch(ax, [-gp gp gp -gp], [-gp -gp gp gp], [0 0 0 0], ...
      [0.93 0.95 0.93], 'EdgeColor', [0.7 0.75 0.7], 'FaceAlpha', 0.6); %#ok<NASGU>

trail   = animatedline(ax, 'Color', [0.2 0.4 0.9], 'LineWidth', 1.0, ...
                       'MaximumNumPoints', 6000);
sp_dot  = plot3(ax, 0, 0, 0, 'rs', 'MarkerSize', 12, ...
                'MarkerFaceColor', [1 0.7 0.7]);
sp_line = plot3(ax, [0 0], [0 0], [0 0], 'r:', 'LineWidth', 0.8);

drone = makeDrone(ax);

% =====================================================================
% Real-time tracking plots (position + attitude, setpoint vs actual)
% =====================================================================
trk_window = 20;                                  % rolling window length (s)
max_pts    = 4000;                                % cap on points per line

ax_pos = axes('Parent', fig, 'Units', 'normalized', ...
              'Position', [0.06 0.07 0.40 0.32]);
hold(ax_pos, 'on'); grid(ax_pos, 'on'); box(ax_pos, 'on');
title(ax_pos, 'Position tracking (solid = actual, dashed = setpoint)');
xlabel(ax_pos, 'time (s)'); ylabel(ax_pos, 'm');
xlim(ax_pos, [0 trk_window]);

c_n = [0.85 0.10 0.10];
c_e = [0.10 0.55 0.20];
c_a = [0.10 0.30 0.85];
trk.pn_act = animatedline(ax_pos, 'Color', c_n, 'LineWidth', 1.4, 'MaximumNumPoints', max_pts);
trk.pe_act = animatedline(ax_pos, 'Color', c_e, 'LineWidth', 1.4, 'MaximumNumPoints', max_pts);
trk.pa_act = animatedline(ax_pos, 'Color', c_a, 'LineWidth', 1.4, 'MaximumNumPoints', max_pts);
trk.pn_sp  = animatedline(ax_pos, 'Color', c_n, 'LineStyle', '--', 'LineWidth', 1.0, 'MaximumNumPoints', max_pts);
trk.pe_sp  = animatedline(ax_pos, 'Color', c_e, 'LineStyle', '--', 'LineWidth', 1.0, 'MaximumNumPoints', max_pts);
trk.pa_sp  = animatedline(ax_pos, 'Color', c_a, 'LineStyle', '--', 'LineWidth', 1.0, 'MaximumNumPoints', max_pts);
legend(ax_pos, {'N', 'E', 'Alt'}, 'Location', 'northwest', 'Orientation', 'horizontal');

ax_att = axes('Parent', fig, 'Units', 'normalized', ...
              'Position', [0.55 0.07 0.40 0.32]);
hold(ax_att, 'on'); grid(ax_att, 'on'); box(ax_att, 'on');
title(ax_att, 'Attitude tracking (solid = actual, dashed = setpoint)');
xlabel(ax_att, 'time (s)'); ylabel(ax_att, 'deg');
xlim(ax_att, [0 trk_window]);

c_r = [0.85 0.10 0.10];
c_p = [0.10 0.55 0.20];
c_y = [0.10 0.30 0.85];
trk.r_act = animatedline(ax_att, 'Color', c_r, 'LineWidth', 1.4, 'MaximumNumPoints', max_pts);
trk.p_act = animatedline(ax_att, 'Color', c_p, 'LineWidth', 1.4, 'MaximumNumPoints', max_pts);
trk.y_act = animatedline(ax_att, 'Color', c_y, 'LineWidth', 1.4, 'MaximumNumPoints', max_pts);
trk.r_sp  = animatedline(ax_att, 'Color', c_r, 'LineStyle', '--', 'LineWidth', 1.0, 'MaximumNumPoints', max_pts);
trk.p_sp  = animatedline(ax_att, 'Color', c_p, 'LineStyle', '--', 'LineWidth', 1.0, 'MaximumNumPoints', max_pts);
trk.y_sp  = animatedline(ax_att, 'Color', c_y, 'LineStyle', '--', 'LineWidth', 1.0, 'MaximumNumPoints', max_pts);
legend(ax_att, {'roll', 'pitch', 'yaw'}, 'Location', 'northwest', 'Orientation', 'horizontal');

% =====================================================================
% Slider panel
% =====================================================================
panel = uipanel(fig, 'Units', 'normalized', 'Position', [0.74 0.46 0.24 0.50], ...
                'BackgroundColor', 'w', 'Title', 'Setpoint', ...
                'FontWeight', 'bold');

[sl_x,   lbl_x]   = mkSlider(panel, [0.06 0.84 0.88 0.10], 'X (North)',  -range_xy, range_xy, 0);
[sl_y,   lbl_y]   = mkSlider(panel, [0.06 0.68 0.88 0.10], 'Y (East)',   -range_xy, range_xy, 0);
[sl_z,   lbl_z]   = mkSlider(panel, [0.06 0.52 0.88 0.10], 'Altitude',    0,         range_alt, 0);
[sl_yaw, lbl_yaw] = mkSlider(panel, [0.06 0.36 0.88 0.10], 'Yaw (deg)', -180,        180,       0);

state_lbl = uicontrol(panel, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.06 0.13 0.88 0.18], 'BackgroundColor', 'w', ...
    'HorizontalAlignment', 'left', 'FontName', 'Courier New', ...
    'FontSize', 9, 'String', '');

uicontrol(panel, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.06 0.03 0.42 0.08], 'String', 'Reset', ...
    'Callback', @(~,~) setappdata(fig, 'reset_request', true));
uicontrol(panel, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.52 0.03 0.42 0.08], 'String', 'Stop', ...
    'FontWeight', 'bold', ...
    'Callback', @(~,~) setappdata(fig, 'running', false));

% =====================================================================
% Sim modules
% =====================================================================
plant    = QuadrotorDynamics(p);
plant.reset([0;0;0], 0);
pos_ctl  = PositionController(p);
att_ctl  = AttitudeController(p);
rate_ctl = RateController(p);
alloc    = ControlAllocator(p);

dt_rate = 1 / p.rate_hz.rate;
n_att   = round(p.rate_hz.rate / p.rate_hz.attitude);
n_pos   = round(p.rate_hz.rate / p.rate_hz.position);
dt_pos  = n_pos * dt_rate;

fps = 100;
dt_frame = 1 / fps;

q_sp          = [1; 0; 0; 0];
thrust_body_z = -p.pos.thr_hover;
yawspeed_sp   = NaN;
rate_sp       = [0; 0; 0];

setappdata(fig, 'running', true);
setappdata(fig, 'reset_request', false);

k = 0;
t_sim = 0;
while ishandle(fig) && getappdata(fig, 'running')
    frame_t0 = tic;

    if getappdata(fig, 'reset_request')
        plant.reset([0;0;0], 0);
        pos_ctl.reset();
        rate_ctl.reset();
        clearpoints(trail);
        clearpoints(trk.pn_act); clearpoints(trk.pe_act); clearpoints(trk.pa_act);
        clearpoints(trk.pn_sp);  clearpoints(trk.pe_sp);  clearpoints(trk.pa_sp);
        clearpoints(trk.r_act);  clearpoints(trk.p_act);  clearpoints(trk.y_act);
        clearpoints(trk.r_sp);   clearpoints(trk.p_sp);   clearpoints(trk.y_sp);
        t_sim = 0;
        setappdata(fig, 'reset_request', false);
    end

    % --- Read sliders (poll each frame) ---
    alt_cmd = max(0, get(sl_z, 'Value'));        % hard-clamp altitude floor
    pos_sp = [ get(sl_x, 'Value');
               get(sl_y, 'Value');
              -alt_cmd ];                        % NED z = -altitude
    yaw_sp = deg2rad(get(sl_yaw, 'Value'));

    set(lbl_x,   'String', sprintf('X (North)  = %+6.1f m',  pos_sp(1)));
    set(lbl_y,   'String', sprintf('Y (East)   = %+6.1f m',  pos_sp(2)));
    set(lbl_z,   'String', sprintf('Altitude   = %5.1f m',   alt_cmd));
    set(lbl_yaw, 'String', sprintf('Yaw        = %+5.0f deg', rad2deg(yaw_sp)));

    % --- Advance physics by dt_frame seconds in dt_rate substeps ---
    n_steps = max(1, round(dt_frame / dt_rate));
    for i = 1:n_steps
        s = plant.state();

        if mod(k, n_pos) == 0
            [q_sp, thrust_body_z, ~, ~, ~] = pos_ctl.update( ...
                s.position_ned, s.velocity_ned, s.acceleration_ned, ...
                pos_sp, yaw_sp, [], [], dt_pos);
            yawspeed_sp = NaN;
        end

        if mod(k, n_att) == 0
            rate_sp = att_ctl.update(s.attitude_q, q_sp, yawspeed_sp);
        end

        % "Landed" only when both the drone AND the setpoint are at the
        % ground — keeps the rate integrator alive during low hovers.
        landed = (s.position_ned(3) > -0.05) && ...
                 (norm(s.velocity_ned) < 0.3) && ...
                 (pos_sp(3) > -0.05);
        torque = rate_ctl.update(s.angular_vel_b, rate_sp, [0;0;0], dt_rate, landed);
        T_mag  = max(0, -thrust_body_z);
        [m, sat_pos, sat_neg] = alloc.allocate(torque, T_mag);
        rate_ctl.setSaturationStatus(sat_pos, sat_neg);

        plant.step(m, dt_rate);

        % Soft ground floor: altitude cannot go below 0 (NED z <= 0).
        if plant.pos_ned(3) > 0
            plant.pos_ned(3) = 0;
            if plant.vel_ned(3) > 0
                plant.vel_ned(3) = 0;
            end
        end

        k = k + 1;
    end

    % --- Render ---
    s = plant.state();
    eN = s.position_ned(1);
    eE = s.position_ned(2);
    eU = max(0, -s.position_ned(3));
    addpoints(trail, eE, eN, eU);

    snU = max(0, -pos_sp(3));
    set(sp_dot,  'XData', pos_sp(2), 'YData', pos_sp(1), 'ZData', snU);
    set(sp_line, 'XData', [eE pos_sp(2)], ...
                 'YData', [eN pos_sp(1)], ...
                 'ZData', [eU snU]);

    drone = updateDrone(drone, s.position_ned, s.attitude_q);

    % Camera follow: keep a tight box around drone + setpoint.
    cx = 0.5 * (eE + pos_sp(2));
    cy = 0.5 * (eN + pos_sp(1));
    half = max([8, abs(eE - pos_sp(2)), abs(eN - pos_sp(1))]) + 4;
    xlim(ax, [cx - half, cx + half]);
    ylim(ax, [cy - half, cy + half]);
    zlim(ax, [0, max(8, max(eU, snU) + 4)]);

    rpy    = quat_to_euler(s.attitude_q);
    rpy_sp = quat_to_euler(q_sp);

    t_sim = t_sim + dt_frame;
    addpoints(trk.pn_act, t_sim, eN);
    addpoints(trk.pe_act, t_sim, eE);
    addpoints(trk.pa_act, t_sim, eU);
    addpoints(trk.pn_sp,  t_sim, pos_sp(1));
    addpoints(trk.pe_sp,  t_sim, pos_sp(2));
    addpoints(trk.pa_sp,  t_sim, snU);
    addpoints(trk.r_act,  t_sim, rad2deg(rpy(1)));
    addpoints(trk.p_act,  t_sim, rad2deg(rpy(2)));
    addpoints(trk.y_act,  t_sim, rad2deg(rpy(3)));
    addpoints(trk.r_sp,   t_sim, rad2deg(rpy_sp(1)));
    addpoints(trk.p_sp,   t_sim, rad2deg(rpy_sp(2)));
    addpoints(trk.y_sp,   t_sim, rad2deg(yaw_sp));

    if t_sim > trk_window
        xlim(ax_pos, [t_sim - trk_window, t_sim]);
        xlim(ax_att, [t_sim - trk_window, t_sim]);
    end

    set(state_lbl, 'String', sprintf( ...
        ['pos    N=%+6.2f m\n' ...
         '       E=%+6.2f m\n' ...
         '       Alt=%5.2f m\n' ...
         'speed  %5.2f m/s\n' ...
         'yaw    %+6.1f deg'], ...
        eN, eE, eU, norm(s.velocity_ned), rad2deg(rpy(3))));

    drawnow limitrate;

    elapsed = toc(frame_t0);
    if elapsed < dt_frame
        pause(dt_frame - elapsed);
    end
end

if ishandle(fig)
    delete(fig);
end
end


% =========================================================================
% UI helpers
% =========================================================================
function [sl, lbl] = mkSlider(parent, pos, name, vmin, vmax, v0)
% pos = [x y w h] in normalized coords. Top half is label, bottom is slider.
x = pos(1); y = pos(2); w = pos(3); h = pos(4);
lbl_h = 0.5 * h;
sl_h  = 0.5 * h;
lbl = uicontrol(parent, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [x, y + sl_h, w, lbl_h], 'BackgroundColor', 'w', ...
    'HorizontalAlignment', 'left', 'FontName', 'Courier New', ...
    'FontSize', 9, 'String', sprintf('%s = %.1f', name, v0));
sl = uicontrol(parent, 'Style', 'slider', 'Units', 'normalized', ...
    'Position', [x, y, w, sl_h], ...
    'Min', vmin, 'Max', vmax, 'Value', v0, ...
    'SliderStep', [0.005 0.05]);
end


% =========================================================================
% Drone visual: four arms in a quad-X plus a red forward indicator.
% Arms drawn from CG to each rotor in body frame (FRD), then rotated
% into NED via the current attitude quaternion.
% =========================================================================
function drone = makeDrone(ax)
% Four arms, drawn as a single line strip with NaN breaks.
arm = 1.2;                                        % visual arm length (m)
drone.arms_body = [ ...
     0   arm   NaN  -arm   NaN   arm   NaN  -arm;   % FRD x  (Forward)
     0   arm   NaN  -arm   NaN  -arm   NaN   arm;   % FRD y  (Right)
     0     0   NaN     0   NaN     0   NaN     0];  % FRD z  (Down)
drone.rotors_body = [ ...
     arm  -arm   arm  -arm;
     arm  -arm  -arm   arm;
       0     0     0     0];
drone.front_body = [0 1.2*arm; 0 0; 0 0];

drone.arms = plot3(ax, nan(1, 8), nan(1, 8), nan(1, 8), ...
                   'k-', 'LineWidth', 3);
drone.rotors = plot3(ax, nan(1, 4), nan(1, 4), nan(1, 4), ...
                     'ko', 'MarkerSize', 14, 'MarkerFaceColor', [0.2 0.2 0.2]);
drone.front = plot3(ax, nan(1, 2), nan(1, 2), nan(1, 2), ...
                    'r-', 'LineWidth', 3.5);
end


function drone = updateDrone(drone, pos_ned, q)
R = quat_to_dcm(q);                               % body -> NED

arms_ned   = R * drone.arms_body   + pos_ned;
rotors_ned = R * drone.rotors_body + pos_ned;
front_ned  = R * drone.front_body  + pos_ned;

set(drone.arms,   'XData', arms_ned(2, :), ...
                  'YData', arms_ned(1, :), ...
                  'ZData', max(0, -arms_ned(3, :)));
set(drone.rotors, 'XData', rotors_ned(2, :), ...
                  'YData', rotors_ned(1, :), ...
                  'ZData', max(0, -rotors_ned(3, :)));
set(drone.front,  'XData', front_ned(2, :), ...
                  'YData', front_ned(1, :), ...
                  'ZData', max(0, -front_ned(3, :)));
end
