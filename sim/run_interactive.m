function run_interactive()
% Interactive PX4 controller demo — VELOCITY-LOOP variant.
%
% The position-control outer loop is bypassed (gain_pos_p(1:2) = 0), so the
% sliders command velocity directly:
%
%   [VN, VE, VD] -> vel_sp_ff -> PositionController (vel loop only)
%                -> AttitudeController -> RateController
%                -> ControlAllocator -> QuadrotorDynamics
%
% Vertical altitude-hold is preserved: the VD slider is integrated into
% an `alt_target` (m above ground) which the controller's z position-P
% term tracks. The altitude measurement fed to the controller has Gaussian
% noise added to mimic a barometer-aided altitude estimate.
%
% Frame conventions: NED internally. VD positive = down (descend).
% Yaw slider drives an absolute yaw setpoint. Soft ground floor at z = 0.

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(fullfile(root, 'src', 'params'));
addpath(fullfile(root, 'src', 'math'));
addpath(fullfile(root, 'src', 'plant'));
addpath(fullfile(root, 'src', 'controllers'));

p = px4_params();

range_vh   = 5;        % +/- m/s for VN / VE sliders
range_vd   = 3;        % +/- m/s for VD slider (positive = descend)
range_xy   = 50;       % ground-plane half-width (m), purely visual
baro_sigma = 0.2;      % simulated barometer noise std (m), injected on
                       % the altitude measurement fed to the controller

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

drone = makeDrone(ax, root);

% =====================================================================
% Real-time tracking plots (position + attitude, setpoint vs actual)
% =====================================================================
trk_window = 20;                                  % rolling window length (s)
max_pts    = 4000;                                % cap on points per line

ax_vel = axes('Parent', fig, 'Units', 'normalized', ...
              'Position', [0.06 0.07 0.40 0.32]);
hold(ax_vel, 'on'); grid(ax_vel, 'on'); box(ax_vel, 'on');
title(ax_vel, 'Velocity tracking (solid = actual, dashed = setpoint)');
xlabel(ax_vel, 'time (s)'); ylabel(ax_vel, 'm/s');
xlim(ax_vel, [0 trk_window]);

c_n = [0.85 0.10 0.10];
c_e = [0.10 0.55 0.20];
c_d = [0.10 0.30 0.85];
trk.vn_act = animatedline(ax_vel, 'Color', c_n, 'LineWidth', 1.4, 'MaximumNumPoints', max_pts);
trk.ve_act = animatedline(ax_vel, 'Color', c_e, 'LineWidth', 1.4, 'MaximumNumPoints', max_pts);
trk.vd_act = animatedline(ax_vel, 'Color', c_d, 'LineWidth', 1.4, 'MaximumNumPoints', max_pts);
trk.vn_sp  = animatedline(ax_vel, 'Color', c_n, 'LineStyle', '--', 'LineWidth', 1.0, 'MaximumNumPoints', max_pts);
trk.ve_sp  = animatedline(ax_vel, 'Color', c_e, 'LineStyle', '--', 'LineWidth', 1.0, 'MaximumNumPoints', max_pts);
trk.vd_sp  = animatedline(ax_vel, 'Color', c_d, 'LineStyle', '--', 'LineWidth', 1.0, 'MaximumNumPoints', max_pts);
legend(ax_vel, {'VN', 'VE', 'VD'}, 'Location', 'northwest', 'Orientation', 'horizontal');

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

[sl_vn,  lbl_vn]  = mkSlider(panel, [0.06 0.84 0.88 0.10], 'VN (m/s)',  -range_vh, range_vh, 0);
[sl_ve,  lbl_ve]  = mkSlider(panel, [0.06 0.68 0.88 0.10], 'VE (m/s)',  -range_vh, range_vh, 0);
[sl_vd,  lbl_vd]  = mkSlider(panel, [0.06 0.52 0.88 0.10], 'VD (m/s)',  -range_vd, range_vd, 0);
[sl_yaw, lbl_yaw] = mkSlider(panel, [0.06 0.36 0.88 0.10], 'Yaw (deg)', -180,       180,      0);

% Force live-drag updates: by default uicontrol slider only refreshes its
% Value on release. Attaching a ContinuousValueChange listener makes Value
% update on every drag tick — the polling loop then sees fresh commands.
addlistener(sl_vn,  'ContinuousValueChange', @(~,~) []);
addlistener(sl_ve,  'ContinuousValueChange', @(~,~) []);
addlistener(sl_vd,  'ContinuousValueChange', @(~,~) []);
addlistener(sl_yaw, 'ContinuousValueChange', @(~,~) []);

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
% Outer loop is velocity, not position: zero the horizontal position-P gain
% so the controller becomes a pass-through of the slider velocity command on
% XY. Z is left alone — altitude-hold remains active.
pos_ctl.gain_pos_p(1:2) = 0;
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
m_last        = [0; 0; 0; 0];           % most recent motor commands (for prop spin)

setappdata(fig, 'running', true);
setappdata(fig, 'reset_request', false);

k = 0;
t_sim = 0;
alt_target  = 0;             % integrated from VD slider — altitude-hold target (m)
vel_sp_used = [0; 0; 0];     % most recent vel setpoint as seen by the velocity loop
while ishandle(fig) && getappdata(fig, 'running')
    frame_t0 = tic;

    if getappdata(fig, 'reset_request')
        plant.reset([0;0;0], 0);
        pos_ctl.reset();
        rate_ctl.reset();
        clearpoints(trail);
        clearpoints(trk.vn_act); clearpoints(trk.ve_act); clearpoints(trk.vd_act);
        clearpoints(trk.vn_sp);  clearpoints(trk.ve_sp);  clearpoints(trk.vd_sp);
        clearpoints(trk.r_act);  clearpoints(trk.p_act);  clearpoints(trk.y_act);
        clearpoints(trk.r_sp);   clearpoints(trk.p_sp);   clearpoints(trk.y_sp);
        t_sim       = 0;
        alt_target  = 0;
        vel_sp_used = [0; 0; 0];
        setappdata(fig, 'reset_request', false);
    end

    % --- Read sliders (poll each frame) ---
    vN_cmd = get(sl_vn, 'Value');
    vE_cmd = get(sl_ve, 'Value');
    vD_cmd = get(sl_vd, 'Value');
    yaw_sp = deg2rad(get(sl_yaw, 'Value'));

    % Integrate altitude-hold target from VD command. Negative VD = climb.
    alt_target = max(0, alt_target - vD_cmd * dt_frame);

    vel_sp_cmd = [vN_cmd; vE_cmd; vD_cmd];

    set(lbl_vn,  'String', sprintf('VN     = %+5.2f m/s',  vN_cmd));
    set(lbl_ve,  'String', sprintf('VE     = %+5.2f m/s',  vE_cmd));
    set(lbl_vd,  'String', sprintf('VD     = %+5.2f m/s',  vD_cmd));
    set(lbl_yaw, 'String', sprintf('Yaw    = %+5.0f deg',  rad2deg(yaw_sp)));

    % --- Advance physics by dt_frame seconds in dt_rate substeps ---
    n_steps = max(1, round(dt_frame / dt_rate));
    for i = 1:n_steps
        s = plant.state();

        if mod(k, n_pos) == 0
            % Build the position vector seen by the controller: true XY (error
            % is irrelevant since gain_pos_p(1:2)=0), and a noisy altitude
            % (NED z) to mimic a barometer-aided altitude estimate.
            pos_meas = s.position_ned;
            pos_meas(3) = pos_meas(3) + baro_sigma * randn();

            % Position setpoint: only Z is meaningful (altitude-hold target).
            % XY is set to current position so any residual position error is
            % zero — defensive in case gain_pos_p(1:2) is later non-zero.
            pos_sp_inner = [s.position_ned(1); s.position_ned(2); -alt_target];

            [q_sp, thrust_body_z, vel_sp_used, ~, ~] = pos_ctl.update( ...
                pos_meas, s.velocity_ned, s.acceleration_ned, ...
                pos_sp_inner, yaw_sp, vel_sp_cmd, [], dt_pos);
            yawspeed_sp = NaN;
        end

        if mod(k, n_att) == 0
            rate_sp = att_ctl.update(s.attitude_q, q_sp, yawspeed_sp);
        end

        % "Landed" only when both the drone AND the altitude target are at
        % the ground — keeps the rate integrator alive during low hovers.
        landed = (s.position_ned(3) > -0.05) && ...
                 (norm(s.velocity_ned) < 0.3) && ...
                 (alt_target < 0.05);
        torque = rate_ctl.update(s.angular_vel_b, rate_sp, [0;0;0], dt_rate, landed);
        T_mag  = max(0, -thrust_body_z);
        [m, sat_pos, sat_neg] = alloc.allocate(torque, T_mag);
        rate_ctl.setSaturationStatus(sat_pos, sat_neg);
        m_last = m;

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

    % Mark the altitude-hold target as a column at the drone's current xy.
    set(sp_dot,  'XData', eE, 'YData', eN, 'ZData', alt_target);
    set(sp_line, 'XData', [eE eE], ...
                 'YData', [eN eN], ...
                 'ZData', [eU alt_target]);

    drone = updateDrone(drone, s.position_ned, s.attitude_q, m_last, dt_frame);

    % Camera follow: stay centred on the drone (no horizontal setpoint
    % to bracket — the user commands velocity, not position).
    half_xy = 8;
    xlim(ax, [eE - half_xy, eE + half_xy]);
    ylim(ax, [eN - half_xy, eN + half_xy]);
    zlim(ax, [0, max(8, max(eU, alt_target) + 4)]);

    rpy    = quat_to_euler(s.attitude_q);
    rpy_sp = quat_to_euler(q_sp);

    t_sim = t_sim + dt_frame;
    addpoints(trk.vn_act, t_sim, s.velocity_ned(1));
    addpoints(trk.ve_act, t_sim, s.velocity_ned(2));
    addpoints(trk.vd_act, t_sim, s.velocity_ned(3));
    addpoints(trk.vn_sp,  t_sim, vel_sp_used(1));
    addpoints(trk.ve_sp,  t_sim, vel_sp_used(2));
    addpoints(trk.vd_sp,  t_sim, vel_sp_used(3));
    addpoints(trk.r_act,  t_sim, rad2deg(rpy(1)));
    addpoints(trk.p_act,  t_sim, rad2deg(rpy(2)));
    addpoints(trk.y_act,  t_sim, rad2deg(rpy(3)));
    addpoints(trk.r_sp,   t_sim, rad2deg(rpy_sp(1)));
    addpoints(trk.p_sp,   t_sim, rad2deg(rpy_sp(2)));
    addpoints(trk.y_sp,   t_sim, rad2deg(yaw_sp));

    if t_sim > trk_window
        xlim(ax_vel, [t_sim - trk_window, t_sim]);
        xlim(ax_att, [t_sim - trk_window, t_sim]);
    end

    set(state_lbl, 'String', sprintf( ...
        ['pos    N=%+6.2f m\n' ...
         '       E=%+6.2f m\n' ...
         '       Alt=%5.2f m\n' ...
         'tgt    Alt=%5.2f m\n' ...
         'speed  %5.2f m/s\n' ...
         'yaw    %+6.1f deg'], ...
        eN, eE, eU, alt_target, norm(s.velocity_ned), rad2deg(rpy(3))));

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
% Drone visual: STL mesh body + four spinning propellers.
%
% Rendering is built on `hgtransform`: each piece is parented to a
% transform whose 4x4 Matrix is updated per frame, instead of rewriting
% vertex arrays. That's an order-of-magnitude faster than per-vertex
% updates on a 15k-tri body mesh.
%
% Rotor mount positions are auto-detected from the body STL by clustering
% the outermost points by quadrant — so propellers sit on the body's
% actual motor mounts rather than at hard-coded corners.
%
% Frame mapping STL -> FRD: (X, Y, Z) -> (X, Z, Y). I.e. STL Y axis is
% assumed to be FRD Z (down). Edit `swapToFRD` if the model looks wrong.
% =========================================================================
function drone = makeDrone(ax, root)
arm = 1.2;                                          % nominal half-span (m)

% --- Body STL ---
[bV, bF] = readStlAny(fullfile(root, 'QuadCopter_Body.stl'));
bV = bV - mean(bV, 1);

% Detect rotor mount centroids by quadrant of the four outermost clusters.
horiz_r = sqrt(bV(:, 1).^2 + bV(:, 3).^2);
mask_outer = horiz_r > quantile(horiz_r, 0.85);
quads = [+1 +1; -1 +1; +1 -1; -1 -1];               % STL X-sign, STL Z-sign
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

% Apply STL -> FRD swap to body verts AND rotor mounts.
bV          = swapToFRD(bV);
mounts_frd  = swapToFRD(mounts_raw);

% Auto-scale so body's max horizontal (FRD X/Y) extent equals arm.
horiz_extent = max(max(abs(bV(:, 1:2)), [], 1));
scale = arm / max(horiz_extent, eps);
bV          = bV * scale;
mounts_frd  = mounts_frd * scale;

% Decimate body — full mesh is ~457k tris.
if size(bF, 1) > 12000
    fv = reducepatch(struct('faces', bF, 'vertices', bV), 12000);
    bV = fv.vertices; bF = fv.faces;
end

% --- Body under its own hgtransform (handles attitude + position) ---
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
pV = pV * (arm * 0.45 / max(prop_horiz, eps));      % diameter ~ 0.9*arm

% Each prop xform is a child of the body xform — body handles attitude /
% position, prop xform handles rotor offset + spin around its own z.
drone.prop_xform   = gobjects(1, 4);
for i = 1:4
    drone.prop_xform(i) = hgtransform('Parent', drone.body_xform);
    patch('Parent', drone.prop_xform(i), 'Faces', pF, 'Vertices', pV, ...
          'FaceColor', [0.10 0.10 0.12], 'EdgeColor', 'none', ...
          'FaceLighting', 'gouraud', 'AmbientStrength', 0.5);
end
drone.rotors_body  = mounts_frd';                   % 3x4
drone.prop_angle   = zeros(1, 4);
drone.prop_spin_dir = [+1 -1 +1 -1];                % alternating CW / CCW
drone.prop_spin_max = 250;                          % rad/s at full thrust

% Forward indicator (red bar from CG out the nose, body-frame +x).
drone.front = line('Parent', drone.body_xform, ...
                   'XData', [0, 1.4*arm], 'YData', [0, 0], 'ZData', [0, 0], ...
                   'Color', 'r', 'LineWidth', 3.5);

if isempty(findobj(ax, 'Type', 'light'))
    camlight(ax, 'headlight');
    lighting(ax, 'gouraud');
end
end


function drone = updateDrone(drone, pos_ned, q, motor_cmd, dt)
% Build the 4x4 transform that takes body-frame coords (FRD) to plot-frame
% coords (East, North, Altitude).
%   v_plot = ned_to_plot * (R_b2n * v_body + pos_ned)
% where ned_to_plot is the constant matrix [E;N;Up] = [y;x;-z]_ned.
ned_to_plot = [0 1 0; 1 0 0; 0 0 -1];
R_b2n = quat_to_dcm(q);
M_body = eye(4);
M_body(1:3, 1:3) = ned_to_plot * R_b2n;
M_body(1:3, 4)   = ned_to_plot * pos_ned;
set(drone.body_xform, 'Matrix', M_body);

% Propeller xforms are CHILDREN of the body xform, so each only needs to
% express prop-local -> body-frame: spin around its own z, then translate
% to the rotor mount position (in body frame).
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
% STL frame -> FRD body frame. Empirically for these meshes, STL Y
% behaves like FRD Z (down), so we map (X, Y, Z) -> (X, Z, Y).
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
