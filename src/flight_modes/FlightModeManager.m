classdef FlightModeManager < handle
% Top-level flight-mode dispatcher. Each tick consumes the current vehicle
% state plus stick input, runs one of eight PX4-style modes, and returns a
% command struct that the rest of the controller cascade consumes.
%
% PX4 references (paths under /home/teymur/git/SynapLine/synap-px4):
%   Stabilized:  src/modules/flight_mode_manager/tasks/ManualAcceleration/FlightTaskManualAcceleration.cpp
%   Altitude:    src/modules/flight_mode_manager/tasks/ManualAltitude/FlightTaskManualAltitude.cpp
%   Position:    src/modules/flight_mode_manager/tasks/ManualPosition/FlightTaskManualPosition.cpp
%   Mission:     src/modules/navigator/mission.cpp
%   RTL:         src/modules/navigator/rtl_direct.cpp
%   Hold:        src/modules/navigator/loiter.cpp
%   Land:        src/modules/navigator/land.cpp + FlightTaskAuto::_prepareLandSetpoints (lines 226-301)
%   Takeoff:     src/modules/navigator/takeoff.cpp
%
% Output `cmd` struct:
%   cmd.kind          : 'attitude'   q_sp + thrust go straight to RateController
%                                    (used by Stabilized, Altitude — they bypass
%                                    the position controller entirely)
%                       'position'   pos_sp/vel_sp_ff/yaw_sp drive PositionController
%                                    as in the existing run_mission.m chain
%   cmd.q_sp          (4x1)          attitude target (only when kind='attitude')
%   cmd.thrust_body_z (scalar, neg)  collective thrust   (only when kind='attitude')
%   cmd.pos_sp        (3x1 NED)      (only when kind='position')
%   cmd.vel_sp_ff     (3x1)          velocity FF, NaN entries = no FF
%   cmd.acc_sp_ff     (3x1)          acceleration FF (always [], unused for now)
%   cmd.yaw_sp        (scalar)
%   cmd.yawspeed_sp   (scalar)       NaN if no yaw-rate FF commanded
%   cmd.mode          (string)       echoes the mode (post-auto-transitions)

    properties
        p                   % px4_params struct
        mode = 'stabilized' % current flight mode (string)

        % Lock state (manual modes)
        pos_lock            % 3x1 NED: held position when sticks centred
        xy_locked           % logical: XY lock engaged
        z_locked            % logical: Z lock engaged
        yaw_lock            % scalar: held yaw heading (rad)
        alt_target          % NED z target for Altitude mode integration

        % Z-PID integrator for Altitude mode (independent of PositionController)
        alt_int             % m/s^2

        % Auto-mode state
        home_pos            % 3x1 NED (origin pinned at activation time)
        nav                 % Navigator instance for Mission
        rtl_phase           % 'climb' / 'cruise' / 'land'
        land_target_z       % NED z that we're descending toward (Land/RTL phase 'land')
        takeoff_complete    % logical (Takeoff mode finishes by switching to Hold)
    end

    methods
        function obj = FlightModeManager(p)
            obj.p             = p;
            obj.pos_lock      = [0; 0; 0];
            obj.xy_locked     = false;
            obj.z_locked      = false;
            obj.yaw_lock      = 0;
            obj.alt_target    = 0;
            obj.alt_int       = 0;
            obj.home_pos      = [0; 0; 0];
            obj.nav           = [];
            obj.rtl_phase     = 'climb';
            obj.land_target_z = 0;
            obj.takeoff_complete = false;
        end

        function setMode(obj, mode, state)
        % Switch mode. Re-latches locks against the current state so that
        % engaging Position/Altitude/Hold/Land/Takeoff doesn't snap.
            obj.mode      = mode;
            obj.pos_lock  = state.position_ned;
            obj.alt_target = state.position_ned(3);
            obj.alt_int   = 0;
            obj.xy_locked = false;
            obj.z_locked  = false;
            yaw_now       = quat_yaw(state.attitude_q);
            obj.yaw_lock  = yaw_now;

            switch mode
                case 'rtl'
                    obj.rtl_phase = 'climb';
                case 'land'
                    obj.land_target_z = state.position_ned(3);
                case 'takeoff'
                    obj.takeoff_complete = false;
                case 'mission'
                    if ~isempty(obj.nav)
                        obj.nav.idx = 1;
                        obj.nav.initialized = false;
                    end
            end
        end

        function setHome(obj, pos_ned)
            obj.home_pos = pos_ned(:);
        end

        function setMission(obj, waypoints_ne_d)
        % waypoints_ne_d: Nx3 [N E D] in NED. Yaw and per-WP acceptance
        % default to current yaw and NAV_ACC_RAD. Pass an empty matrix
        % to clear the mission.
            if isempty(waypoints_ne_d)
                obj.nav = [];
                return;
            end
            n = size(waypoints_ne_d, 1);
            wp5 = [waypoints_ne_d, ...
                   ones(n, 1) * obj.yaw_lock, ...
                   ones(n, 1) * obj.p.nav.acc_rad];
            obj.nav = Navigator(obj.p, wp5);
        end

        function clearMission(obj)
            obj.nav = [];
        end

        function cmd = update(obj, state, sticks, dt)
        % sticks struct: .left_x .left_y .right_x .right_y in [-1, 1]
        %   left_x  = yaw stick   (right positive)
        %   left_y  = throttle    (up positive)
        %   right_x = roll        (right positive)
        %   right_y = pitch       (up positive  => stick forward => nose down)
            switch obj.mode
                case 'stabilized', cmd = obj.runStabilized(state, sticks, dt);
                case 'altitude',   cmd = obj.runAltitude(state, sticks, dt);
                case 'position',   cmd = obj.runPosition(state, sticks, dt);
                case 'hold',       cmd = obj.runHold(state);
                case 'mission',    cmd = obj.runMission(state, dt);
                case 'rtl',        cmd = obj.runRTL(state);
                case 'land',       cmd = obj.runLand(state, dt);
                case 'takeoff',    cmd = obj.runTakeoff(state);
                otherwise,         error('Unknown flight mode: %s', obj.mode);
            end
            cmd.mode = obj.mode;
        end
    end

    methods (Access = private)
        % =================================================================
        % Stabilized (FlightTaskManualAcceleration). No XY/Z hold.
        % Stick → roll/pitch angle, yaw rate, direct thrust.
        % =================================================================
        function cmd = runStabilized(obj, state, sticks, dt)
            [r, p_ang, yawspeed] = obj.stickToAngleAndYawrate(sticks);
            obj.yaw_lock = wrapToPi(obj.yaw_lock + yawspeed * dt);

            % Throttle: stick centre → hover, +1 → thr_max, -1 → thr_min.
            thr = obj.throttleStickToThrust(sticks.left_y);

            cmd.kind          = 'attitude';
            cmd.q_sp          = euler_to_quat(r, p_ang, obj.yaw_lock);
            cmd.thrust_body_z = -thr;
            cmd.yaw_sp        = obj.yaw_lock;
            cmd.yawspeed_sp   = yawspeed;
            cmd.pos_sp        = state.position_ned;
            cmd.vel_sp_ff     = nan(3, 1);
            cmd.acc_sp_ff     = [];
        end

        % =================================================================
        % Altitude (FlightTaskManualAltitude). Roll/pitch direct from
        % sticks (no XY hold); altitude is held by integrating the throttle
        % stick into alt_target, then a Z-PID drives thrust.
        % =================================================================
        function cmd = runAltitude(obj, state, sticks, dt)
            [r, p_ang, yawspeed] = obj.stickToAngleAndYawrate(sticks);
            obj.yaw_lock = wrapToPi(obj.yaw_lock + yawspeed * dt);

            % Throttle stick → vertical-velocity setpoint (NED z).
            % Up stick (+) → climb → vel_z_ned negative.
            sthr = obj.applyDeadzoneExpo(sticks.left_y);
            if sthr > 0
                vel_z_target = -sthr * obj.p.pos.vel_z_up;
            else
                vel_z_target = -sthr * obj.p.pos.vel_z_down;
            end

            % Altitude lock: when stick centred and vehicle stopped, latch
            % alt_target. Otherwise integrate the velocity command.
            if abs(sthr) < obj.p.man.deadzone && ...
               abs(state.velocity_ned(3)) < obj.p.man.hold_max_z
                if ~obj.z_locked
                    obj.alt_target = state.position_ned(3);
                    obj.z_locked = true;
                end
            else
                obj.z_locked   = false;
                obj.alt_target = obj.alt_target + vel_z_target * dt;
            end

            thrust_body_z = obj.altitudeThrust(state, vel_z_target, dt);

            cmd.kind          = 'attitude';
            cmd.q_sp          = euler_to_quat(r, p_ang, obj.yaw_lock);
            cmd.thrust_body_z = thrust_body_z;
            cmd.yaw_sp        = obj.yaw_lock;
            cmd.yawspeed_sp   = yawspeed;
            cmd.pos_sp        = [state.position_ned(1:2); obj.alt_target];
            cmd.vel_sp_ff     = nan(3, 1);
            cmd.acc_sp_ff     = [];
        end

        % =================================================================
        % Position (FlightTaskManualPosition). Stick → horizontal velocity
        % FF (heading-rotated) and vertical velocity FF; position held when
        % sticks centred and vehicle is stopped.
        % =================================================================
        function cmd = runPosition(obj, state, sticks, dt)
            sx = obj.applyDeadzoneExpo(sticks.right_x);    % roll  → right velocity (body y)
            sy = obj.applyDeadzoneExpo(sticks.right_y);    % pitch → forward vel    (body x)
            sthr = obj.applyDeadzoneExpo(sticks.left_y);
            syaw = obj.applyDeadzoneExpo(sticks.left_x);
            yawspeed = syaw * obj.p.man.yaw_rate_max;
            obj.yaw_lock = wrapToPi(obj.yaw_lock + yawspeed * dt);

            % Heading-rotated XY velocity setpoint.
            vbody = [sy; sx] * obj.p.man.vel_xy_max;        % [vx_body; vy_body]
            R2 = [cos(obj.yaw_lock), -sin(obj.yaw_lock);
                  sin(obj.yaw_lock),  cos(obj.yaw_lock)];
            v_xy_ned = R2 * vbody;                          % NED x=N, y=E

            % XY position lock (FlightTaskManualPosition.cpp:107-108).
            if abs(sx) < obj.p.man.deadzone && abs(sy) < obj.p.man.deadzone && ...
               norm(state.velocity_ned(1:2)) < obj.p.man.hold_max_xy
                if ~obj.xy_locked
                    obj.pos_lock(1:2) = state.position_ned(1:2);
                    obj.xy_locked = true;
                end
            else
                obj.xy_locked      = false;
                obj.pos_lock(1:2)  = state.position_ned(1:2);  % no XY pos error
            end

            % Z velocity FF (same logic as Altitude).
            if sthr > 0
                vel_z_ff = -sthr * obj.p.pos.vel_z_up;
            else
                vel_z_ff = -sthr * obj.p.pos.vel_z_down;
            end

            % Z lock identical to Altitude.
            if abs(sthr) < obj.p.man.deadzone && ...
               abs(state.velocity_ned(3)) < obj.p.man.hold_max_z
                if ~obj.z_locked
                    obj.pos_lock(3) = state.position_ned(3);
                    obj.z_locked = true;
                end
            else
                obj.z_locked    = false;
                obj.pos_lock(3) = state.position_ned(3);
            end

            % Build command. vel_sp_ff is given on axes that are NOT locked.
            vel_sp_ff = nan(3, 1);
            if ~obj.xy_locked, vel_sp_ff(1:2) = v_xy_ned; end
            if ~obj.z_locked,  vel_sp_ff(3)   = vel_z_ff; end

            cmd.kind        = 'position';
            cmd.pos_sp      = obj.pos_lock;
            cmd.vel_sp_ff   = vel_sp_ff;
            cmd.acc_sp_ff   = [];
            cmd.yaw_sp      = obj.yaw_lock;
            cmd.yawspeed_sp = yawspeed;
        end

        % =================================================================
        % Hold (Loiter): freeze position at activation; sticks ignored.
        % =================================================================
        function cmd = runHold(obj, ~)
            cmd.kind        = 'position';
            cmd.pos_sp      = obj.pos_lock;
            cmd.vel_sp_ff   = [];
            cmd.acc_sp_ff   = [];
            cmd.yaw_sp      = obj.yaw_lock;
            cmd.yawspeed_sp = NaN;
        end

        % =================================================================
        % Mission: feed waypoints through Navigator, which produces a slewed
        % pos_sp + vel_sp_ff. When the mission finishes, hold the last WP.
        % =================================================================
        function cmd = runMission(obj, state, dt)
            if isempty(obj.nav) || size(obj.nav.waypoints, 1) == 0
                % No mission: degrade to Hold so the drone doesn't fall.
                cmd = obj.runHold(state);
                return;
            end
            [pos_sp, yaw_sp, vel_sp_ff, ~] = obj.nav.update(state.position_ned, dt);

            cmd.kind        = 'position';
            cmd.pos_sp      = pos_sp;
            cmd.vel_sp_ff   = vel_sp_ff;
            cmd.acc_sp_ff   = [];
            cmd.yaw_sp      = yaw_sp;
            cmd.yawspeed_sp = NaN;
        end

        % =================================================================
        % RTL: 3-phase state machine — climb to RTL_ALT at current XY,
        % cruise to home XY at RTL_ALT, then land at home.
        % =================================================================
        function cmd = runRTL(obj, state)
            rtl_alt_ned = -obj.p.auto.rtl_alt;
            home_xy = obj.home_pos(1:2);
            cur_xy  = state.position_ned(1:2);

            switch obj.rtl_phase
                case 'climb'
                    pos_sp = [cur_xy; rtl_alt_ned];
                    if abs(state.position_ned(3) - rtl_alt_ned) < obj.p.nav.alt_acc_rad
                        obj.rtl_phase = 'cruise';
                    end
                case 'cruise'
                    pos_sp = [home_xy; rtl_alt_ned];
                    if norm(cur_xy - home_xy) < obj.p.nav.acc_rad && ...
                       abs(state.position_ned(3) - rtl_alt_ned) < obj.p.nav.alt_acc_rad
                        obj.rtl_phase = 'land';
                        obj.pos_lock  = [home_xy; rtl_alt_ned];
                        obj.land_target_z = rtl_alt_ned;
                    end
                    pos_sp = [home_xy; rtl_alt_ned];
                case 'land'
                    cmd = obj.runLand(state, 1/obj.p.rate_hz.position);
                    return;
            end

            cmd.kind        = 'position';
            cmd.pos_sp      = pos_sp;
            cmd.vel_sp_ff   = [0; 0; 0];
            cmd.acc_sp_ff   = [];
            cmd.yaw_sp      = obj.yaw_lock;
            cmd.yawspeed_sp = NaN;
        end

        % =================================================================
        % Land: descend at MPC_LAND_SPEED, switching to MPC_LAND_CRWL near
        % the ground (FlightTaskAuto::_prepareLandSetpoints lines 226-301).
        % =================================================================
        function cmd = runLand(obj, state, dt)
            alt_above_ground = -state.position_ned(3);

            if alt_above_ground > obj.p.auto.land_alt1
                vz = obj.p.auto.land_speed;
            elseif alt_above_ground > obj.p.auto.land_alt3
                % linear blend between land_speed and land_crawl
                t = (alt_above_ground - obj.p.auto.land_alt3) / ...
                    (obj.p.auto.land_alt1 - obj.p.auto.land_alt3);
                vz = obj.p.auto.land_crawl + t * (obj.p.auto.land_speed - obj.p.auto.land_crawl);
            else
                vz = obj.p.auto.land_crawl;
            end

            obj.land_target_z = obj.land_target_z + vz * dt;          % NED z (down +)
            % Keep XY lock at the value latched on entry (or by RTL).
            cmd.kind        = 'position';
            cmd.pos_sp      = [obj.pos_lock(1:2); obj.land_target_z];
            cmd.vel_sp_ff   = [0; 0; vz];
            cmd.acc_sp_ff   = [];
            cmd.yaw_sp      = obj.yaw_lock;
            cmd.yawspeed_sp = NaN;
        end

        % =================================================================
        % Takeoff: climb to home_xy + takeoff_alt, then auto-transition to
        % Hold once within altitude acceptance.
        % =================================================================
        function cmd = runTakeoff(obj, state)
            target_z = -obj.p.auto.takeoff_alt;
            target_xy = obj.pos_lock(1:2);              % whatever position we were in on entry

            cmd.kind        = 'position';
            cmd.pos_sp      = [target_xy; target_z];
            cmd.vel_sp_ff   = [0; 0; -obj.p.auto.takeoff_speed];
            cmd.acc_sp_ff   = [];
            cmd.yaw_sp      = obj.yaw_lock;
            cmd.yawspeed_sp = NaN;

            if abs(state.position_ned(3) - target_z) < obj.p.nav.alt_acc_rad
                obj.takeoff_complete = true;
                obj.mode = 'hold';
                obj.pos_lock = state.position_ned;
            end
        end

        % =================================================================
        % Stick helpers
        % =================================================================
        function [roll, pitch, yawspeed] = stickToAngleAndYawrate(obj, sticks)
            sx = obj.applyDeadzoneExpo(sticks.right_x);
            sy = obj.applyDeadzoneExpo(sticks.right_y);
            syaw = obj.applyDeadzoneExpo(sticks.left_x);

            roll  =  sx * obj.p.man.tilt_max;     % stick right → roll right (positive)
            pitch = -sy * obj.p.man.tilt_max;     % stick forward (positive y) → nose down (negative pitch)
            yawspeed = syaw * obj.p.man.yaw_rate_max;
        end

        function thrust = throttleStickToThrust(obj, sthr)
            % stick = -1 → thr_min, 0 → hover, +1 → thr_max.
            if sthr >= 0
                thrust = obj.p.pos.thr_hover + sthr * (obj.p.pos.thr_max - obj.p.pos.thr_hover);
            else
                thrust = obj.p.pos.thr_hover + sthr * (obj.p.pos.thr_hover - obj.p.pos.thr_min);
            end
            thrust = max(obj.p.pos.thr_min, min(obj.p.pos.thr_max, thrust));
        end

        function s = applyDeadzoneExpo(obj, s)
            % Sticks::checkAndUpdateStickInputs deadzone + expo.
            dz = obj.p.man.deadzone;
            if abs(s) < dz
                s = 0; return;
            end
            s = sign(s) * (abs(s) - dz) / (1 - dz);
            % Cubic expo: out = expo * s^3 + (1 - expo) * s.
            s = obj.p.man.expo * s^3 + (1 - obj.p.man.expo) * s;
            s = max(-1, min(1, s));
        end

        % =================================================================
        % Z-PID for Altitude mode. Mirrors PositionControl.cpp's Z chain
        % but uses the *current* attitude's body-z to project to body
        % thrust (since we're not letting the position controller pick
        % the tilt direction in this mode).
        % =================================================================
        function thrust_body_z = altitudeThrust(obj, state, vel_z_target, dt)
            % Position-P → vel_target (NED z; positive = down). Cap to vel limits.
            vel_z_pos = (obj.alt_target - state.position_ned(3)) * obj.p.pos.gain_pos_p(3);
            vel_sp_z  = vel_z_pos + vel_z_target;
            vel_sp_z  = max(-obj.p.pos.vel_z_up, min(obj.p.pos.vel_z_down, vel_sp_z));

            % Velocity-PID → acceleration setpoint (NED z).
            vel_err = vel_sp_z - state.velocity_ned(3);
            obj.alt_int = obj.alt_int + obj.p.pos.gain_vel_i(3) * vel_err * dt;
            obj.alt_int = max(-obj.p.g, min(obj.p.g, obj.alt_int));
            acc_sp_z = obj.p.pos.gain_vel_p(3) * vel_err + obj.alt_int ...
                     - obj.p.pos.gain_vel_d(3) * state.acceleration_ned(3);

            % NED-z thrust (negative because thrust pushes "up", i.e. -z).
            thrust_ned_z = acc_sp_z * (obj.p.pos.thr_hover / obj.p.g) - obj.p.pos.thr_hover;

            % Project onto body z. body_z_ned(3) = cos of tilt from level.
            R_b2n = quat_to_dcm(state.attitude_q);
            cos_tilt = max(0.5, R_b2n(3, 3));
            thrust_body_z = thrust_ned_z / cos_tilt;
            thrust_body_z = max(-obj.p.pos.thr_max, min(-obj.p.pos.thr_min, thrust_body_z));
        end
    end
end


function y = quat_yaw(q)
% Yaw component of a Hamilton quaternion [w; x; y; z] — third element of
% quat_to_euler, inlined to avoid a circular dependency on the full Euler
% conversion (quat_to_euler is fine, this is just a shorthand).
w = q(1); x = q(2); y_ = q(3); z = q(4);
siny_cosp = 2 * (w*z + x*y_);
cosy_cosp = 1 - 2 * (y_*y_ + z*z);
y = atan2(siny_cosp, cosy_cosp);
end


function y = wrapToPi(x)
% Wrap to (-pi, pi]. Standalone to avoid Mapping Toolbox dependency.
y = mod(x + pi, 2*pi) - pi;
end
