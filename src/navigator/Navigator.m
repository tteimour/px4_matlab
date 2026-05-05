classdef Navigator < handle
% Mission/auto-mode waypoint sequencer with slewed position setpoint.
%
% The internal position setpoint moves toward the active waypoint at
% (vel_xy_max, vel_z_up/down) — a coarse stand-in for PX4 FlightTaskAuto's
% jerk-limited path generator. The position controller then sees a
% smooth target instead of a step, which is what allows the cascade to
% track cleanly between waypoints.
%
% References:
%   src/modules/navigator/mission.cpp
%   src/modules/navigator/mission_block.cpp  (acceptance check, l. 303-372)
%   src/modules/navigator/navigator_params.c (NAV_ACC_RAD)
%   src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp
%       (in PX4: jerk-limited shaping; we use a constant-velocity slew)
%
% Acceptance: identical to PX4 — horizontal radius AND vertical tolerance
% checked against the *vehicle* position, not the slewed setpoint.

    properties
        waypoints       % Nx5: [N E D yaw_rad acc_rad]
        idx             % active waypoint index (1-based)
        acc_rad_default % NAV_ACC_RAD
        alt_acc_rad     % NAV_MC_ALT_RAD

        vel_xy_max      % horizontal slew speed (m/s)
        vel_z_up        % up speed limit (m/s, applied to negative z motion)
        vel_z_down      % down speed limit (m/s, applied to positive z motion)

        pos_sp_internal % 3x1, slewed setpoint (NED)
        initialized     % logical
    end

    methods
        function obj = Navigator(p, waypoints)
            obj.waypoints       = waypoints;
            obj.idx             = 1;
            obj.acc_rad_default = p.nav.acc_rad;
            obj.alt_acc_rad     = p.nav.alt_acc_rad;
            obj.vel_xy_max      = p.pos.vel_xy_max;
            obj.vel_z_up        = p.pos.vel_z_up;
            obj.vel_z_down      = p.pos.vel_z_down;
            obj.pos_sp_internal = [0;0;0];
            obj.initialized     = false;
        end

        function [pos_sp, yaw_sp, vel_sp_ff, done] = update(obj, pos, dt)
        % Returns the slewed position setpoint, yaw setpoint, velocity
        % feedforward, and a `done` flag once the last waypoint is reached.
            if ~obj.initialized
                obj.pos_sp_internal = pos(:);
                obj.initialized = true;
            end

            done = obj.idx > size(obj.waypoints, 1);
            if done
                pos_sp = obj.waypoints(end, 1:3)';
                yaw_sp = obj.waypoints(end, 4);
                vel_sp_ff = [0;0;0];
                return;
            end

            wp      = obj.waypoints(obj.idx, :);
            target  = wp(1:3)';
            yaw_sp  = wp(4);
            acc_rad = wp(5);
            if ~isfinite(acc_rad), acc_rad = obj.acc_rad_default; end

            % --- Slew the internal setpoint toward the target ---
            delta = target - obj.pos_sp_internal;

            % Horizontal
            d_xy = norm(delta(1:2));
            max_step_xy = obj.vel_xy_max * dt;
            if d_xy <= max_step_xy
                obj.pos_sp_internal(1:2) = target(1:2);
                vel_xy = [0; 0];
            else
                dir_xy = delta(1:2) / d_xy;
                obj.pos_sp_internal(1:2) = obj.pos_sp_internal(1:2) + dir_xy * max_step_xy;
                vel_xy = dir_xy * obj.vel_xy_max;
            end

            % Vertical (sign of delta(3) selects up vs down speed cap)
            if delta(3) >= 0
                v_lim = obj.vel_z_down;     % +z = down in NED
            else
                v_lim = obj.vel_z_up;
            end
            max_step_z = v_lim * dt;
            if abs(delta(3)) <= max_step_z
                obj.pos_sp_internal(3) = target(3);
                vel_z = 0;
            else
                obj.pos_sp_internal(3) = obj.pos_sp_internal(3) + sign(delta(3)) * max_step_z;
                vel_z = sign(delta(3)) * v_lim;
            end

            pos_sp    = obj.pos_sp_internal;
            vel_sp_ff = [vel_xy; vel_z];

            % --- Acceptance check (against vehicle, not slewed setpoint) ---
            d_xy_veh = norm(pos(1:2) - target(1:2));
            d_z_veh  = abs(pos(3) - target(3));
            if d_xy_veh <= acc_rad && d_z_veh <= obj.alt_acc_rad
                obj.idx = obj.idx + 1;
            end

            done = obj.idx > size(obj.waypoints, 1);
        end
    end
end
