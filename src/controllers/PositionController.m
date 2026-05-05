classdef PositionController < handle
% Cascaded position-velocity-acceleration controller. Direct translation of
%   src/modules/mc_pos_control/PositionControl/PositionControl.cpp
% (specifically _positionControl(), _velocityControl(), _accelerationControl(),
%  and getAttitudeSetpoint()).
%
% Inputs each tick:
%   pos        : 3x1 NED position (m)
%   vel        : 3x1 NED velocity (m/s)
%   acc        : 3x1 NED acceleration (m/s^2), used for D term
%   pos_sp     : 3x1 NED position setpoint
%   yaw_sp     : scalar yaw setpoint (rad)
%   vel_sp_ff  : 3x1 feedforward velocity (NaN to ignore)
%   acc_sp_ff  : 3x1 feedforward acceleration (NaN to ignore)
%   yawspeed_sp: scalar feedforward yaw rate (NaN to ignore)
%   dt         : timestep (s)
%
% Outputs:
%   q_sp           : 4x1 attitude quaternion (body-to-NED)
%   thrust_body_z  : scalar thrust along body -z (negative when up); the
%                    magnitude is what the allocator consumes.
%   yawspeed_sp_out: pass-through to attitude controller.
%
% Notes:
%   - Output `thrust_body_z` is in normalized [0,1] units (PX4 convention),
%     ready for the allocator. PX4 uses negative numbers because body z
%     is down; we return abs() externally where needed.

    properties
        % Tunable
        gain_pos_p   % 3x1
        gain_vel_p   % 3x1
        gain_vel_i   % 3x1
        gain_vel_d   % 3x1
        lim_vel_horizontal
        lim_vel_up
        lim_vel_down
        lim_tilt     % rad
        thr_min
        thr_max
        hover_thrust
        g

        % horizontal-thrust margin (PX4: MPC_THR_XY_MARG default 0.3)
        thr_xy_margin = 0.3;

        % Integrator state
        vel_int      % 3x1
    end

    methods
        function obj = PositionController(p)
            obj.gain_pos_p = p.pos.gain_pos_p;
            obj.gain_vel_p = p.pos.gain_vel_p;
            obj.gain_vel_i = p.pos.gain_vel_i;
            obj.gain_vel_d = p.pos.gain_vel_d;
            obj.lim_vel_horizontal = p.pos.vel_xy_max;
            obj.lim_vel_up   = p.pos.vel_z_up;
            obj.lim_vel_down = p.pos.vel_z_down;
            obj.lim_tilt = p.pos.tilt_max;
            obj.thr_min = p.pos.thr_min;
            obj.thr_max = p.pos.thr_max;
            obj.hover_thrust = p.pos.thr_hover;
            obj.g = p.g;

            obj.vel_int = [0;0;0];
        end

        function reset(obj)
            obj.vel_int = [0;0;0];
        end

        function [q_sp, thrust_body_z, vel_sp, acc_sp, thr_sp] = update( ...
                obj, pos, vel, acc, pos_sp, yaw_sp, ...
                vel_sp_ff, acc_sp_ff, dt) %#ok<INUSD>

            if nargin < 8 || isempty(vel_sp_ff), vel_sp_ff = nan(3,1); end
            if nargin < 9 || isempty(acc_sp_ff), acc_sp_ff = nan(3,1); end

            % --- Position loop -> velocity setpoint ---
            % vel_sp_position = (pos_sp - pos) .* gain_pos_p
            vel_sp_position = (pos_sp - pos) .* obj.gain_pos_p;

            % vel_sp = vel_sp_position + ff (NaN-safe addition)
            vel_sp = vel_sp_position;
            mask = isfinite(vel_sp_ff);
            vel_sp(mask) = vel_sp(mask) + vel_sp_ff(mask);

            % Constrain horizontal velocity (PX4 ControlMath::constrainXY).
            % The "P" component takes priority, then we add as much FF as fits.
            v_p_xy = vel_sp_position(1:2);
            v_ff_xy = vel_sp(1:2) - v_p_xy;
            vel_sp(1:2) = constrainXY(v_p_xy, v_ff_xy, obj.lim_vel_horizontal);
            % Constrain vertical: NED z+ down, so up-velocity (negative z) limited
            % by lim_vel_up, down-velocity (positive z) by lim_vel_down.
            vel_sp(3) = max(-obj.lim_vel_up, min(obj.lim_vel_down, vel_sp(3)));

            % --- Velocity loop -> acceleration setpoint ---
            % Vertical integrator pre-clamp (PositionControl.cpp:142)
            obj.vel_int(3) = max(-obj.g, min(obj.g, obj.vel_int(3)));

            vel_error = vel_sp - vel;
            acc_sp_velocity = vel_error .* obj.gain_vel_p ...
                            + obj.vel_int ...
                            - acc .* obj.gain_vel_d;

            % Acc setpoint = acc_sp_velocity + acc_ff (NaN-safe)
            acc_sp = acc_sp_velocity;
            mask = isfinite(acc_sp_ff);
            acc_sp(mask) = acc_sp(mask) + acc_sp_ff(mask);

            % --- Acceleration -> body_z + collective thrust ---
            [thr_sp, body_z] = obj.accelerationToThrust(acc_sp);

            % --- Vertical anti-windup: hold integrator when saturated ---
            if (thr_sp(3) >= -obj.thr_min && vel_error(3) >= 0) || ...
               (thr_sp(3) <= -obj.thr_max && vel_error(3) <= 0)
                vel_error(3) = 0;
            end

            % --- Vertical-priority XY thrust saturation ---
            thr_xy_norm = norm(thr_sp(1:2));
            thr_max_sq = obj.thr_max^2;
            allocated_xy = min(thr_xy_norm, obj.thr_xy_margin);
            thr_z_max_sq = thr_max_sq - allocated_xy^2;
            thr_sp(3) = max(thr_sp(3), -sqrt(max(0, thr_z_max_sq)));

            thr_max_xy_sq = thr_max_sq - thr_sp(3)^2;
            thr_max_xy = 0;
            if thr_max_xy_sq > 0
                thr_max_xy = sqrt(thr_max_xy_sq);
            end
            if thr_xy_norm > thr_max_xy
                thr_sp(1:2) = thr_sp(1:2) / thr_xy_norm * thr_max_xy;
            end

            % --- Horizontal tracking anti-windup (Rundqwist 1990) ---
            % acc_xy that the saturated thrust actually produces:
            acc_sp_xy_produced = thr_sp(1:2) * (obj.g / obj.hover_thrust);
            if dot(acc_sp(1:2), acc_sp(1:2)) > dot(acc_sp_xy_produced, acc_sp_xy_produced)
                arw_gain = 2 / obj.gain_vel_p(1);
                vel_error(1:2) = vel_error(1:2) - arw_gain * (acc_sp(1:2) - acc_sp_xy_produced);
            end

            % --- Integrate velocity-loop integrator ---
            vel_error(~isfinite(vel_error)) = 0;
            obj.vel_int = obj.vel_int + vel_error .* obj.gain_vel_i * dt;

            % --- Build attitude setpoint from thrust setpoint ---
            [q_sp, thrust_body_z] = thrust_to_attitude(thr_sp, yaw_sp);

            % body_z is unused externally but kept for diagnostics.
            %#ok<NASGU>
        end
    end

    methods (Access = private)
        function [thr_sp, body_z] = accelerationToThrust(obj, acc_sp)
        % PositionControl.cpp:204-222 (_accelerationControl)
            z_specific_force = -obj.g + acc_sp(3);
            body_z = [-acc_sp(1); -acc_sp(2); -z_specific_force];
            n = norm(body_z);
            if n > 0
                body_z = body_z / n;
            else
                body_z = [0; 0; 1];
            end
            body_z = limit_tilt(body_z, [0;0;1], obj.lim_tilt);

            % Hover-thrust scaling: T_hover delivers exactly 1 g, so the
            % "thrust per unit acceleration" is hover/g.
            thrust_ned_z = acc_sp(3) * (obj.hover_thrust / obj.g) - obj.hover_thrust;
            cos_ned_body = body_z(3);  % dot([0;0;1], body_z)
            % Collective thrust along body z (negative because body z down):
            % T_proj = thrust_ned_z / cos_ned_body, but capped by -thr_min
            % so the cap (thrust must be at least obj.thr_min upward).
            collective_thrust = min(thrust_ned_z / max(cos_ned_body, 1e-3), -obj.thr_min);
            thr_sp = body_z * collective_thrust;
        end
    end
end


function v = constrainXY(v0, v1, max_norm)
% Direct port of ControlMath::constrainXY() (ControlMath.cpp).
% Prioritizes v0 (P component); fits as much of v1 (FF) as the budget allows.

if norm(v0 + v1) <= max_norm
    v = v0 + v1;
elseif norm(v0) >= max_norm
    % v0 alone exceeds budget: keep direction of v0, scale to max.
    v = v0 / max(norm(v0), eps) * max_norm;
else
    % Solve for s so that |v0 + s*v1| = max_norm, s in [0,1].
    a = dot(v1, v1);
    b = 2 * dot(v0, v1);
    c = dot(v0, v0) - max_norm^2;
    if a < eps
        v = v0;
        return;
    end
    disc = b*b - 4*a*c;
    s = (-b + sqrt(max(0, disc))) / (2*a);
    s = max(0, min(1, s));
    v = v0 + s * v1;
end
end
