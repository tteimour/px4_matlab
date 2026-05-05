classdef QuadrotorDynamics < handle
% Six-DOF rigid-body quadrotor dynamics, FRD body / NED world.
% Iris airframe parameters (px4_params).
%
% State (column vectors):
%   pos_ned       3x1 position in NED (m)
%   vel_ned       3x1 velocity in NED (m/s)
%   q             4x1 attitude quaternion [w;x;y;z], body-to-NED
%   omega_b       3x1 body angular rate FRD (rad/s)
%   acc_ned       3x1 last specific acceleration in NED (m/s^2), exposed
%                     in lieu of an EKF (CLAUDE.md: ground-truth state).
%
% Inputs:
%   motor_cmd     4x1 normalized motor commands in [0,1]
%                 (rotor numbering matches PX4 quad-X: 0=FR, 1=BL, 2=FL, 3=BR)
%
% External forces modelled: gravity, motor thrust along body -z.
% Aerodynamic drag, ground effect, motor dynamics: omitted (out of scope
% per CLAUDE.md; controller does not depend on them).

    properties
        % --- physical constants (immutable after construct) ---
        mass            % kg
        I               % 3x3 inertia tensor (FRD body)
        I_inv           % 3x3 inverse (cached)
        g               % m/s^2
        rotor_pos       % 3x4 column-i = rotor i position in FRD (m)
        rotor_thrust_max % scalar, per-rotor max thrust (N)
        rotor_km        % scalar, momentConstant (Iris: 0.06)
        rotor_spin      % 4x1, +1 CW / -1 CCW

        % --- mutable state ---
        pos_ned         % 3x1
        vel_ned         % 3x1
        q               % 4x1 [w;x;y;z]
        omega_b         % 3x1
        acc_ned         % 3x1, last computed
    end

    methods
        function obj = QuadrotorDynamics(p)
            obj.mass  = p.airframe.mass;
            obj.I     = p.airframe.I;
            obj.I_inv = inv(p.airframe.I);
            obj.g     = p.g;
            obj.rotor_pos        = p.airframe.rotor_pos_frd;
            obj.rotor_thrust_max = p.airframe.rotor_thrust_max;
            obj.rotor_km         = p.airframe.rotor_moment_const;
            obj.rotor_spin       = p.airframe.rotor_spin;

            obj.pos_ned = [0;0;0];
            obj.vel_ned = [0;0;0];
            obj.q       = [1;0;0;0];
            obj.omega_b = [0;0;0];
            obj.acc_ned = [0;0;0];
        end

        function s = state(obj)
        % Ground-truth state struct consumed by the controllers, in the
        % shape declared in CLAUDE.md.
            s.position_ned    = obj.pos_ned;
            s.velocity_ned    = obj.vel_ned;
            s.attitude_q      = obj.q;
            s.angular_vel_b   = obj.omega_b;
            s.acceleration_ned = obj.acc_ned;
        end

        function reset(obj, pos_ned, yaw)
        % Initialize plant in level flight at given NED position + yaw.
            if nargin < 2, pos_ned = [0;0;0]; end
            if nargin < 3, yaw = 0; end
            obj.pos_ned = pos_ned(:);
            obj.vel_ned = [0;0;0];
            obj.q = [cos(yaw/2); 0; 0; sin(yaw/2)];
            obj.omega_b = [0;0;0];
            obj.acc_ned = [0;0;0];
        end

        function step(obj, motor_cmd, dt)
        % Advance the state by dt using fourth-order Runge-Kutta.
        % motor_cmd is clamped to [0,1] before use (the allocator is
        % expected to do this, but we double-check to keep the plant
        % robust to misuse).
            motor_cmd = max(min(motor_cmd(:), 1), 0);
            x0 = obj.pack();

            k1 = obj.deriv(x0,                    motor_cmd);
            k2 = obj.deriv(obj.add_(x0, dt/2*k1), motor_cmd);
            k3 = obj.deriv(obj.add_(x0, dt/2*k2), motor_cmd);
            k4 = obj.deriv(obj.add_(x0, dt  *k3), motor_cmd);

            x1 = obj.add_(x0, (dt/6) * (k1 + 2*k2 + 2*k3 + k4));
            obj.unpack(x1);

            % Recompute current acceleration so state() exposes a fresh value.
            [~, ~, ~, ~, obj.acc_ned] = obj.forces(obj.q, motor_cmd);

            % Renormalize quaternion to combat integration drift.
            obj.q = quat_normalize(obj.q);
        end
    end

    % ============================================================
    % Internals
    % ============================================================
    methods (Access = private)
        function x = pack(obj)
        % Pack state into a 13-vector for the integrator.
            x = [obj.pos_ned; obj.vel_ned; obj.q; obj.omega_b];
        end

        function unpack(obj, x)
            obj.pos_ned = x(1:3);
            obj.vel_ned = x(4:6);
            obj.q       = x(7:10);
            obj.omega_b = x(11:13);
        end

        function y = add_(~, a, b)
            y = a + b;
        end

        function dx = deriv(obj, x, motor_cmd)
        % d/dt of [pos; vel; q; omega] under thrust + gravity + body torque.
            vel    = x(4:6);
            qx     = x(7:10);
            omega  = x(11:13);

            % Forces / torques in their natural frames.
            [F_ned, tau_b, ~, ~, acc_ned] = obj.forces(qx, motor_cmd);
            obj.acc_ned = acc_ned;  %#ok<NASGU> kept for state() consistency

            % Position derivative: world velocity.
            dpos = vel;
            % Velocity derivative: F_ned / m  (gravity already included).
            dvel = F_ned / obj.mass;
            % Quaternion kinematics: dq = 0.5 * q ⊗ [0; omega_body]
            omega_q = [0; omega(1); omega(2); omega(3)];
            dq = 0.5 * quat_multiply(qx, omega_q);
            % Euler's rigid-body equation (FRD body):
            %   I*omega_dot + omega x (I*omega) = tau
            domega = obj.I_inv * (tau_b - cross(omega, obj.I * omega));

            dx = [dpos; dvel; dq; domega];
        end

        function [F_ned, tau_b, F_body, T_total, acc_ned] = forces(obj, qx, motor_cmd)
        % Compute net force in NED, net torque in body, body-frame thrust
        % vector and total thrust magnitude.
            % Per-rotor thrust magnitude (N).
            T = motor_cmd .* obj.rotor_thrust_max;          % 4x1
            T_total = sum(T);

            % Each rotor pushes the body along -z_body (FRD: up).
            F_body = [0; 0; -T_total];

            % Rotate body force into NED for translational dynamics.
            R_b2n = quat_to_dcm(qx);
            F_thrust_ned = R_b2n * F_body;

            % Gravity in NED (z-down).
            F_grav_ned = [0; 0; obj.mass * obj.g];

            F_ned = F_thrust_ned + F_grav_ned;

            % Torques about CG in body frame:
            %   tau_arm_i = r_i x F_i, with F_i = (0,0,-T_i)
            %   tau_react_i = spin_i * km * T_i along body +z (CW prop -> +z reaction in FRD)
            tau_b = [0;0;0];
            for i = 1:4
                r = obj.rotor_pos(:, i);
                Fi = [0; 0; -T(i)];
                tau_b = tau_b + cross(r, Fi);
                tau_b(3) = tau_b(3) + obj.rotor_spin(i) * obj.rotor_km * T(i);
            end

            % Specific acceleration in NED (excluding gravity is also useful;
            % here we report total inertial accel as the EKF would).
            acc_ned = F_ned / obj.mass;
        end
    end
end
