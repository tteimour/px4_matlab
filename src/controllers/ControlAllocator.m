classdef ControlAllocator < handle
% Quad-X control allocator. Inverts the effectiveness matrix derived from
% the PX4 4001_quad_x airframe definition (CA_ROTORn_PX/PY/KM) to map
% (tau_x, tau_y, tau_z, T_z) to four normalized motor commands in [0,1].
%
% Reference:
%   src/modules/control_allocator/VehicleActuatorEffectiveness/
%       ActuatorEffectivenessRotors.cpp   (effectiveness construction)
%   src/lib/control_allocation/control_allocation/
%       ControlAllocationPseudoInverse.cpp (allocation via pseudo-inverse)
%
% Effectiveness construction (with axis = (0,0,-1) for upward thrust on
% a multicopter, FRD body):
%   For rotor i at position (px_i, py_i):
%     tau_x_i = -ct * py_i           (-py because z component of (p x axis) with axis_z=-1)
%     tau_y_i = +ct * px_i
%     tau_z_i = +ct * km_i           (sign of spin)
%     T_z_i   = +ct                  (we use POSITIVE-thrust-magnitude convention here:
%                                     downstream the plant treats this as upward force)
%
% Output convention:
%   - Each motor command is a normalized [0,1] value.
%   - Total thrust T_z is taken as a positive scalar (magnitude of upward
%     thrust expressed in motor-command units; 1.0 = all motors at full).

    properties
        E       % 4x4 effectiveness matrix
        Einv    % 4x4 pseudo-inverse, NORMALIZED so input=1 maps to motor saturation.
                % Matches PX4 ControlAllocationPseudoInverse::normalizeControlAllocationMatrix().
        scale   % 4x1 per-axis normalization factors (roll, pitch, yaw, thrust)
        sat_pos % 3x1 logical: positive saturation flags (roll, pitch, yaw)
        sat_neg % 3x1 logical: negative saturation flags
    end

    methods
        function obj = ControlAllocator(p)
            ct = 1.0;   % unit thrust coefficient at the allocator level
            n = p.alloc.n_rotors;
            E = zeros(4, n);
            for i = 1:n
                px = p.alloc.rotor_px(i);
                py = p.alloc.rotor_py(i);
                km = p.alloc.rotor_km(i);
                E(1, i) = -ct * py;     % tau_x
                E(2, i) =  ct * px;     % tau_y
                E(3, i) =  ct * km;     % tau_z
                E(4, i) =  ct;          % thrust magnitude
            end
            obj.E = E;
            mix = pinv(E);              % nx4

            % --- per-axis normalization (PX4 convention) ---
            % Roll/pitch share the same scale, taken as the max RMS over
            % the two columns. (ControlAllocationPseudoInverse.cpp:99-115)
            scales = ones(4, 1);
            for ax = 1:2
                col = mix(:, ax);
                nz = sum(abs(col) > 1e-3);
                if nz > 0
                    scales(ax) = sqrt(sum(col.^2) / (nz / 2));
                end
            end
            scales(1) = max(scales(1), scales(2));
            scales(2) = scales(1);
            % Yaw uses the column max-abs (line 117).
            scales(3) = max(abs(mix(:, 3)));
            if scales(3) < 1e-9, scales(3) = 1; end
            % Thrust uses sum(|col|) / num_nonzero (line 134).
            col = mix(:, 4);
            nz = sum(abs(col) > 1e-9);
            if nz > 0
                scales(4) = sum(abs(col)) / nz;
            end

            % Apply normalization (line 158-167).
            mix(:, 1) = mix(:, 1) / scales(1);
            mix(:, 2) = mix(:, 2) / scales(2);
            mix(:, 3) = mix(:, 3) / scales(3);
            mix(:, 4) = mix(:, 4) / scales(4);

            obj.Einv = mix;
            obj.scale = scales;
            obj.sat_pos = [false; false; false];
            obj.sat_neg = [false; false; false];
        end

        function [m, sat_pos, sat_neg] = allocate(obj, tau, T)
        % tau : 3x1 torque setpoint [tau_x; tau_y; tau_z] (normalized)
        % T   : scalar collective thrust magnitude in [0,1]
        %
        % Returns motor commands (4x1) clamped to [0,1] and per-axis
        % saturation flags fed back to the rate controller.
            u = [tau(1); tau(2); tau(3); T];
            m_raw = obj.Einv * u;

            m = max(0, min(1, m_raw));

            % Detect axis-level saturation: if clamping changed any motor,
            % project the motor delta back to torque axes via E to see
            % which axis "lost" authority and in which direction.
            dm = m - m_raw;
            d_axes = obj.E * dm;          % 4x1: how torque/thrust changed
            sat_pos = false(3,1);
            sat_neg = false(3,1);
            tol = 1e-9;
            for i = 1:3
                if d_axes(i) < -tol
                    % We had to reduce torque on this axis -> positive demand was clipped.
                    sat_pos(i) = true;
                elseif d_axes(i) > tol
                    sat_neg(i) = true;
                end
            end
            obj.sat_pos = sat_pos;
            obj.sat_neg = sat_neg;
        end
    end
end
