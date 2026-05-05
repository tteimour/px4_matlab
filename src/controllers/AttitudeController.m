classdef AttitudeController < handle
% Quaternion attitude controller. Direct translation of
%   src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp:55-114
% (Brescianini et al. reduced-attitude formulation with yaw-priority weight.)
%
% update(q, q_sp, yawspeed_sp) returns a 3x1 body-rate setpoint (rad/s).

    properties
        proportional_gain   % 3x1 internally-stored gains; yaw scaled by 1/yaw_w
        yaw_w               % scalar in [0,1]
        rate_limit          % 3x1 rad/s
    end

    methods
        function obj = AttitudeController(p)
            obj.rate_limit = p.att.rate_max;
            obj.setProportionalGain(p.att.gain_p, p.att.yaw_weight);
        end

        function setProportionalGain(obj, gain, yaw_weight)
        % AttitudeControl.cpp:42-52
            obj.proportional_gain = gain;
            obj.yaw_w = max(0, min(1, yaw_weight));
            if obj.yaw_w > 1e-4
                obj.proportional_gain(3) = obj.proportional_gain(3) / obj.yaw_w;
            end
        end

        function rate_sp = update(obj, q, q_sp, yawspeed_sp)
        % q          : 4x1 current attitude (body-to-NED)
        % q_sp       : 4x1 desired attitude (body-to-NED)
        % yawspeed_sp: scalar feedforward yaw rate in NED-z (use NaN if none)
            qd = q_sp;

            % --- reduced desired attitude: ignore yaw, prioritize roll/pitch ---
            e_z   = quat_dcm_z(q);
            e_z_d = quat_dcm_z(qd);
            qd_red = quat_from_two_vectors(e_z, e_z_d);

            if abs(qd_red(2)) > (1 - 1e-5) || abs(qd_red(3)) > (1 - 1e-5)
                % Singular: a 180-deg flip, ignore reduction.
                qd_red = qd;
            else
                qd_red = quat_multiply(qd_red, q);
            end

            % --- extract delta yaw and re-scale by yaw weight ---
            qd_dyaw = quat_multiply(quat_inverse(qd_red), qd);
            qd_dyaw = quat_canonicalize(qd_dyaw);
            qd_dyaw(1) = max(-1, min(1, qd_dyaw(1)));
            qd_dyaw(4) = max(-1, min(1, qd_dyaw(4)));
            qd = quat_multiply(qd_red, ...
                [cos(obj.yaw_w * acos(qd_dyaw(1))); ...
                 0; 0; ...
                 sin(obj.yaw_w * asin(qd_dyaw(4)))]);

            % --- attitude error: quaternion from current to desired ---
            qe = quat_multiply(quat_inverse(q), qd);
            % AttitudeControl.cpp:92: e_q = 2 * Im(qe.canonical())
            qe = quat_canonicalize(qe);
            eq = 2 * qe(2:4);

            % --- proportional rate setpoint ---
            rate_sp = eq .* obj.proportional_gain;

            % --- world-frame yaw-rate feedforward, rotated into body ---
            if isfinite(yawspeed_sp)
                % AttitudeControl.cpp:104-106:
                %   rate_sp += q.inversed().dcm_z() * yawspeed_sp
                % q.inversed().dcm_z() is the NED z-axis expressed in body
                % frame (= R_n2b's third column = R_b2n's third row).
                R_b2n = quat_to_dcm(q);
                ned_z_in_body = R_b2n(3, :)';
                rate_sp = rate_sp + ned_z_in_body * yawspeed_sp;
            end

            % --- per-axis clamp ---
            rate_sp = max(-obj.rate_limit, min(obj.rate_limit, rate_sp));
        end
    end
end
