classdef RateController < handle
% Body-rate PID + feedforward controller. Direct translation of
%   src/lib/rate_control/rate_control.cpp:71-118
%
% Signature mirrors PX4's RateControl::update():
%   torque = update(rate, rate_sp, angular_accel, dt, landed)
%
% Output is a normalized 3x1 torque vector (axes: roll, pitch, yaw) in
% the same units PX4 uses for downstream control allocation. Saturation
% feedback from the allocator is fed back via setSaturationStatus().

    properties
        gain_p          % 3x1 effective P (= MC_*RATE_P * MC_*RATE_K)
        gain_i          % 3x1 effective I
        gain_d          % 3x1 effective D
        gain_ff         % 3x1 feedforward
        lim_int         % 3x1 integrator clamp (MC_*R_INT_LIM)
        i_factor_cutoff % scalar (rad/s), rate_control.cpp:107

        rate_int        % 3x1 integrator state
        sat_pos         % 3x1 logical: allocator saturated positive on this axis
        sat_neg         % 3x1 logical: allocator saturated negative on this axis
    end

    methods
        function obj = RateController(p)
            obj.gain_p   = p.rate.gain_p   .* p.rate.gain_k;
            obj.gain_i   = p.rate.gain_i   .* p.rate.gain_k;
            obj.gain_d   = p.rate.gain_d   .* p.rate.gain_k;
            obj.gain_ff  = p.rate.gain_ff;
            obj.lim_int  = p.rate.int_lim;
            obj.i_factor_cutoff = p.rate.i_factor_cutoff_rate;

            obj.rate_int = [0;0;0];
            obj.sat_pos  = [false; false; false];
            obj.sat_neg  = [false; false; false];
        end

        function reset(obj)
            obj.rate_int = [0;0;0];
            obj.sat_pos = [false; false; false];
            obj.sat_neg = [false; false; false];
        end

        function setSaturationStatus(obj, sat_pos, sat_neg)
            obj.sat_pos = logical(sat_pos(:));
            obj.sat_neg = logical(sat_neg(:));
        end

        function torque = update(obj, rate, rate_sp, angular_accel, dt, landed)
        % rate_control.cpp:78
        %   torque = Kp .* err  +  rate_int  -  Kd .* angular_accel  +  Kff .* rate_sp
            rate_error = rate_sp - rate;

            torque = obj.gain_p .* rate_error ...
                   + obj.rate_int ...
                   - obj.gain_d .* angular_accel ...
                   + obj.gain_ff .* rate_sp;

            if ~landed
                obj.updateIntegral(rate_error, dt);
            end
        end
    end

    methods (Access = private)
        function updateIntegral(obj, rate_error, dt)
        % rate_control.cpp:88-118
            for i = 1:3
                err = rate_error(i);

                % Allocator saturation: clamp the contribution that would
                % drive the integrator further into saturation.
                if obj.sat_pos(i)
                    err = min(err, 0);
                end
                if obj.sat_neg(i)
                    err = max(err, 0);
                end

                % i-factor: quadratic decay vs. cutoff rate (rate_control.cpp:107-108).
                i_factor = err / obj.i_factor_cutoff;
                i_factor = max(0, 1 - i_factor*i_factor);

                rate_i = obj.rate_int(i) + i_factor * obj.gain_i(i) * err * dt;

                if isfinite(rate_i)
                    obj.rate_int(i) = max(-obj.lim_int(i), min(obj.lim_int(i), rate_i));
                end
            end
        end
    end
end
