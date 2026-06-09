classdef LowPassFilter2p < handle
% LowPassFilter2p — 2nd-order Butterworth low-pass (biquad), per-axis on a
% 3x1 vector. Faithful port of PX4
%   src/lib/mathlib/math/filter/LowPassFilter2p.hpp
%     set_cutoff_frequency() lines 61-99   (coefficient design)
%     apply()                lines 99-110  (Direct Form II Transposed)
%     reset()                lines 125-145
%
% Models the gyro low-pass PX4's VehicleAngularVelocity applies
% (IMU_GYRO_CUTOFF, default 40 Hz; imu_gyro_parameters.c:128) BEFORE
% mc_rate_control consumes the angular rate. Coefficients are scalar; the
% delay elements are per-axis (3x1) so the three gyro axes filter
% independently with identical dynamics.

    properties
        b0 = 1; b1 = 0; b2 = 0; a1 = 0; a2 = 0;
        d1 = zeros(3, 1);     % delay element 1 (per axis)
        d2 = zeros(3, 1);     % delay element 2 (per axis)
        sample_freq = 0;
        cutoff_freq = 0;
        enabled = false;      % false -> passthrough (cutoff disabled)
    end

    methods
        function obj = LowPassFilter2p(sample_freq, cutoff_freq)
            obj.setCutoff(sample_freq, cutoff_freq);
        end

        function set_cutoff_frequency(obj, sample_freq, cutoff_freq)
        % PX4-named alias (math/filter/LowPassFilter2p.hpp) — callers
        % ported verbatim from PX4 (e.g. SystemIdentification) use this.
            obj.setCutoff(sample_freq, cutoff_freq);
        end

        function setCutoff(obj, sample_freq, cutoff_freq)
        % LowPassFilter2p.hpp:61-99. Disabled (passthrough) for invalid /
        % zero cutoff, matching PX4's guard.
            if (sample_freq <= 0) || (cutoff_freq <= 0) || ...
               (cutoff_freq >= sample_freq / 2) || ...
               ~isfinite(sample_freq) || ~isfinite(cutoff_freq)
                obj.enabled = false;
                obj.b0 = 1; obj.b1 = 0; obj.b2 = 0; obj.a1 = 0; obj.a2 = 0;
                return;
            end
            obj.cutoff_freq = max(cutoff_freq, sample_freq * 0.001);
            obj.sample_freq = sample_freq;

            fr  = sample_freq / obj.cutoff_freq;
            ohm = tan(pi / fr);
            c   = 1 + 2 * cos(pi / 4) * ohm + ohm^2;

            obj.b0 = ohm^2 / c;
            obj.b1 = 2 * obj.b0;
            obj.b2 = obj.b0;
            obj.a1 = 2 * (ohm^2 - 1) / c;
            obj.a2 = (1 - 2 * cos(pi / 4) * ohm + ohm^2) / c;
            obj.enabled = true;
        end

        function y = apply(obj, x)
        % Direct Form II Transposed (LowPassFilter2p.hpp:99-110).
            x = x(:);
            if ~obj.enabled, y = x; return; end
            d0 = x - obj.d1 * obj.a1 - obj.d2 * obj.a2;
            y  = d0 * obj.b0 + obj.d1 * obj.b1 + obj.d2 * obj.b2;
            obj.d2 = obj.d1;
            obj.d1 = d0;
        end

        function reset(obj, x)
        % Seed the delay line so a step input settles immediately
        % (LowPassFilter2p.hpp:125-145). Avoids a startup transient.
            x = x(:);
            if abs(1 + obj.a1 + obj.a2) > eps
                v = x / (1 + obj.a1 + obj.a2);
            else
                v = x;
            end
            obj.d1 = v;
            obj.d2 = v;
        end
    end
end
