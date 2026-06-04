classdef LowPassFilter2p < handle
% Second-order (biquad) low-pass filter, Direct Form II.
% Direct translation of
%   src/lib/mathlib/math/filter/LowPassFilter2p.hpp  (PX4)
%
% Butterworth design with quality factor Q = cos(pi/4) (damping ratio
% 1/sqrt(2)). Used by SystemIdentification to band-limit the input/output
% signals before recursive least-squares identification.
%
% Conventions: scalar signals only (the autotuner filters one axis at a
% time). dt is encoded as (sample_freq, cutoff_freq) exactly as in PX4.

    properties
        % Normalized biquad coefficients (a0 == 1).
        a1 = 0
        a2 = 0
        b0 = 1
        b1 = 0
        b2 = 0
        % Direct-Form-II delay elements (_delay_element_1/2 in PX4).
        d1 = 0
        d2 = 0
        cutoff_freq = 0
        sample_freq = 0
    end

    methods
        function obj = LowPassFilter2p(sample_freq, cutoff_freq)
            if nargin == 2
                obj.set_cutoff_frequency(sample_freq, cutoff_freq);
            end
        end

        function set_cutoff_frequency(obj, sample_freq, cutoff_freq)
        % LowPassFilter2p.hpp: set_cutoff_frequency()
            if (sample_freq <= 0) || (cutoff_freq <= 0) ...
                    || (cutoff_freq >= sample_freq / 2) ...
                    || ~isfinite(sample_freq) || ~isfinite(cutoff_freq)
                obj.disable();
                return;
            end

            % reset delay elements on filter change
            obj.d1 = 0;
            obj.d2 = 0;

            obj.cutoff_freq = max(cutoff_freq, sample_freq * 0.001);
            obj.sample_freq = sample_freq;

            fr  = obj.sample_freq / obj.cutoff_freq;
            ohm = tan(pi / fr);
            c   = 1 + 2 * cos(pi / 4) * ohm + ohm * ohm;

            obj.b0 = ohm * ohm / c;
            obj.b1 = 2 * obj.b0;
            obj.b2 = obj.b0;

            obj.a1 = 2 * (ohm * ohm - 1) / c;
            obj.a2 = (1 - 2 * cos(pi / 4) * ohm + ohm * ohm) / c;

            if ~all(isfinite([obj.b0, obj.b1, obj.b2, obj.a1, obj.a2]))
                obj.disable();
            end
        end

        function out = apply(obj, sample)
        % Direct Form II implementation (LowPassFilter2p.hpp: apply()).
            d0  = sample - obj.d1 * obj.a1 - obj.d2 * obj.a2;
            out = d0 * obj.b0 + obj.d1 * obj.b1 + obj.d2 * obj.b2;
            obj.d2 = obj.d1;
            obj.d1 = d0;
        end

        function out = reset(obj, sample)
        % LowPassFilter2p.hpp: reset(). Sets the delay line to the steady
        % state for a constant input so the filter does not ring on init.
            if ~isfinite(sample)
                sample = 0;
            end

            if abs(1 + obj.a1 + obj.a2) > eps('single')   % FLT_EPSILON
                obj.d1 = sample / (1 + obj.a1 + obj.a2);
                obj.d2 = obj.d1;
                if ~isfinite(obj.d1) || ~isfinite(obj.d2)
                    obj.d1 = sample;
                    obj.d2 = sample;
                end
            else
                obj.d1 = sample;
                obj.d2 = sample;
            end

            out = obj.apply(sample);
        end

        function disable(obj)
        % Pass-through (no filtering): b0 = 1, all other coeffs 0.
            obj.sample_freq = 0;
            obj.cutoff_freq = 0;
            obj.d1 = 0;
            obj.d2 = 0;
            obj.b0 = 1;
            obj.b1 = 0;
            obj.b2 = 0;
            obj.a1 = 0;
            obj.a2 = 0;
        end
    end
end
