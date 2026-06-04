classdef signal_generator
% Excitation-signal generators. Direct translation of
%   src/lib/system_identification/signal_generator.hpp  (PX4)
%
% Frequency sweeps (chirps) useful for offline / open-loop identification.
% The attitude autotuner itself injects a decreasing-period square wave
% (see McAutotuneAttitudeControl.getIdentificationSignal), but these
% sweeps are part of the ported library and are handy for bench tests.
%
% Implemented as a stateless class with static methods to mirror the PX4
% `signal_generator` namespace.

    methods (Static)
        function s = linearSineSweep(f_start, f_end, duration, t)
        % Linear-in-frequency sine sweep. Returns 0 for t > duration.
            if t > duration
                s = 0;
                return;
            end
            w_start = f_start * 2 * pi;
            w_end   = f_end * 2 * pi;
            s = sin(w_start * t + 0.5 * (w_end - w_start) * t * t / duration);
        end

        function s = logSineSweep(f_start, f_end, duration, t)
        % Logarithmic (exponential) sine sweep. Returns 0 for t > duration.
            if t > duration
                s = 0;
                return;
            end
            f_ratio = f_end / max(f_start, 0.1);
            s = sin(2 * pi * f_start * duration * (f_ratio ^ (t / duration) - 1) / log(f_ratio));
        end
    end
end
