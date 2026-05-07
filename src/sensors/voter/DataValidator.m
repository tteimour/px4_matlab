classdef DataValidator < handle
% DataValidator — per-sensor health & confidence tracker.
%
% Translation of PX4 src/lib/validation/DataValidator.cpp (lines 47-142).
% Tracks:
%   * elapsed time since last sample (for timeout)
%   * error_count delta (for error_density accumulation)
%   * stale-data detection (same value repeated)
%   * confidence = 1 - error_density / ERROR_DENSITY_WINDOW
%
% A sensor is considered healthy if confidence >= MIN_REGULAR_CONFIDENCE
% AND it has produced a sample within timeout. The validator is fed
% samples by the owning VotedSensors group.

    properties (Constant)
        ERROR_DENSITY_WINDOW   = 100.0   % samples
        MIN_REGULAR_CONFIDENCE = 0.9
        DEFAULT_TIMEOUT_S      = 0.2
    end

    properties
        priority      = 50          % effective priority (1..100)
        timeout_s     = 0.2
        error_count   = 0           % cumulative error count from sensor
        error_density = 0           % decaying error rate
        stale_count   = 0
        last_t        = -inf
        last_value    = []          % last raw scalar/vector — for stale detect
    end

    methods
        function put(obj, t, value_for_stale_check, error_count_in)
            if nargin < 4, error_count_in = obj.error_count; end

            % Error density: increments on each new error, decays toward 0.
            if error_count_in > obj.error_count
                d = error_count_in - obj.error_count;
                obj.error_density = min( ...
                    obj.error_density + d, ...
                    DataValidator.ERROR_DENSITY_WINDOW);
            else
                obj.error_density = max(obj.error_density - 1, 0);
            end
            obj.error_count = error_count_in;

            % Stale detection.
            if ~isempty(obj.last_value) && isequal(obj.last_value, value_for_stale_check)
                obj.stale_count = obj.stale_count + 1;
            else
                obj.stale_count = 0;
            end
            obj.last_value = value_for_stale_check;
            obj.last_t     = t;
        end

        function c = confidence(obj, t_now)
            % Map error density to a 0..1 confidence; 0 if timed out or stale.
            if t_now - obj.last_t > obj.timeout_s
                c = 0.0;
                return;
            end
            if obj.stale_count > 100
                c = 0.0;
                return;
            end
            c = 1.0 - obj.error_density / DataValidator.ERROR_DENSITY_WINDOW;
            c = max(0.0, min(1.0, c));
        end

        function tf = healthy(obj, t_now)
            tf = obj.confidence(t_now) >= DataValidator.MIN_REGULAR_CONFIDENCE;
        end

        function reset(obj)
            obj.error_count   = 0;
            obj.error_density = 0;
            obj.stale_count   = 0;
            obj.last_t        = -inf;
            obj.last_value    = [];
        end
    end
end
