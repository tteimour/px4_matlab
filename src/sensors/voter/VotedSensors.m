classdef VotedSensors < handle
% VotedSensors — sensor group with priority + confidence selection.
%
% Owns a list of Sensor handles + matching DataValidator instances.
% Steps them each sim tick, ingests fresh samples into validators, and
% selects the primary using PX4's get_best() rules:
%   src/lib/validation/DataValidatorGroup.cpp lines 140-246
%
% Switch logic (in priority order — first match wins):
%   1. If current selection is below MIN_REGULAR_CONFIDENCE and the
%      candidate is at or above it, switch.
%   2. Else if candidate's confidence > current best AND
%      candidate priority >= current best priority, switch.
%   3. Else if candidate's confidence within 1% of current best AND
%      candidate priority > current best priority, switch.
%
% On switch, the failover_count increments and the failed sensor's
% effective priority is reduced to 1 (PX4 voted_sensors_update.cpp:341).

    properties
        sensors           % cell array of Sensor handles
        validators        % cell array of DataValidator
        selected_idx      = 0
        failover_count    = 0
        failover_state    = 'none'   % 'none' | 'no_data' | 'stale' | 'timeout' | 'high_errcount' | 'high_density'
    end

    methods
        function obj = VotedSensors(sensor_list)
            obj.sensors    = sensor_list;
            n = numel(sensor_list);
            obj.validators = cell(1, n);
            for i = 1:n
                v = DataValidator();
                v.priority  = sensor_list{i}.priority;
                v.timeout_s = max(0.05, 5.0 / max(sensor_list{i}.sample_rate_hz, 1));
                obj.validators{i} = v;
            end
            % Initial selection: highest priority.
            [~, idx] = max(cellfun(@(s) s.priority, sensor_list));
            obj.selected_idx = idx;
        end

        function step(obj, t, ground_truth)
            for i = 1:numel(obj.sensors)
                obj.sensors{i}.step(t, ground_truth);
                if obj.sensors{i}.newSampleAvailable()
                    s = obj.sensors{i}.latest();
                    obj.validators{i}.put(t, s, 0);
                end
            end
            obj.updateSelection(t);
        end

        function updateSelection(obj, t)
            best_conf = -inf;
            best_prio = -inf;
            best_idx  = obj.selected_idx;

            % Confidence + priority of current selection.
            cur_conf = 0.0; cur_prio = 0;
            if obj.selected_idx > 0
                cur_conf = obj.validators{obj.selected_idx}.confidence(t);
                cur_prio = obj.validators{obj.selected_idx}.priority;
            end

            for i = 1:numel(obj.sensors)
                ci = obj.validators{i}.confidence(t);
                pi = obj.validators{i}.priority;
                if ci > best_conf || ...
                   (abs(ci - best_conf) < 0.01 && pi > best_prio)
                    best_conf = ci;
                    best_prio = pi;
                    best_idx  = i;
                end
            end

            new_idx = obj.selected_idx;
            % Rule 1: current is below threshold, candidate at/above.
            if cur_conf < DataValidator.MIN_REGULAR_CONFIDENCE && ...
               best_conf >= DataValidator.MIN_REGULAR_CONFIDENCE
                new_idx = best_idx;
            % Rule 2: strictly higher confidence, priority not lower.
            elseif best_conf > cur_conf && best_prio >= cur_prio
                new_idx = best_idx;
            % Rule 3: tie on confidence, candidate priority higher.
            elseif abs(best_conf - cur_conf) < 0.01 && best_prio > cur_prio
                new_idx = best_idx;
            end

            if new_idx ~= obj.selected_idx
                obj.failover_count = obj.failover_count + 1;
                obj.failover_state = obj.classifyFailure(obj.selected_idx, t);
                % Demote the failed sensor's effective priority to 1.
                if obj.selected_idx > 0
                    obj.validators{obj.selected_idx}.priority = 1;
                end
                obj.selected_idx = new_idx;
            end
        end

        function s = primary(obj)
            % Returns the latest sample from the currently selected sensor,
            % or [] if none has been published yet.
            if obj.selected_idx <= 0, s = []; return; end
            s = obj.sensors{obj.selected_idx}.latest();
        end

        function id = primaryDeviceId(obj)
            if obj.selected_idx <= 0, id = uint32(0); return; end
            id = obj.sensors{obj.selected_idx}.device_id;
        end
    end

    methods (Access = private)
        function tag = classifyFailure(obj, idx, t)
            if idx <= 0
                tag = 'no_data'; return;
            end
            v = obj.validators{idx};
            if t - v.last_t > v.timeout_s
                tag = 'timeout';
            elseif v.stale_count > 100
                tag = 'stale';
            elseif v.error_density >= DataValidator.ERROR_DENSITY_WINDOW * 0.9
                tag = 'high_density';
            else
                tag = 'low_confidence';
            end
        end
    end
end
