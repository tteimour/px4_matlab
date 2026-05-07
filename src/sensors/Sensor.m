classdef Sensor < handle
% Sensor — abstract base for all sensor chip models.
%
% A sensor wraps a chip-accurate measurement model that runs at its
% own native rate (independent of the sim step) and emits sample
% structs with measurement-to-output latency.
%
% Lifecycle:
%   1. Owner constructs the sensor with rate, latency, priority, instance.
%   2. Each sim step calls step(t, ground_truth). The sensor:
%        - generates 0..N new samples whose scheduled sample time falls
%          in (last_t, t]
%        - pushes each into a delay queue (publish_t = sample_t + latency)
%        - releases samples whose publish_t has elapsed; the most recent
%          released sample becomes available via latest()
%   3. Voter / consumer calls latest() to get the current published sample
%      or check newSampleAvailable() to detect a newly released sample.
%
% Subclass contract:
%   - implement measure(t_sample, ground_truth) -> sample struct.
%   - sample MUST contain field 't' (scheduled sample timestamp) and
%     'instance' (sensor instance id). Other fields are sensor-type
%     specific (see Imu/Baro/Mag/GnssSensor).
%
% Mirrors the role of a single PX4 sensor driver (e.g. ICM45686 driver
% feeding vehicle_imu_s). The voter / vehicle_*  layer is built on top
% in src/sensors/voter/.

    properties
        sample_rate_hz   % chip native ODR
        latency_s        % measurement-to-output latency (sensor + bus + driver)
        priority         % CAL_*_PRIO equivalent (1..100, higher = preferred)
        instance         % stable instance id for voter
        healthy          % logical — chip is producing valid data
        rng              % per-sensor RandStream for repeatability
        device_id        % uint32 stand-in for PX4 device_id (chip+bus encoding)
    end

    properties (Access = protected)
        sample_period_   % 1/sample_rate_hz
        next_sample_t_   % time of next due sample (s)
        delay_queue_     % FIFO of pending {publish_t, sample}
        latest_          % most recently released sample (post-latency)
        new_sample_      % logical — set true when a fresh sample is released
    end

    methods
        function obj = Sensor(rate_hz, latency_s, priority, instance, ...
                              device_id, seed)
            obj.sample_rate_hz = rate_hz;
            obj.latency_s      = latency_s;
            obj.priority       = priority;
            obj.instance       = instance;
            obj.device_id      = uint32(device_id);
            obj.healthy        = true;
            obj.sample_period_ = 1.0 / rate_hz;
            obj.next_sample_t_ = 0.0;
            obj.delay_queue_   = repmat(struct('publish_t', 0, 'sample', []), 0, 1);
            obj.latest_        = [];
            obj.new_sample_    = false;
            if nargin < 6 || isempty(seed)
                seed = 1000 + uint32(instance) + uint32(mod(device_id, 1000));
            end
            obj.rng = RandStream('mt19937ar', 'Seed', seed);
        end

        function step(obj, t, ground_truth)
            % Advance the sensor to time t given current ground truth.
            % Generates due samples and releases delayed ones.
            obj.new_sample_ = false;

            % Generate every sample whose scheduled time has arrived.
            while t >= obj.next_sample_t_ - 1e-12
                t_sample = obj.next_sample_t_;
                s = obj.measure(t_sample, ground_truth);
                if ~isfield(s, 't'),        s.t = t_sample;     end
                if ~isfield(s, 'instance'), s.instance = obj.instance; end
                if ~isfield(s, 'device_id'),s.device_id = obj.device_id; end
                obj.delay_queue_(end+1, 1) = struct( ...
                    'publish_t', t_sample + obj.latency_s, ...
                    'sample',    s);
                obj.next_sample_t_ = obj.next_sample_t_ + obj.sample_period_;
            end

            % Release any sample whose publish_t has elapsed.
            while ~isempty(obj.delay_queue_) && t >= obj.delay_queue_(1).publish_t
                obj.latest_     = obj.delay_queue_(1).sample;
                obj.new_sample_ = true;
                obj.delay_queue_(1) = [];
            end
        end

        function tf = newSampleAvailable(obj)
            tf = obj.new_sample_;
        end

        function s = latest(obj)
            s = obj.latest_;
        end

        function reset(obj)
            obj.next_sample_t_ = 0.0;
            obj.delay_queue_   = repmat(struct('publish_t', 0, 'sample', []), 0, 1);
            obj.latest_        = [];
            obj.new_sample_    = false;
            obj.healthy        = true;
        end
    end

    methods (Abstract, Access = protected)
        s = measure(obj, t_sample, ground_truth)
    end
end
