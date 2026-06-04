classdef McAutotuneAttitudeControl < handle
% Multicopter attitude/rate auto-tuner. Direct translation of
%   src/modules/mc_autotune_attitude_control/mc_autotune_attitude_control.cpp
%   src/modules/mc_autotune_attitude_control/mc_autotune_attitude_control.hpp
% (PX4, M. Bresciani)
%
% The tuner injects a decreasing-period square-wave rate setpoint on one
% body axis at a time, records the resulting (torque setpoint -> body
% rate) response, runs an online ARX recursive-least-squares
% identification (SystemIdentification), and designs rate-loop PID gains
% from the identified model using the GMVC law (pid_design_gmvc). The
% attitude P gain follows the PX4 empirical rule "a 60 deg error should
% produce the maximum control output".
%
% Differences from the PX4 work-queue module (documented deviations):
%   * uORB plumbing is replaced by a single step() call per rate-loop
%     tick, fed dt, the torque setpoint, and the body angular velocity
%     (matching the project's direct-struct-passing convention).
%   * checkFilters() configures the filters once on the first tick using
%     the supplied (constant) dt instead of averaging 1000 live sample
%     intervals; the filter-design math is otherwise identical.
%   * Parameters are not committed to flash; the resulting gains are
%     exposed via getResults() / getTuned() in both standard and PX4
%     parallel form.
%
% Usage (per axis, see sim/run_autotune.m):
%   at = McAutotuneAttitudeControl(p);
%   at.start();
%   ... each rate-loop tick:
%       inj    = at.injection();                 % add to attitude rate_sp
%       torque = rate_ctl.update(omega, rate_sp + inj, ...);
%       ... allocate, step plant ...
%       at.step(dt, torque, omega, armed, manual_rp);
%   ... until at.isDone()

    properties (Constant)
        STATE_IDLE            = 0
        STATE_INIT            = 1
        STATE_ROLL            = 2
        STATE_ROLL_PAUSE      = 3
        STATE_PITCH           = 4
        STATE_PITCH_PAUSE     = 5
        STATE_YAW             = 6
        STATE_YAW_PAUSE       = 7
        STATE_VERIFICATION    = 8
        STATE_APPLY           = 9
        STATE_TEST            = 10
        STATE_COMPLETE        = 11
        STATE_FAIL            = 12
        STATE_WAIT_FOR_DISARM = 13

        model_dt_min = 2e-3     % 500 Hz
        model_dt_max = 10e-3    % 100 Hz
        publishing_dt_s = 100e-3
        converged_thr = 50      % variance convergence threshold
    end

    properties
        sys_id                  % SystemIdentification

        % ---- configuration (MC_AT_* / IMU_GYRO_CUTOFF) ----
        sysid_amp   = 0.7       % MC_AT_SYSID_AMP
        rise_time   = 0.14      % MC_AT_RISE_TIME [s]
        apply_mode  = 0         % MC_AT_APPLY (0=log only,1=after disarm,2=in air)
        gyro_cutoff = 40.0      % IMU_GYRO_CUTOFF [Hz]

        % ---- nominal rate-loop gains (for input_scale + backup) ----
        nom_rate_p              % 3x1 MC_*RATE_P
        nom_rate_k              % 3x1 MC_*RATE_K
        nom_rate_i              % 3x1 MC_*RATE_I
        nom_rate_d              % 3x1 MC_*RATE_D
        nom_att_p               % 3x1 MC_*_P

        % ---- state machine ----
        state = 0               % numeric STATE_* (starts STATE_IDLE)
        now_s = 0
        state_start_time = 0
        start_requested = false
        done = false
        steps_counter = 5
        max_steps = 5
        signal_sign = 0
        armed = true
        manual = [0; 0]         % [roll; pitch] normalized stick
        control_power = [0; 0; 0]

        % ---- identification / design products ----
        kid = [0; 0; 0]         % [kc; ki; kd] for the current axis
        attitude_p = 0
        rate_k = [0; 0; 0]      % per-axis kc  (standard form)
        rate_i = [0; 0; 0]      % per-axis ki
        rate_d = [0; 0; 0]      % per-axis kd
        att_p  = [0; 0; 0]      % per-axis attitude P
        id_coeff = zeros(5, 3)  % per-axis identified [a1;a2;b0;b1;b2] (scaled)
        last_coeff = zeros(5, 1)
        last_model_dt = 0
        input_scale = 1.0
        gains_backup_available = false
        gains_backup = struct()
        tuned = struct()        % saved gains in PX4 parallel form

        % ---- filter configuration / timing ----
        filters_configured = false
        filter_dt = 0.01
        model_update_scaler = 1
        model_update_counter = 0
        last_publish = 0
        last_model_update = 0
        signal_filter           % AlphaFilter wash-out

        rate_sp = [0; 0; 0]     % current injected excitation (held)

        % ---- optional logging for plots ----
        log_enable = false
        hist = struct('t', [], 'coeff', [], 'var', [], 'fitness', [], ...
                      'innov', [], 'kc', [], 'ki', [], 'kd', [], ...
                      'att_p', [], 'rate_sp', [], 'state', [], ...
                      'u_filt', [], 'y_filt', [])
    end

    methods
        function obj = McAutotuneAttitudeControl(p, opts)
            if nargin < 2, opts = struct(); end
            obj.sys_id = SystemIdentification();
            obj.signal_filter = AlphaFilter();

            % Nominal gains from the project param set. Note px4_params
            % stores effective P/I/D already multiplied by K with K = 1,
            % which is the standard parallel form (MC_*RATE_K = 1).
            obj.nom_rate_p = p.rate.gain_p(:);
            obj.nom_rate_k = p.rate.gain_k(:);
            obj.nom_rate_i = p.rate.gain_i(:);
            obj.nom_rate_d = p.rate.gain_d(:);
            obj.nom_att_p  = p.att.gain_p(:);

            % Optional overrides.
            if isfield(opts, 'sysid_amp'),   obj.sysid_amp   = opts.sysid_amp;   end
            if isfield(opts, 'rise_time'),   obj.rise_time   = opts.rise_time;   end
            if isfield(opts, 'apply_mode'),  obj.apply_mode  = opts.apply_mode;  end
            if isfield(opts, 'gyro_cutoff'), obj.gyro_cutoff = opts.gyro_cutoff; end
            if isfield(opts, 'log_enable'),  obj.log_enable  = opts.log_enable;  end

            % mc_autotune_attitude_control.cpp init(): wash-out filter runs
            % in the slow publishing loop.
            obj.signal_filter.setParameters(obj.publishing_dt_s, 0.2);

            obj.state = obj.STATE_IDLE;
        end

        function start(obj)
        % Request the identification sequence (mirrors MC_AT_START = 1).
            obj.start_requested = true;
            obj.done = false;
        end

        function inj = injection(obj)
        % Excitation rate setpoint to be ADDED to the attitude controller
        % output during ROLL/PITCH/YAW/TEST (mc_att_control_main.cpp:354).
            inj = obj.rate_sp;
        end

        function tf = isDone(obj)
            tf = obj.done;
        end

        function s = stateName(obj)
            names = {'idle','init','roll','roll_pause','pitch','pitch_pause', ...
                     'yaw','yaw_pause','verification','apply','test', ...
                     'complete','fail','wait_for_disarm'};
            s = names{obj.state + 1};
        end

        function inj = step(obj, dt, torque, omega, armed, manual_rp)
        % One rate-loop tick. Mirrors mc_autotune_attitude_control.cpp Run().
            if nargin < 5, armed = true; end
            if nargin < 6, manual_rp = [0; 0]; end

            obj.now_s = obj.now_s + dt;
            obj.armed = armed;
            obj.manual = manual_rp(:);

            % When idle, only watch for the start request.
            if obj.state == obj.STATE_IDLE
                obj.updateStateMachine();
                inj = obj.rate_sp;
                return;
            end

            % Configure the filters once, using the known dt.
            obj.checkFilters(dt);

            % Feed the identification filters at full rate on the active axis.
            switch obj.state
                case obj.STATE_ROLL
                    obj.sys_id.updateFilters(obj.input_scale * torque(1), omega(1));
                case obj.STATE_PITCH
                    obj.sys_id.updateFilters(obj.input_scale * torque(2), omega(2));
                case obj.STATE_YAW
                    obj.sys_id.updateFilters(obj.input_scale * torque(3), omega(3));
            end

            % Update the RLS model at the (lower) model-update rate.
            obj.model_update_counter = obj.model_update_counter + 1;
            if obj.model_update_counter >= obj.model_update_scaler
                if any(obj.state == [obj.STATE_ROLL, obj.STATE_PITCH, obj.STATE_YAW])
                    obj.sys_id.update();            % model only
                    obj.last_model_update = obj.now_s;
                end
                obj.model_update_counter = 0;
            end

            % Slow publishing loop: state machine + gain design + excitation.
            if (obj.now_s - obj.last_publish) > obj.publishing_dt_s || obj.last_publish == 0
                obj.updateStateMachine();           % uses last cycle's gains
                obj.computeGains();                 % refresh kid / attitude_p

                if obj.sys_id.areFiltersInitialized()
                    obj.rate_sp = obj.getIdentificationSignal();
                else
                    obj.rate_sp = [0; 0; 0];
                end

                obj.recordStatus();
                obj.last_publish = obj.now_s;
            end

            inj = obj.rate_sp;
        end

        function r = getResults(obj)
        % Identification + design summary for all three axes.
            r.state      = obj.stateName();
            r.state_num  = obj.state;
            r.success    = obj.areGainsGood();
            r.rate_k     = obj.rate_k;     % standard-form kc per axis
            r.rate_i     = obj.rate_i;     % ki = 1/Ti
            r.rate_d     = obj.rate_d;     % kd = Td
            r.att_p      = obj.att_p;
            r.id_coeff   = obj.id_coeff;   % [a1;a2;b0;b1;b2] per axis (scaled)
            r.nom_rate_p = obj.nom_rate_p;
            r.nom_rate_i = obj.nom_rate_i;
            r.nom_rate_d = obj.nom_rate_d;
            r.nom_att_p  = obj.nom_att_p;
            r.tuned      = obj.getTuned();
        end

        function t = getTuned(obj)
        % New gains in PX4 parallel form (what saveGainsToParams would set).
        % MC_*RATE_K is set to 1 and P absorbs the standard-form kc.
            t.MC_ROLLRATE_P  = obj.rate_k(1);
            t.MC_ROLLRATE_K  = 1.0;
            t.MC_ROLLRATE_I  = obj.rate_k(1) * obj.rate_i(1);
            t.MC_ROLLRATE_D  = obj.rate_k(1) * obj.rate_d(1);
            t.MC_ROLL_P      = obj.att_p(1);
            t.MC_PITCHRATE_P = obj.rate_k(2);
            t.MC_PITCHRATE_K = 1.0;
            t.MC_PITCHRATE_I = obj.rate_k(2) * obj.rate_i(2);
            t.MC_PITCHRATE_D = obj.rate_k(2) * obj.rate_d(2);
            t.MC_PITCH_P     = obj.att_p(2);
            t.MC_YAWRATE_P   = obj.rate_k(3);
            t.MC_YAWRATE_K   = 1.0;
            t.MC_YAWRATE_I   = obj.rate_k(3) * obj.rate_i(3);
            t.MC_YAWRATE_D   = obj.rate_k(3) * obj.rate_d(3);
            t.MC_YAW_P       = obj.att_p(3);
        end
    end

    methods (Access = private)
        function checkFilters(obj, dt)
        % mc_autotune_attitude_control.cpp: checkFilters(). Configured once
        % here (constant dt) rather than from a 1000-sample running average.
            if obj.filters_configured
                return;
            end

            obj.filter_dt = dt;
            filter_rate_hz = 1 / obj.filter_dt;

            obj.sys_id.setLpfCutoffFrequency(filter_rate_hz, obj.gyro_cutoff);
            obj.sys_id.setHpfCutoffFrequency(filter_rate_hz, 0.5);

            % Model sampling time from the gyro cutoff (a proxy for the max
            % usable control bandwidth), clamped to [2ms, 10ms].
            model_dt = min(max(max(1 / (2 * obj.gyro_cutoff), obj.filter_dt), ...
                               obj.model_dt_min), obj.model_dt_max);
            obj.model_update_scaler = max(floor(model_dt / obj.filter_dt), 1);
            model_dt = obj.model_update_scaler * obj.filter_dt;

            obj.sys_id.setForgettingFactor(60, model_dt);
            obj.sys_id.setFitnessLpfTimeConstant(1, model_dt);

            obj.filters_configured = true;
        end

        function computeGains(obj)
        % mc_autotune_attitude_control.cpp Run(): GMVC gain design.
            coeff = obj.sys_id.getCoefficients();     % [a1;a2;b0;b1;b2]
            coeff(3) = coeff(3) * obj.input_scale;    % b0
            coeff(4) = coeff(4) * obj.input_scale;    % b1
            coeff(5) = coeff(5) * obj.input_scale;    % b2

            num = [coeff(3); coeff(4); coeff(5)];
            den = [1; coeff(1); coeff(2)];

            model_dt = obj.model_update_scaler * obj.filter_dt;

            if any(obj.state == [obj.STATE_YAW, obj.STATE_YAW_PAUSE])
                desired_rise_time = 0.2;
            else
                desired_rise_time = obj.rise_time;
            end

            obj.kid = pid_design_gmvc(num, den, model_dt, desired_rise_time, 0.0, 0.7);

            % Prevent the D term from going just negative if not needed.
            if (obj.kid(3) < 0) && (obj.kid(3) > -0.001)
                obj.kid(3) = 0;
            end

            % "An error of 60 deg should produce the maximum control output":
            % K_att * K_rate * rad(60) = 1.
            obj.attitude_p = min(max(1 / (deg2rad(60) * obj.kid(1)), 2), 6.5);

            obj.last_coeff = coeff;
            obj.last_model_dt = model_dt;
        end

        function updateStateMachine(obj)
        % mc_autotune_attitude_control.cpp: updateStateMachine(now)
            now = obj.now_s;

            switch obj.state
                case obj.STATE_IDLE
                    if obj.start_requested
                        obj.state = obj.STATE_INIT;     % actuator cb always OK here
                        obj.state_start_time = now;
                    end

                case obj.STATE_INIT
                    if obj.filters_configured
                        obj.state = obj.STATE_ROLL;
                        obj.state_start_time = now;
                        obj.sys_id.reset();
                        obj.steps_counter = 5;
                        obj.max_steps = 10;
                        obj.signal_sign = 1;
                        obj.input_scale = 1 / (obj.nom_rate_p(1) * obj.nom_rate_k(1));
                        obj.signal_filter.reset(0);
                        obj.gains_backup_available = false;
                    end

                case obj.STATE_ROLL
                    if obj.areAllSmallerThan(obj.sys_id.getVariances(), obj.converged_thr) ...
                            && ((now - obj.state_start_time) > 5)
                        obj.copyGains(1);
                        obj.state = obj.STATE_ROLL_PAUSE;
                        obj.state_start_time = now;
                    end

                case obj.STATE_ROLL_PAUSE
                    if (now - obj.state_start_time) > 2
                        obj.state = obj.STATE_PITCH;
                        obj.state_start_time = now;
                        obj.sys_id.reset();
                        obj.input_scale = 1 / (obj.nom_rate_p(2) * obj.nom_rate_k(2));
                        obj.signal_filter.reset(0);
                        obj.signal_sign = 1;
                        obj.steps_counter = 5;
                        obj.max_steps = 10;
                    end

                case obj.STATE_PITCH
                    if obj.areAllSmallerThan(obj.sys_id.getVariances(), obj.converged_thr) ...
                            && ((now - obj.state_start_time) > 5)
                        obj.copyGains(2);
                        obj.state = obj.STATE_PITCH_PAUSE;
                        obj.state_start_time = now;
                    end

                case obj.STATE_PITCH_PAUSE
                    if (now - obj.state_start_time) > 2
                        obj.state = obj.STATE_YAW;
                        obj.state_start_time = now;
                        obj.sys_id.reset();
                        obj.input_scale = 1 / (obj.nom_rate_p(3) * obj.nom_rate_k(3));
                        obj.signal_filter.reset(0);
                        obj.signal_sign = 1;
                        obj.steps_counter = 5;
                        obj.max_steps = 10;
                    end

                case obj.STATE_YAW
                    if obj.areAllSmallerThan(obj.sys_id.getVariances(), obj.converged_thr) ...
                            && ((now - obj.state_start_time) > 5)
                        obj.copyGains(3);
                        obj.state = obj.STATE_YAW_PAUSE;
                        obj.state_start_time = now;
                    end

                case obj.STATE_YAW_PAUSE
                    if (now - obj.state_start_time) > 2
                        obj.state = obj.STATE_VERIFICATION;
                        obj.state_start_time = now;
                        obj.sys_id.reset();
                        obj.signal_filter.reset(0);
                        obj.signal_sign = 1;
                        obj.steps_counter = 5;
                        obj.max_steps = 10;
                    end

                case obj.STATE_VERIFICATION
                    if obj.areGainsGood()
                        obj.state = obj.STATE_APPLY;
                    else
                        obj.state = obj.STATE_FAIL;
                    end
                    obj.state_start_time = now;

                case obj.STATE_APPLY
                    if obj.apply_mode == 1
                        obj.state = obj.STATE_WAIT_FOR_DISARM;
                    elseif obj.apply_mode == 2
                        obj.backupAndSaveGainsToParams();
                        obj.state = obj.STATE_TEST;
                    else
                        obj.state = obj.STATE_COMPLETE;
                    end
                    obj.state_start_time = now;

                case obj.STATE_WAIT_FOR_DISARM
                    if ~obj.armed
                        obj.saveGainsToParams();
                        obj.state = obj.STATE_COMPLETE;
                        obj.state_start_time = now;
                    end

                case obj.STATE_TEST
                    if (now - obj.state_start_time) > 4
                        obj.state = obj.STATE_COMPLETE;
                        obj.state_start_time = now;
                    elseif ((now - obj.state_start_time) < 4) ...
                            && ((now - obj.state_start_time) > 1) ...
                            && (norm(obj.control_power) > 0.1)
                        obj.state = obj.STATE_FAIL;
                        obj.revertParamGains();
                        obj.state_start_time = now;
                    end

                case {obj.STATE_COMPLETE, obj.STATE_FAIL}
                    % Linger briefly so consumers see the final result.
                    if (now - obj.state_start_time) > 2
                        obj.state = obj.STATE_IDLE;
                        obj.stopAutotune();
                    end
            end

            % Pilot intervention or convergence timeout aborts immediately.
            if (obj.state ~= obj.STATE_WAIT_FOR_DISARM) ...
                    && (obj.state ~= obj.STATE_IDLE) ...
                    && (((now - obj.state_start_time) > 20) ...
                        || (abs(obj.manual(1)) > 0.05) ...
                        || (abs(obj.manual(2)) > 0.05))
                obj.state = obj.STATE_FAIL;
                obj.state_start_time = now;
            end
        end

        function tf = areAllSmallerThan(~, vect, threshold)
            tf = all(vect < threshold);
        end

        function copyGains(obj, index)
        % mc_autotune_attitude_control.cpp: copyGains()
            if index <= 3
                obj.rate_k(index) = obj.kid(1);
                obj.rate_i(index) = obj.kid(2);
                obj.rate_d(index) = obj.kid(3);
                obj.att_p(index)  = obj.attitude_p;
                obj.id_coeff(:, index) = obj.last_coeff;
            end
        end

        function tf = areGainsGood(obj)
        % mc_autotune_attitude_control.cpp: areGainsGood()
            are_positive = (min(obj.rate_k) > 0) ...
                && (min(obj.rate_i) > 0) ...
                && (min(obj.rate_d) >= 0) ...
                && (min(obj.att_p)  > 0);

            are_small_enough = (max(obj.rate_k) < 0.5) ...
                && (max(obj.rate_i) < 10) ...
                && (max(obj.rate_d) < 0.1) ...
                && (max(obj.att_p)  < 12);

            tf = are_positive && are_small_enough;
        end

        function saveGainsToParams(obj)
        % mc_autotune_attitude_control.cpp: saveGainsToParams(). Stores the
        % new gains (PX4 parallel form) into obj.tuned.
            obj.tuned = obj.getTuned();
        end

        function backupAndSaveGainsToParams(obj)
        % mc_autotune_attitude_control.cpp: backupAndSaveGainsToParams().
            obj.gains_backup.rate_k = obj.rate_k;
            obj.gains_backup.rate_i = obj.rate_i;
            obj.gains_backup.rate_d = obj.rate_d;
            obj.gains_backup.att_p  = obj.att_p;
            obj.saveGainsToParams();
            obj.gains_backup_available = true;
        end

        function revertParamGains(obj)
            if obj.gains_backup_available
                obj.rate_k = obj.gains_backup.rate_k;
                obj.rate_i = obj.gains_backup.rate_i;
                obj.rate_d = obj.gains_backup.rate_d;
                obj.att_p  = obj.gains_backup.att_p;
                obj.saveGainsToParams();
            end
        end

        function stopAutotune(obj)
            obj.start_requested = false;
            obj.done = true;
        end

        function rate_sp = getIdentificationSignal(obj)
        % mc_autotune_attitude_control.cpp: getIdentificationSignal().
        % Decreasing-period square wave with a DC wash-out, so the drone
        % stays roughly centred while a range of frequencies is excited.
            if obj.steps_counter > obj.max_steps
                if obj.signal_sign == 1
                    obj.signal_sign = 0;
                else
                    obj.signal_sign = 1;
                end
                obj.steps_counter = 0;

                if obj.max_steps > 1
                    obj.max_steps = obj.max_steps - 1;
                else
                    obj.max_steps = 5;
                end
            end

            obj.steps_counter = obj.steps_counter + 1;

            step = double(obj.signal_sign) * obj.sysid_amp;

            rate_sp = [0; 0; 0];
            signal = step - obj.signal_filter.getState();   % wash-out

            switch obj.state
                case obj.STATE_ROLL,  rate_sp(1) = signal;
                case obj.STATE_PITCH, rate_sp(2) = signal;
                case obj.STATE_YAW,   rate_sp(3) = signal;
                case obj.STATE_TEST,  rate_sp(1) = signal; rate_sp(2) = signal;
            end

            obj.signal_filter.update(step);
        end

        function recordStatus(obj)
            if ~obj.log_enable
                return;
            end
            obj.hist.t(end + 1)        = obj.now_s;
            obj.hist.coeff(:, end + 1) = obj.last_coeff;
            obj.hist.var(:, end + 1)   = obj.sys_id.getVariances();
            obj.hist.fitness(end + 1)  = obj.sys_id.getFitness();
            obj.hist.innov(end + 1)    = obj.sys_id.getInnovation();
            obj.hist.kc(end + 1)       = obj.kid(1);
            obj.hist.ki(end + 1)       = obj.kid(2);
            obj.hist.kd(end + 1)       = obj.kid(3);
            obj.hist.att_p(end + 1)    = obj.attitude_p;
            obj.hist.rate_sp(:, end+1) = obj.rate_sp;
            obj.hist.state(end + 1)    = obj.state;
            obj.hist.u_filt(end + 1)   = obj.sys_id.getFilteredInputData();
            obj.hist.y_filt(end + 1)   = obj.sys_id.getFilteredOutputData();
        end
    end
end
