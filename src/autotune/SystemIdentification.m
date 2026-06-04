classdef SystemIdentification < handle
% Single-axis online system identification. Direct translation of
%   src/lib/system_identification/system_identification.{hpp,cpp}  (PX4)
%
% Pipeline for each (u, y) = (torque setpoint, body rate) sample:
%   u, y --> 2nd-order low-pass (anti-alias / de-noise)
%        --> first-order high-pass (remove bias / DC drift)
%        --> ArxRls<2,2,1> recursive least-squares
% A separate AlphaFilter low-passes the "fitness" metric (rate of change
% of the estimate), used as a convergence indicator.
%
% The identified discrete-time plant model is
%   G(q^-1) = (b_0 + b_1 q^-1 + b_2 q^-2) / (1 + a_1 q^-1 + a_2 q^-2)
% with coefficients returned by getCoefficients() as [a_1 a_2 b_0 b_1 b_2]'.

    properties
        rls                          % ArxRls<2,2,1>
        u_lpf                        % LowPassFilter2p on the input
        y_lpf                        % LowPassFilter2p on the output
        alpha_hpf = 0                % high-pass filter coefficient
        u_hpf = 0
        y_hpf = 0
        u_prev = 0
        y_prev = 0
        are_filters_initialized = false
        fitness_lpf                  % AlphaFilter on the fitness metric
        dt = 0.1
    end

    methods
        function obj = SystemIdentification()
            obj.rls = ArxRls(2, 2, 1);
            obj.u_lpf = LowPassFilter2p(400, 30);   % PX4 defaults
            obj.y_lpf = LowPassFilter2p(400, 30);
            obj.fitness_lpf = AlphaFilter();
        end

        function reset(obj, id_state_init)
        % system_identification.cpp: reset()
            if nargin < 2
                id_state_init = [];
            end
            obj.rls.reset(id_state_init);
            obj.u_lpf.reset(0);
            obj.y_lpf.reset(0);   % NB: PX4 resets u_lpf twice (typo) and
                                  % never y_lpf; harmless because
                                  % are_filters_initialized=false forces a
                                  % re-init on the next sample. We reset
                                  % both for clarity.
            obj.u_hpf = 0;
            obj.y_hpf = 0;
            obj.u_prev = 0;
            obj.y_prev = 0;
            obj.fitness_lpf.reset(10);
            obj.are_filters_initialized = false;
        end

        function update(obj, u, y)
        % Two overloads matching PX4:
        %   update(u, y) : update filters then the model    (nargin == 3)
        %   update()     : update model only (filters already advanced)
            if nargin == 3
                obj.updateFilters(u, y);
            end
            obj.rls.update(obj.u_hpf, obj.y_hpf);
            obj.updateFitness();
        end

        function updateFilters(obj, u, y)
        % system_identification.cpp: updateFilters()
            if ~obj.are_filters_initialized
                obj.u_lpf.reset(u);
                obj.y_lpf.reset(y);
                obj.u_hpf = 0;
                obj.y_hpf = 0;
                obj.u_prev = u;
                obj.y_prev = y;
                obj.are_filters_initialized = true;
                return;
            end

            u_lpf_v = obj.u_lpf.apply(u);
            y_lpf_v = obj.y_lpf.apply(y);
            obj.u_hpf = obj.alpha_hpf * obj.u_hpf + obj.alpha_hpf * (u_lpf_v - obj.u_prev);
            obj.y_hpf = obj.alpha_hpf * obj.y_hpf + obj.alpha_hpf * (y_lpf_v - obj.y_prev);

            obj.u_prev = u_lpf_v;
            obj.y_prev = y_lpf_v;
        end

        function updateFitness(obj)
        % system_identification.cpp: updateFitness(). Fitness = low-passed
        % rate of change of the parameter estimate (smaller = converged).
            diff = obj.rls.getDiffEstimate();
            s = sum(diff);
            if obj.dt > eps('single')
                obj.fitness_lpf.update(s / obj.dt);
            end
        end

        function tf = areFiltersInitialized(obj)
            tf = obj.are_filters_initialized;
        end

        function c = getCoefficients(obj)
            c = obj.rls.getCoefficients();
        end

        function v = getVariances(obj)
            v = obj.rls.getVariances();
        end

        function d = getDiffEstimate(obj)
            d = obj.rls.getDiffEstimate();
        end

        function f = getFitness(obj)
            f = obj.fitness_lpf.getState();
        end

        function inn = getInnovation(obj)
            inn = obj.rls.getInnovation();
        end

        function u = getFilteredInputData(obj)
            u = obj.u_hpf;
        end

        function y = getFilteredOutputData(obj)
            y = obj.y_hpf;
        end

        function setLpfCutoffFrequency(obj, sample_freq, cutoff)
            obj.u_lpf.set_cutoff_frequency(sample_freq, cutoff);
            obj.y_lpf.set_cutoff_frequency(sample_freq, cutoff);
        end

        function setHpfCutoffFrequency(obj, sample_freq, cutoff)
            obj.alpha_hpf = sample_freq / (sample_freq + 2 * pi * cutoff);
        end

        function setForgettingFactor(obj, time_constant, dt)
            obj.rls.setForgettingFactor(time_constant, dt);
        end

        function setFitnessLpfTimeConstant(obj, time_constant, dt)
            obj.fitness_lpf.setParameters(dt, time_constant);
            obj.dt = dt;
        end
    end
end
