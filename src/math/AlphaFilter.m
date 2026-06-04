classdef AlphaFilter < handle
% First-order IIR ("alpha") low-pass filter, a.k.a. leaky integrator /
% forgetting average. Scalar specialization of
%   src/lib/mathlib/math/filter/AlphaFilter.hpp  (PX4)
%
% The autotuner uses it twice:
%   - as the fitness low-pass in SystemIdentification
%   - as the wash-out filter that removes the DC term from the injected
%     square-wave excitation in McAutotuneAttitudeControl

    properties
        time_constant = 0
        alpha = 0
        filter_state = 0
    end

    methods
        function obj = AlphaFilter(varargin)
        % AlphaFilter(), AlphaFilter(time_constant), or
        % AlphaFilter(sample_interval, time_constant).
            if nargin == 1
                obj.time_constant = varargin{1};
            elseif nargin == 2
                obj.setParameters(varargin{1}, varargin{2});
            end
        end

        function setParameters(obj, sample_interval, time_constant)
        % AlphaFilter.hpp: setParameters(). Both args in the same units.
            denominator = time_constant + sample_interval;
            if denominator > eps('single')
                obj.setAlpha(sample_interval / denominator);
            end
            obj.time_constant = time_constant;
        end

        function setAlpha(obj, alpha)
            obj.alpha = alpha;
        end

        function reset(obj, sample)
            obj.filter_state = sample;
        end

        function s = update(obj, sample)
        % AlphaFilter.hpp: updateCalculation() (scalar form).
            obj.filter_state = obj.filter_state + obj.alpha * (sample - obj.filter_state);
            s = obj.filter_state;
        end

        function s = getState(obj)
            s = obj.filter_state;
        end
    end
end
