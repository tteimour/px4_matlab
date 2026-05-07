classdef WindModel < handle
%WINDMODEL  Steady wind plus first-order coloured-noise turbulence (NED).
%
%   Total wind = steady_ned + turbulence(t)
%
%   Turbulence is the simplified Dryden / Ornstein-Uhlenbeck process
%   used in flight-dynamics simulators: per axis, white Gaussian noise
%   driven through a first-order low-pass with correlation time tau and
%   steady-state standard deviation sigma. Discretized as
%
%       x[k+1] = (1 - dt/tau) * x[k] + sigma*sqrt(2*dt/tau) * w[k],   w~N(0,1)
%
%   so that Var(x_i) -> sigma^2 and autocorrelation decays with tau.
%
%   Reset zeros the turbulence state; the steady component is left
%   alone (it is owned by the user via the UI / params).

    properties
        steady_ned   = [0; 0; 0]    % m/s
        turb_enable  = false
        turb_sigma   = 1.0          % m/s, 1-sigma intensity per axis
        turb_tau     = 2.0          % s, correlation time
        last_wind_ned = [0; 0; 0]   % cached total wind from last update()
    end
    properties (Access = private)
        turb_state = [0; 0; 0]
    end

    methods
        function obj = WindModel(p)
            if nargin >= 1 && isstruct(p) && isfield(p, 'wind')
                w = p.wind;
                if isfield(w, 'steady'),      obj.steady_ned  = w.steady(:); end
                if isfield(w, 'turb_enable'), obj.turb_enable = logical(w.turb_enable); end
                if isfield(w, 'turb_sigma'),  obj.turb_sigma  = w.turb_sigma; end
                if isfield(w, 'turb_tau'),    obj.turb_tau    = w.turb_tau; end
            end
            obj.last_wind_ned = obj.steady_ned;
        end

        function reset(obj)
            obj.turb_state    = [0; 0; 0];
            obj.last_wind_ned = obj.steady_ned;
        end

        function w = update(obj, dt)
            if obj.turb_enable && obj.turb_tau > 0 && obj.turb_sigma > 0
                a = max(0, 1 - dt / obj.turb_tau);
                b = obj.turb_sigma * sqrt(2 * dt / obj.turb_tau);
                obj.turb_state = a * obj.turb_state + b * randn(3, 1);
            else
                obj.turb_state = [0; 0; 0];
            end
            w = obj.steady_ned + obj.turb_state;
            obj.last_wind_ned = w;
        end
    end
end
