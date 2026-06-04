classdef ArxRls < handle
% Efficient recursive weighted least-squares (RLS) estimator for an ARX
% model, without matrix inversion. Direct translation of
%   src/lib/system_identification/arx_rls.hpp  (PX4, M. Bresciani)
%
% Model:
%   A(q^-1) y(k) = q^-d B(q^-1) u(k) + A(q^-1) e(k)
% with
%   A(q^-1) = 1 + a_1 q^-1 + ... + a_N q^-N
%   B(q^-1) = b_0 + b_1 q^-1 + ... + b_M q^-M
%   d       = input/output delay
% Parameter vector (the RLS state):
%   theta = [a_1 .. a_N  b_0 .. b_M]'   (length N+M+1)
%
% The attitude autotuner instantiates this as ArxRls(2, 2, 1).
%
% Reference: Identification de systemes dynamiques, D. Bonvin and
% A. Karimi, EPFL, 2011.

    properties
        N                       % order of A(q^-1)
        M                       % order of B(q^-1)
        D                       % delay d
        n_theta                 % N + M + 1
        P                       % covariance matrix (n_theta x n_theta)
        theta_hat               % parameter estimate (n_theta x 1)
        diff_theta_hat          % |theta_hat(k) - theta_hat(k-1)| (n_theta x 1)
        innovation = 0
        u_buf                   % input shift register  (length M+D+1)
        y_buf                   % output shift register (length N+1)
        nb_samples = 0
        lambda = 1              % forgetting factor
    end

    methods
        function obj = ArxRls(N, M, D)
            if nargin < 3
                N = 2; M = 2; D = 1;   % attitude autotuner default
            end
            assert(N >= M, 'The transfer function needs to be proper (N >= M)');
            obj.N = N;
            obj.M = M;
            obj.D = D;
            obj.n_theta = N + M + 1;
            obj.reset();
        end

        function setForgettingFactor(obj, a, b)
        % setForgettingFactor(time_constant, dt) -> lambda = 1 - dt/tc
        % setForgettingFactor(lambda)            -> lambda = lambda
            if nargin == 3
                obj.lambda = 1 - b / a;
            else
                obj.lambda = a;
            end
        end

        function c = getCoefficients(obj)
            c = obj.theta_hat;            % [a_1 .. a_N b_0 .. b_M]'
        end

        function v = getVariances(obj)
            v = diag(obj.P);
        end

        function d = getDiffEstimate(obj)
            d = obj.diff_theta_hat;
        end

        function inn = getInnovation(obj)
            inn = obj.innovation;
        end

        function reset(obj, theta_init)
        % arx_rls.hpp: reset(). Covariance reset to 10e3 * I, parameters
        % to theta_init (zeros by default), shift registers cleared.
            obj.P = zeros(obj.n_theta);
            for i = 1:obj.n_theta
                obj.P(i, i) = 10e3;
            end

            obj.diff_theta_hat = zeros(obj.n_theta, 1);

            if nargin < 2 || isempty(theta_init)
                obj.theta_hat = zeros(obj.n_theta, 1);
            else
                obj.theta_hat = theta_init(:);
            end

            obj.u_buf = zeros(obj.M + obj.D + 1, 1);
            obj.y_buf = zeros(obj.N + 1, 1);
            obj.nb_samples = 0;
            obj.innovation = 0;
        end

        function update(obj, u, y)
        % arx_rls.hpp: update(). One RLS step with new input u and output y.
            theta_prev = obj.theta_hat;

            obj.addInputOutput(u, y);

            if ~obj.isBufferFull()
                % Do not start updating while the buffer still holds zeros.
                return;
            end

            phi = obj.constructDesignVector();      % n_theta x 1

            % Operator order matches arx_rls.hpp exactly:
            %   P = (P - P*phi*phi'*P / (lambda + phi'*P*phi)) / lambda
            % then innovation uses the OLD theta_hat, then theta_hat uses
            % the NEW P.
            Pphi  = obj.P * phi;
            denom = obj.lambda + (phi' * Pphi);
            obj.P = (obj.P - (Pphi * (phi' * obj.P)) / denom) / obj.lambda;

            obj.innovation = obj.y_buf(obj.N + 1) - (phi' * obj.theta_hat);
            obj.theta_hat  = obj.theta_hat + obj.P * phi * obj.innovation;

            obj.diff_theta_hat = abs(obj.theta_hat - theta_prev);
        end
    end

    methods (Access = private)
        function addInputOutput(obj, u, y)
        % arx_rls.hpp: addInputOutput()
            obj.shiftRegisters();
            obj.u_buf(obj.M + obj.D + 1) = u;       % _u[M+D] = u
            obj.y_buf(obj.N + 1) = y;               % _y[N]   = y

            if ~obj.isBufferFull()
                obj.nb_samples = obj.nb_samples + 1;
            end
        end

        function shiftRegisters(obj)
        % arx_rls.hpp: shiftRegisters() — shift both buffers one step left.
            obj.y_buf(1:obj.N)         = obj.y_buf(2:obj.N + 1);
            obj.u_buf(1:obj.M + obj.D) = obj.u_buf(2:obj.M + obj.D + 1);
        end

        function tf = isBufferFull(obj)
            tf = obj.nb_samples > (obj.M + obj.N + obj.D);
        end

        function phi = constructDesignVector(obj)
        % arx_rls.hpp: constructDesignVector()
        %   phi = [ -y(k-1) .. -y(k-N)  u(k-D) .. u(k-D-M) ]'
        % In buffer terms (newest sample at the end of each buffer):
        %   phi(1:N)     = -y_buf(N : -1 : 1)
        %   phi(N+1:end) =  u_buf(M+1 : -1 : 1)
            phi = zeros(obj.n_theta, 1);
            phi(1:obj.N)     = -obj.y_buf(obj.N:-1:1);
            phi(obj.N + 1:end) = obj.u_buf((obj.M + 1):-1:1);
        end
    end
end
