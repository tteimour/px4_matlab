function kid = pid_design_gmvc(num, den, dt, sigma, delta, lbda)
% PID gain design via Generalized Minimum Variance Control (GMVC) law.
% Direct translation of
%   src/lib/pid_design/pid_design.hpp : computePidGmvc()  (PX4)
%
% Inputs:
%   num   : numerator coefficients [b0; b1; b2] of the discrete plant
%   den   : denominator coefficients [1; a1; a2] of the discrete plant
%   dt    : sampling time of the identified model [s]
%   sigma : desired closed-loop rise time [s]      (default 0.1)
%   delta : damping index in [0,1]; 0 = critical, 1 = Butterworth (def 1)
%   lbda  : detuning coefficient; increase to detune kc only      (def 0.5)
%
% Output:
%   kid = [kc; ki; kd] in standard (series) form:
%       u = kc * (1 + ki*dt + kd/dt) * e
%   kc : controller gain, ki : integral gain (= 1/Ti), kd : derivative (Td)
%
% Reference: T. Yamamoto, K. Fujii, M. Kaneda, "Design and implementation
% of a self-tuning PID controller," 1998.

    if nargin < 4 || isempty(sigma), sigma = 0.1; end
    if nargin < 5 || isempty(delta), delta = 1.0; end
    if nargin < 6 || isempty(lbda),  lbda  = 0.5; end

    sigma = min(max(sigma, 0.01), 1.0);
    delta = min(max(delta, 0.0),  1.0);
    lbda  = min(max(lbda,  0.0),  10.0);

    a1 = den(2);
    a2 = den(3);
    b0 = num(1);
    b1 = num(2);
    b2 = num(3);

    % Solve GMVC law (see PX4 derivation in pid_synthesis_symbolic.py).
    rho = dt / sigma;
    mu  = 0.25 * (1 - delta) + 0.51 * delta;   % mu in [0.25, 0.51]
    p1  = -2 * exp(-rho / (2 * mu)) * cos(sqrt(4 * mu - 1) * rho / (2 * mu));
    p2  = exp(-rho / mu);
    e1  = -a1 + p1 + 1;
    f0  = -a1 * e1 + a1 - a2 + e1 + p2;
    f1  = a1 * e1 - a2 * e1 + a2;
    f2  = a2 * e1;

    % Translate to PID gains.
    nu = lbda + (e1 + 1) * (b0 + b1 + b2);

    if abs(nu) < eps('single')
        kid = [0; 0; 0];
        return;
    end

    kc = -(f1 + 2 * f2) / nu;
    ki = -(f0 + f1 + f2) / (dt * (f1 + 2 * f2));
    ki = ki / 5;   % not in the original paper; required for reasonable gains
    kd = -dt * f2 / (f1 + 2 * f2);

    kid = [kc; ki; kd];
end
