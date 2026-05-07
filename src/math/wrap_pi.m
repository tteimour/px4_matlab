function a = wrap_pi(a)
% wrap_pi — wrap angle (rad) into the closed interval [-pi, pi].
%   Toolbox-free replacement for wrapToPi from the Mapping Toolbox.
    a = mod(a + pi, 2*pi) - pi;
end
