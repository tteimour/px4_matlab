function test_pid_design()
% Unit tests for pid_design_gmvc against PX4 reference values from
%   src/lib/pid_design/pid_design_test.cpp

addpaths();

%% testImpl: compare against the PX4 Python-generated reference gains
num = [0.0098; -0.162; 0.147];     % [b0; b1; b2]
den = [1; -1.814; 0.822];          % [1; a1; a2]
dt  = 0.004;
kid = pid_design_gmvc(num, den, dt, 0.1, 1.0, 0.5);

assert_near(kid(1), 0.129,         1e-3,  'GMVC kc');
assert_near(kid(2), 11.911 / 5.0,  1e-3,  'GMVC ki (includes /5 factor)');
assert_near(kid(3), 0.0463,        1e-4,  'GMVC kd');

%% Degenerate numerator (nu ~ 0) returns zeros without error
kid0 = pid_design_gmvc([0; 0; 0], [1; -1.814; 0.822], dt, 0.1, 1.0, 0.0);
assert_near(kid0, [0; 0; 0], 1e-12, 'GMVC returns zeros when nu ~ 0');

fprintf('test_pid_design: PASS\n');
end


function addpaths()
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(fullfile(root, 'src', 'autotune'));
end


function assert_near(a, b, tol, msg)
if any(abs(a - b) > tol, 'all')
    error('FAIL: %s (got %s, expected %s)', msg, mat2str(a(:)', 6), mat2str(b(:)', 6));
end
end
