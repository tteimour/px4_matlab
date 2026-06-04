function test_arx_rls()
% Unit tests for ArxRls against PX4 reference values from
%   src/lib/system_identification/arx_rls_test.cpp

addpaths();

%% test211: ArxRls<2,1,1> (n_theta = 4)
% PX4 reference coefficients generated from the Python script.
rls = ArxRls(2, 1, 1);
for i = 1:(2 + 1 + 1)        % fill the buffers with zeros
    rls.update(0, 0);
end
rls.update(1, 2);
rls.update(3, 4);
rls.update(5, 6);
coeff = rls.getCoefficients();
check = [-1.79; 0.97; 0.42; -0.48];
assert_near(coeff, check, 1e-2, 'ArxRls<2,1,1> coefficients (test211)');

%% test221: ArxRls<2,2,1> (n_theta = 5)
rls = ArxRls(2, 2, 1);
for i = 1:(2 + 2 + 1)
    rls.update(0, 0);
end
rls.update(1, 2);
rls.update(3, 4);
rls.update(5, 6);
rls.update(7, 8);
coeff = rls.getCoefficients();
check = [-1.81; 1.06; 0.38; -0.27; 0.26];
assert_near(coeff, check, 1e-2, 'ArxRls<2,2,1> coefficients (test221)');

%% resetTest: covariance + parameters reset, reproducibility
rls = ArxRls(2, 2, 1);
for i = 1:(2 + 2 + 1)
    rls.update(0, 0);
end
rls.update(1, 2); rls.update(3, 4); rls.update(5, 6); rls.update(7, 8);
coeff_before = rls.getCoefficients();

rls.reset();
assert(min(rls.getVariances()) > 5000, 'FAIL: variances reset above 5000');
assert(max(abs(rls.getCoefficients())) < 1e-8, 'FAIL: coefficients reset to zero');

for i = 1:(2 + 2 + 1)
    rls.update(0, 0);
end
rls.update(1, 2); rls.update(3, 4); rls.update(5, 6); rls.update(7, 8);
assert_near(rls.getCoefficients(), coeff_before, 1e-10, 'Reproducible after reset');

%% Forgetting factor setters
rls = ArxRls(2, 2, 1);
rls.setForgettingFactor(80, 1 / 800);          % (time_constant, dt)
assert(abs(rls.lambda - (1 - (1/800)/80)) < 1e-12, 'FAIL: lambda from (tc, dt)');
rls.setForgettingFactor(0.995);                % (lambda)
assert(abs(rls.lambda - 0.995) < 1e-12, 'FAIL: lambda direct');

fprintf('test_arx_rls: PASS\n');
end


function addpaths()
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(fullfile(root, 'src', 'math'));
addpath(fullfile(root, 'src', 'autotune'));
end


function assert_near(a, b, tol, msg)
if any(abs(a - b) > tol, 'all')
    error('FAIL: %s (got %s, expected %s)', msg, mat2str(a(:)', 6), mat2str(b(:)', 6));
end
end
