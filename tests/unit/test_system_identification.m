function test_system_identification()
% Unit tests for SystemIdentification against PX4 reference values from
%   src/lib/system_identification/system_identification_test.cpp

addpaths();

%% basicTest: short integer sequence, fs = 800 Hz
fs = 800;
sysid = SystemIdentification();
sysid.setHpfCutoffFrequency(fs, 0.05);
sysid.setLpfCutoffFrequency(fs, 30);
sysid.setForgettingFactor(80, 1 / fs);

for i = 1:5
    sysid.update(0, 0);             % fill buffers with zeros
end
for i = 0:2:8
    sysid.update(i, i + 1);         % (0,1),(2,3),(4,5),(6,7),(8,9)
end

coeff = sysid.getCoefficients();
check = [-3.96; 1.30; -2.28; -0.33; 0.369];
assert_near(coeff, check, 1e-2, 'SystemIdentification basicTest coefficients');

%% resetTest: reset clears state and is reproducible
fs = 800;
sysid = SystemIdentification();
sysid.setHpfCutoffFrequency(fs, 0.05);
sysid.setLpfCutoffFrequency(fs, 30);
sysid.setForgettingFactor(80, 1 / fs);
for i = 0:2:8
    sysid.update(i, i + 1);
end
coeff_before = sysid.getCoefficients();

sysid.reset();
assert(max(abs(sysid.getCoefficients())) < 1e-8, 'FAIL: coefficients reset');
assert(min(sysid.getVariances()) > 9e3, 'FAIL: variances reset');

for i = 0:2:8
    sysid.update(i, i + 1);
end
assert_near(sysid.getCoefficients(), coeff_before, 1e-8, 'Reproducible after reset');

%% simulatedModelTest: recover a known ARX model with bias, fs = 200 Hz
fs = 200;
gyro_lpf_cutoff = 30;
sysid = SystemIdentification();
sysid.setHpfCutoffFrequency(fs, 0.05);
sysid.setLpfCutoffFrequency(fs, gyro_lpf_cutoff);
sysid.setForgettingFactor(60, 1 / fs);

a1 = -1.77; a2 = 0.77; b0 = 0.3812; b1 = -0.25; b2 = 0.2;
u_bias = -0.1;     % constant control offset
y_bias =  0.2;     % measurement bias

dt = 1 / fs;
duration = 2.0;
n = floor(duration / dt);

% Direct Form II plant state (matches the C++ test helper "apply()").
de1 = 0; de2 = 0;
u = 0; y = 0;

for i = 0:(n - 1)
    if sysid.areFiltersInitialized() && (mod(i, 30) == 0)
        if u > 0, u = -1; else, u = 1; end
    end

    sysid.update(u + u_bias, y + y_bias);   % new input, previous output

    % y = apply(u)
    de0 = u - de1 * a1 - de2 * a2;
    y   = de0 * b0 + de1 * b1 + de2 * b2;
    de2 = de1;
    de1 = de0;
end

coeff = sysid.getCoefficients();
check = [a1; a2; b0; b1; b2];
assert_near(coeff, check, 1e-3, 'SystemIdentification recovers simulated ARX model');

fprintf('test_system_identification: PASS\n');
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
