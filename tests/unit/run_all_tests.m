function run_all_tests()
% Runs all unit tests under tests/unit/ in alphabetical order.
here = fileparts(mfilename('fullpath'));
files = dir(fullfile(here, 'test_*.m'));
fail = 0;
for i = 1:numel(files)
    name = files(i).name(1:end-2);
    fprintf('--- %s ---\n', name);
    try
        feval(name);
    catch ME
        fprintf(2, '  %s\n', ME.message);
        fail = fail + 1;
    end
end
if fail == 0
    fprintf('\nAll tests passed (%d files).\n', numel(files));
else
    fprintf(2, '\n%d / %d tests FAILED.\n', fail, numel(files));
end
end
