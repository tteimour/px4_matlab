function plot_vio_session(matfile)
%PLOT_VIO_SESSION  Reproduce the VIO / EKF / ground-truth comparison figures
% from a saved session .mat. run_interactive.m writes one timestamped .mat into
% this folder on every VIO-disable (and on Stop); it does NOT plot live. Run
% this whenever you want to see the figures:
%
%   plot_vio_session                                  % newest session here
%   plot_vio_session('vio_session_20260613_142530.mat')   % a specific session
%   plot_vio_session('/abs/path/to/some_session.mat')
%
% It loads the logged data (L, n) and runs the SAME plotting engine the live
% disable used, sim/replot_vio_figures.m -- so the figures are identical:
% SE3-aligns VIO to ground truth (umeyama_align), then draws position (NED),
% velocity (NED), error+speed, top-down trajectory, and controller-tracking.

here = fileparts(mfilename('fullpath'));      % .../vio_sessions
root = fileparts(here);                       % repo root
addpath(fullfile(root, 'sim'));               % replot_vio_figures
addpath(fullfile(root, 'src', 'math'));       % umeyama_align, quat_to_dcm

% --- resolve the session file -------------------------------------------------
if nargin < 1 || isempty(matfile)
    f = dir(fullfile(here, 'vio_session_*.mat'));
    if isempty(f)
        error('plot_vio_session:noSessions', ...
            'No vio_session_*.mat in %s. Run a VIO session first.', here);
    end
    [~, newest] = max([f.datenum]);
    matfile = fullfile(f(newest).folder, f(newest).name);
    fprintf('Newest session: %s\n', matfile);
elseif exist(matfile, 'file') ~= 2
    % allow a bare filename relative to this folder
    cand = fullfile(here, matfile);
    if exist(cand, 'file') == 2
        matfile = cand;
    else
        error('plot_vio_session:notFound', 'Session file not found: %s', matfile);
    end
end

% --- load and plot ------------------------------------------------------------
S = load(matfile);
if ~isfield(S, 'L') || ~isfield(S, 'n')
    error('plot_vio_session:badFile', ...
        '%s is not a VIO session file (expected variables L and n).', matfile);
end

replot_vio_figures(S.L, S.n);
end
