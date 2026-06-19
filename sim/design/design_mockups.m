function design_mockups()
% DESIGN_MOCKUPS  Render parameter-tab redesign comps as PNGs.
%
% Generates three visual directions for the parameter tabs (Controller PID
% gains shown), using the live gcs theme palette and REAL default gains from
% px4_params (no invented numbers). Output: sim/design/mockup_A|B|C.png
%
%   A  Tactical Cards   - reference-style grouped cards, header bars,
%                         value chips + [-]/[+] steppers, left accent stripe
%   B  HUD Console      - corner-bracket frames, segmented LCD readouts,
%                         range bar-gauges, scanline texture
%   C  Modern Flat      - axis-colour-coded rows, filled sliders, value pills
%
% Rendered offscreen (no interactive GUI launch) and exported with
% exportgraphics.

here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(fullfile(root, 'src', 'params'));
p = px4_params();

data = buildData(p);
T    = palette();

variants = {'A', 'B', 'C'};
renderfn = {@renderA, @renderB, @renderC};
for i = 1:numel(variants)
    f = newCanvas();
    ax = axes('Parent', f, 'Position', [0 0 1 1]); %#ok<LAXES>
    setupAx(ax);
    renderfn{i}(ax, T, data);
    out = fullfile(here, sprintf('mockup_%s.png', variants{i}));
    exportgraphics(f, out, 'Resolution', 130, 'BackgroundColor', T.bg);
    close(f);
    fprintf('wrote %s\n', out);
end
end


% =========================================================================
% Real parameter data (so the comps show authentic numbers).
% =========================================================================
function d = buildData(p)
d.tabs = {'OVERVIEW', 'CONTROLLER', 'SENSORS', 'EKF2', 'MISSION', 'LOGS'};
d.active = 2;

d.pos = {
    row('Pos P',     p.pos.gain_pos_p(:)',                   'MPC_XY_P / Z_P',      [0 3]);
    row('Vel P',     p.pos.gain_vel_p(:)',                   'MPC_*_VEL_P_ACC',     [0 8]);
    row('Vel I',     p.pos.gain_vel_i(:)',                   'MPC_*_VEL_I_ACC',     [0 5]);
    row('Vel D',     p.pos.gain_vel_d(:)',                   'MPC_*_VEL_D_ACC',     [0 2]);
    row('Tilt max',  rad2deg(p.pos.tilt_max),                'MPC_TILTMAX_AIR  deg',[0 80]);
    row('Hover thr', p.pos.thr_hover,                        'MPC_THR_HOVER',       [0 1]);
};
d.att = {
    row('Att P',    p.att.gain_p(:)',                        'MC_R/P/Y_P',          [0 12]);
    row('Yaw wt',   p.att.yaw_weight,                        'MC_YAW_WEIGHT',       [0 1]);
    row('Rate max', rad2deg(p.att.rate_max(:)'),             'MC_*RATE_MAX  deg/s', [0 360]);
};
d.rate = {
    row('Rate P', p.rate.gain_p(:)'.*p.rate.gain_k(:)',      'MC_*RATE_P x K',      [0 0.6]);
    row('Rate I', p.rate.gain_i(:)'.*p.rate.gain_k(:)',      'MC_*RATE_I x K',      [0 0.8]);
    row('Rate D', p.rate.gain_d(:)'.*p.rate.gain_k(:)',      'MC_*RATE_D x K',      [0 0.02]);
    row('FF',     p.rate.gain_ff(:)',                        'MC_*RATE_FF',         [0 0.5]);
};
end

function r = row(name, vals, param, rng)
% Escape underscores so the TeX renderer shows them literally (not as
% subscripts) in the parameter captions.
r = struct('name', name, 'vals', vals, 'param', strrep(param,'_','\_'), 'rng', rng);
end


% =========================================================================
% VARIANT A — Tactical Cards
% =========================================================================
function renderA(ax, T, d)
chrome(ax, T, d, T.data, 'MODIFIABLE CONTROL PARAMETERS', 'PID GAINS  ·  GROUND-TRUTH FEED');

% Two columns. Left: Position. Right: Attitude + Rate.
cardA(ax, T, 28,  86, 564, 562, 'POSITION LOOP  (OUTER)',  d.pos,  T.data);
cardA(ax, T, 608, 420, 564, 228, 'ATTITUDE LOOP',          d.att,  T.acft);
cardA(ax, T, 608, 86,  564, 314, 'RATE LOOP  (INNER)',     d.rate, T.good);

buttonsA(ax, T);
end

function cardA(ax, T, x, y, w, h, title, rows, accent)
rrect(ax, x, y, w, h, 8, T.panel, T.edge, 1.0);
rrect(ax, x, y, 5, h, 2, accent, accent, 0.1);                 % left accent stripe
% header bar
hb = 40;
rrect(ax, x+9, y+h-hb-6, w-18, hb, 5, T.field, T.edge, 0.8);
txt(ax, x+24, y+h-hb/2-6, upper(title), accent, 11.5, T.mono, 'bold', 'left');
txt(ax, x+w-22, y+h-hb/2-6, 'LIVE', T.sub, 8.5, T.mono, 'bold', 'right');
ledA(ax, x+w-58, y+h-hb/2-6, accent);

n = numel(rows);
top = y+h-hb-18; bot = y+12; rh = (top-bot)/n;
for i = 1:n
    rA(ax, T, x+18, top - i*rh, w-36, rh, rows{i}, accent);
end
end

function rA(ax, T, x, yc0, w, rh, r, accent)
yc = yc0 + rh/2;
txt(ax, x, yc, r.name, T.text, 11, T.font, 'bold', 'left');
txt(ax, x, yc-rh*0.30, r.param, T.sub, 7.5, T.mono, 'normal', 'left');

vals = r.vals; nv = numel(vals);
chipw = 58; gap = 6; stw = 22;
xr = x + w;                              % right edge
% steppers
rrect(ax, xr-stw, yc-11, stw, 22, 4, T.btn, T.edge, 0.8);
txt(ax, xr-stw/2, yc, '+', T.text, 12, T.font, 'bold', 'center');
xr = xr - stw - 4;
rrect(ax, xr-stw, yc-11, stw, 22, 4, T.btn, T.edge, 0.8);
txt(ax, xr-stw/2, yc, '-', T.text, 13, T.font, 'bold', 'center');
xr = xr - stw - gap;
% value chips (right to left)
for k = nv:-1:1
    rrect(ax, xr-chipw, yc-12, chipw, 24, 4, T.field, T.edge, 0.8);
    txt(ax, xr-chipw/2, yc, fmtv(vals(k)), accentForK(T, accent, k, nv), 11, T.mono, 'bold', 'center');
    xr = xr - chipw - gap;
end
hline(ax, x, yc0+1, x+w, T.edge, 0.5);
end

function buttonsA(ax, T)
rrect(ax, 28, 28, 360, 44, 6, T.good, T.good, 0.05);
txt(ax, 208, 50, 'APPLY CHANGES', T.bg, 12, T.font, 'bold', 'center');
rrect(ax, 404, 28, 220, 44, 6, T.btn, T.edge, 1.0);
txt(ax, 514, 50, 'RESTORE DEFAULTS', T.sub, 11, T.font, 'bold', 'center');
txt(ax, 1172, 50, 'AUTOTUNE \rightarrow', T.acft, 11, T.mono, 'bold', 'right');
end


% =========================================================================
% VARIANT B — HUD Console
% =========================================================================
function renderB(ax, T, d)
scanlines(ax, T);
chrome(ax, T, d, T.data, 'CONTROL PARAMETER MATRIX', 'PID  ·  TELEMETRY-LOCKED');

cardB(ax, T, 28,  86, 564, 562, 'POSITION // OUTER', d.pos,  T.data);
cardB(ax, T, 608, 420, 564, 228, 'ATTITUDE',         d.att,  T.acft);
cardB(ax, T, 608, 86,  564, 314, 'RATE // INNER',    d.rate, T.good);

% command bar
rrect(ax, 28, 28, 1144, 44, 2, T.field, T.edge, 1.0);
brackets(ax, 28, 28, 1144, 44, 12, T.data, 1.6);
txt(ax, 52, 50, '\bullet APPLY', T.good, 12, T.mono, 'bold', 'left');
txt(ax, 230, 50, '\bullet RESTORE', T.warn, 12, T.mono, 'bold', 'left');
txt(ax, 1148, 50, 'PARAMS NOMINAL', T.good, 10, T.mono, 'bold', 'right');
end

function cardB(ax, T, x, y, w, h, title, rows, accent)
rrect(ax, x, y, w, h, 2, mix(T.panel, T.bg, 0.4), 'none', 0.1);
brackets(ax, x, y, w, h, 16, accent, 1.8);
txt(ax, x+18, y+h-18, ['\diamondsuit ' upper(title)], accent, 11.5, T.mono, 'bold', 'left');
hline(ax, x+14, y+h-34, x+w-14, T.edge, 0.8);

n = numel(rows);
top = y+h-44; bot = y+14; rh = (top-bot)/n;
for i = 1:n
    rB(ax, T, x+18, top - i*rh, w-36, rh, rows{i}, accent);
end
end

function rB(ax, T, x, yc0, w, rh, r, accent)
yc = yc0 + rh/2;
txt(ax, x, yc+rh*0.08, r.name, T.text, 11, T.mono, 'bold', 'left');
txt(ax, x, yc-rh*0.30, r.param, T.sub, 7.5, T.mono, 'normal', 'left');

vals = r.vals; nv = numel(vals);
% LCD value readout (rightmost), plus a range bar under it
boxw = 84; gap = 8; xr = x + w;
for k = nv:-1:1
    rrect(ax, xr-boxw, yc-1, boxw, rh*0.42, 1.5, T.field, accent, 1.0);
    txt(ax, xr-boxw/2, yc+rh*0.20, fmtv(vals(k)), accent, 12, T.mono, 'bold', 'center');
    % range bar gauge
    frac = clamp((vals(k)-r.rng(1))/max(eps,(r.rng(2)-r.rng(1))), 0, 1);
    by = yc - rh*0.30; bx = xr-boxw; bw = boxw;
    rrect(ax, bx, by, bw, 4, 1, mix(T.field, T.edge, 0.5), 'none', 0.1);
    rrect(ax, bx, by, max(2,bw*frac), 4, 1, accent, 'none', 0.1);
    xr = xr - boxw - gap;
end
end


% =========================================================================
% VARIANT C — Modern Flat (axis-colour-coded sliders)
% =========================================================================
function renderC(ax, T, d)
chrome(ax, T, d, T.good, 'CONTROL TUNING', 'PID GAINS');

% axis legend
legendC(ax, T, 880, 666);

cardC(ax, T, 28,  86, 564, 562, 'Position loop',  d.pos);
cardC(ax, T, 608, 420, 564, 228, 'Attitude loop', d.att);
cardC(ax, T, 608, 86,  564, 314, 'Rate loop',     d.rate);

% pill buttons
rrect(ax, 28, 30, 300, 42, 21, T.good, T.good, 0.05);
txt(ax, 178, 51, 'Apply changes', T.bg, 12, T.font, 'bold', 'center');
rrect(ax, 344, 30, 200, 42, 21, mix(T.panel,T.bg,0.2), T.edge, 1.0);
txt(ax, 444, 51, 'Restore defaults', T.text, 11, T.font, 'normal', 'center');
end

function cardC(ax, T, x, y, w, h, title, rows)
rrect(ax, x, y, w, h, 10, mix(T.panel, T.bg, 0.25), T.edge, 1.0);
txt(ax, x+22, y+h-22, title, T.text, 12.5, T.font, 'bold', 'left');
hline(ax, x+18, y+h-38, x+w-18, T.edge, 0.8);

n = numel(rows);
top = y+h-48; bot = y+14; rh = (top-bot)/n;
for i = 1:n
    rC(ax, T, x+22, top - i*rh, w-44, rh, rows{i});
end
end

function rC(ax, T, x, yc0, w, rh, r)
yc = yc0 + rh/2;
txt(ax, x, yc+rh*0.06, r.name, T.text, 11, T.font, 'bold', 'left');
txt(ax, x, yc-rh*0.30, r.param, T.sub, 7.5, T.mono, 'normal', 'left');

vals = r.vals; nv = numel(vals);
axisCols = {T.data, T.acft, T.nav};
% slider track spanning mid-right
trackx = x + 150; trackw = w - 150 - 96;
% draw nv stacked thin sliders if multi, else one
for k = 1:nv
    yy = yc + (nv>1)*((k-(nv+1)/2)*min(11, rh/(nv+1)));
    frac = clamp((vals(k)-r.rng(1))/max(eps,(r.rng(2)-r.rng(1))), 0, 1);
    col = axisCols{min(k,3)};
    rrect(ax, trackx, yy-2.5, trackw, 5, 2.5, mix(T.field,T.edge,0.6), 'none', 0.1);
    rrect(ax, trackx, yy-2.5, max(3,trackw*frac), 5, 2.5, col, 'none', 0.1);
    % handle
    rrect(ax, trackx+trackw*frac-5, yy-7, 10, 14, 3, col, T.bg, 1.0);
end
% value pill (single combined or first)
vstr = strjoin(arrayfun(@(v) fmtv(v), vals, 'uni', 0), ' ');
pillw = 88;
rrect(ax, x+w-pillw, yc-13, pillw, 26, 13, T.field, T.edge, 1.0);
txt(ax, x+w-pillw/2, yc, vstr, T.text, 9.5, T.mono, 'bold', 'center');
end

function legendC(ax, T, x, y)
cols = {T.data, T.acft, T.nav}; labs = {'ROLL/N', 'PITCH/E', 'YAW/D'};
for i = 1:3
    cx = x + (i-1)*100;
    rrect(ax, cx, y-5, 10, 10, 5, cols{i}, cols{i}, 0.1);
    txt(ax, cx+16, y, labs{i}, T.sub, 9, T.mono, 'bold', 'left');
end
end


% =========================================================================
% Shared chrome: tab bar + title strip
% =========================================================================
function chrome(ax, T, d, accent, title, subtitle)
% top tab bar
rrect(ax, 0, 700, 1200, 60, 0.1, mix(T.panel,T.bg,0.3), 'none', 0.1);
hline(ax, 0, 700, 1200, T.edge, 1.0);
txt(ax, 28, 730, '\diamondsuit  PX4-MATLAB GCS', T.text, 13, T.font, 'bold', 'left');
tx = 360;
for i = 1:numel(d.tabs)
    on = (i==d.active);
    c  = T.text; if ~on, c = T.sub; end
    w  = 12*strlength(d.tabs{i}) + 26;
    txt(ax, tx+w/2, 730, d.tabs{i}, c, 10.5, T.font, ternary(on,'bold','normal'), 'center');
    if on
        rrect(ax, tx+8, 704, w-16, 3, 1.5, accent, accent, 0.1);
    end
    tx = tx + w + 8;
end
% title strip
txt(ax, 28, 674, title, T.text, 13, T.font, 'bold', 'left');
txt(ax, 28, 656, subtitle, accent, 9.5, T.mono, 'bold', 'left');
hline(ax, 0, 698, 1200, mix(T.edge,T.bg,0.3), 0.5);
end


% =========================================================================
% Primitive helpers
% =========================================================================
function f = newCanvas()
f = figure('Units','pixels','Position',[60 60 1200 760], 'Color', 'k', ...
           'Visible','off','MenuBar','none','ToolBar','none', ...
           'InvertHardcopy','off');
end

function setupAx(ax)
hold(ax,'on'); axis(ax,'off');
set(ax,'XLim',[0 1200],'YLim',[0 760],'YDir','normal');
set(ax,'XColor','none','YColor','none','Color','none');
end

function rrect(ax, x, y, w, h, r, face, edge, lw)
cx = min(1, 2*r/max(w,eps)); cy = min(1, 2*r/max(h,eps));
args = {'Position',[x y max(w,0.1) max(h,0.1)],'Curvature',[cx cy],'LineWidth',lw,'Parent',ax};
if ischar(face) && strcmp(face,'none'), args=[args {'FaceColor','none'}];
else, args=[args {'FaceColor',face}]; end
if ischar(edge) && strcmp(edge,'none'), args=[args {'EdgeColor','none'}];
else, args=[args {'EdgeColor',edge}]; end
rectangle(args{:});
end

function txt(ax, x, y, s, col, fs, font, weight, halign)
text(ax, x, y, s, 'Color', col, 'FontSize', fs, 'FontName', font, ...
     'FontWeight', weight, 'HorizontalAlignment', halign, ...
     'VerticalAlignment','middle', 'Interpreter','tex');
end

function hline(ax, x1, y, x2, col, lw)
line(ax, [x1 x2], [y y], 'Color', col, 'LineWidth', lw);
end

function ledA(ax, x, y, col)
rectangle(ax,'Position',[x-4 y-4 8 8],'Curvature',[1 1],'FaceColor',col,'EdgeColor','none');
end

function brackets(ax, x, y, w, h, L, col, lw)
P = {[x x+L; y y],[x x; y y+L], ...
     [x+w-L x+w; y y],[x+w x+w; y y+L], ...
     [x x+L; y+h y+h],[x x; y+h-L y+h], ...
     [x+w-L x+w; y+h y+h],[x+w x+w; y+h-L y+h]};
for i=1:numel(P), line(ax, P{i}(1,:), P{i}(2,:), 'Color', col, 'LineWidth', lw); end
end

function scanlines(ax, T)
c = mix(T.panel, T.bg, 0.35);
for yy = 90:6:648
    line(ax, [28 1172], [yy yy], 'Color', c, 'LineWidth', 0.2);
end
end

function c = accentForK(T, base, k, nv)
if nv == 1, c = base; return; end
cols = {T.data, T.acft, T.nav};
c = cols{min(k,3)};
end


% =========================================================================
% Palette / format utils
% =========================================================================
function T = palette()
T.bg    = hx('0D1117');  T.panel = hx('151C24'); T.field = hx('0A0E13');
T.btn   = hx('1F2935');  T.edge  = hx('2B3947'); T.text  = hx('E9EEF3');
T.sub   = hx('8CA0B3');  T.good  = hx('43D29A'); T.warn  = hx('FFB02E');
T.bad   = hx('FF5252');  T.nav   = hx('E060E0'); T.data  = hx('6FD3FF');
T.acft  = hx('FFD24D');
T.font  = pick({'Ubuntu','Noto Sans','DejaVu Sans','Helvetica'});
T.mono  = pick({'Ubuntu Mono','DejaVu Sans Mono','Liberation Mono','Consolas'});
end

function c = hx(s)
c = [hex2dec(s(1:2)) hex2dec(s(3:4)) hex2dec(s(5:6))] / 255;
end

function c = mix(a, b, t), c = a*(1-t) + b*t; end

function f = pick(cands)
av = listfonts; f = cands{end};
for i = 1:numel(cands)
    if any(strcmpi(av, cands{i})), f = cands{i}; return; end
end
end

function s = fmtv(v)
a = abs(v);
if a >= 100,      s = sprintf('%.0f', v);
elseif a >= 10,   s = sprintf('%.1f', v);
elseif a >= 0.1,  s = sprintf('%.3f', v);
else,             s = sprintf('%.4f', v);
end
end

function y = clamp(x, lo, hi), y = min(hi, max(lo, x)); end
function r = ternary(c, a, b), if c, r = a; else, r = b; end, end
