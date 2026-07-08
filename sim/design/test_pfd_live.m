function test_pfd_live()
% Offscreen validation of the live glass ADI + HSI arc (verbatim copies of
% buildPFD/pfdUpdate/halfDiskPoly from run_interactive.m). Renders at the
% real inst_pan aspect (~470x392 px) and a representative attitude.
here = fileparts(mfilename('fullpath'));
f = figure('Units','pixels','Position',[60 60 470 392],'Color','k', ...
           'Visible','off','MenuBar','none','ToolBar','none','InvertHardcopy','off');
pan = uipanel(f,'Units','normalized','Position',[0 0 1 1],'BackgroundColor',hx('151C24'));
P = buildPFD(pan);
% roll -15, pitch +8, EKF hdg 311, GT hdg 305, wind from 250 @ 7 m/s
toDir = 250-180; wind = 7*[cosd(toDir); sind(toDir); 0];
P.update(deg2rad(-15), deg2rad(8), 311, 305, wind);
exportgraphics(f, fullfile(here,'pfd_live.png'), 'Resolution',150,'BackgroundColor',hx('0D1117'));
close(f); fprintf('wrote pfd_live.png\n');
end

function T = gcsTheme()  % minimal stand-in for the run_interactive local theme
T.bg=hx('0D1117'); T.panel=hx('151C24'); T.field=hx('0A0E13'); T.edge=hx('2B3947');
T.text=hx('E9EEF3'); T.sub=hx('8CA0B3'); T.good=hx('43D29A'); T.bad=hx('FF5252');
T.data=hx('6FD3FF'); T.acft=hx('FFD24D'); T.sky=hx('2C5B86'); T.gnd=hx('5A4632');
T.mono=pick({'Ubuntu Mono','DejaVu Sans Mono','Liberation Mono'});
end
function c=hx(s), c=[hex2dec(s(1:2)) hex2dec(s(3:4)) hex2dec(s(5:6))]/255; end
function fn=pick(c), av=listfonts; fn=c{end}; for i=1:numel(c), if any(strcmpi(av,c{i})), fn=c{i}; return; end, end, end

% ===================== verbatim from run_interactive.m =====================
function P = buildPFD(parent)
T = gcsTheme();
R = 0.97; kP = R * 0.150;
axA = axes('Parent', parent, 'Units', 'normalized', 'Position', [0.05 0.345 0.90 0.625]);
hold(axA, 'on'); axis(axA, 'off');
set(axA, 'XLim', [-1.36 1.36], 'YLim', [-1.18 1.18], 'DataAspectRatio', [1 1 1], 'Color', T.bg);
th = linspace(0, 2*pi, 220);
patch(axA, R*cos(th), R*sin(th), T.gnd, 'EdgeColor', 'none', 'HitTest', 'off');
A.sky     = patch(axA, NaN, NaN, T.sky, 'EdgeColor', 'none', 'HitTest', 'off');
A.horizon = line(axA, NaN, NaN, 'Color', 'w', 'LineWidth', 2.2, 'HitTest', 'off');
A.ladder  = line(axA, NaN, NaN, 'Color', 'w', 'LineWidth', 1.4, 'HitTest', 'off');
A.lblL = gobjects(1, 6); A.lblR = gobjects(1, 6);
for i = 1:6
    A.lblL(i) = text(axA, NaN, NaN, '', 'Color', 'w', 'FontName', T.mono, 'FontSize', 8, ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'HitTest', 'off');
    A.lblR(i) = text(axA, NaN, NaN, '', 'Color', 'w', 'FontName', T.mono, 'FontSize', 8, ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'HitTest', 'off');
end
A.bank = line(axA, NaN, NaN, 'Color', 'w', 'LineWidth', 1.2, 'HitTest', 'off');
yb = R*0.86;
patch(axA, [-0.05 0.05 0], [yb+0.07 yb+0.07 yb], T.acft, 'EdgeColor', 'none', 'HitTest', 'off');
line(axA, [-0.42 -0.12 NaN 0.12 0.42], [0 0 NaN 0 0], 'Color', T.acft, 'LineWidth', 4, 'HitTest', 'off');
line(axA, [-0.12 -0.12 NaN 0.12 0.12], [0 -0.07 NaN 0 -0.07], 'Color', T.acft, 'LineWidth', 4, 'HitTest', 'off');
line(axA, 0, 0, 'Marker', 'o', 'MarkerSize', 4.5, 'MarkerFaceColor', T.acft, 'MarkerEdgeColor', T.acft, 'HitTest', 'off');
line(axA, R*cos(th), R*sin(th), 'Color', T.edge, 'LineWidth', 2, 'HitTest', 'off');
rectangle(axA, 'Position', [-1.34 0.92 0.62 0.22], 'Curvature', 0.25, 'FaceColor', T.field, 'EdgeColor', T.edge);
rectangle(axA, 'Position', [ 0.72 0.92 0.62 0.22], 'Curvature', 0.25, 'FaceColor', T.field, 'EdgeColor', T.edge);
text(axA, -1.30, 1.09, 'ROLL',  'Color', T.sub, 'FontName', T.mono, 'FontSize', 7, 'HitTest', 'off');
text(axA,  0.76, 1.09, 'PITCH', 'Color', T.sub, 'FontName', T.mono, 'FontSize', 7, 'HitTest', 'off');
A.rollTxt  = text(axA, -0.76, 0.99, '--', 'Color', T.text, 'FontName', T.mono, 'FontSize', 12, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'right', 'HitTest', 'off');
A.pitchTxt = text(axA,  1.30, 0.99, '--', 'Color', T.text, 'FontName', T.mono, 'FontSize', 12, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'right', 'HitTest', 'off');
A.R = R; A.kP = kP;
axH = axes('Parent', parent, 'Units', 'normalized', 'Position', [0.05 0.045 0.90 0.265]);
hold(axH, 'on'); axis(axH, 'off');
set(axH, 'XLim', [-1.30 1.30], 'YLim', [0.10 1.05], 'Color', T.field);
A.Hc = -1.54; A.Rc = 2.40; A.Hf = 0.675;
A.hsiTicks = line(axH, NaN, NaN, 'Color', 'w', 'LineWidth', 1.0, 'HitTest', 'off');
A.hsiLbl = gobjects(1, 11);
for i = 1:11
    A.hsiLbl(i) = text(axH, NaN, NaN, '', 'Color', 'w', 'FontName', T.mono, 'FontSize', 9, ...
        'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'HitTest', 'off');
end
A.gtTick = line(axH, NaN, NaN, 'Color', T.good, 'LineWidth', 3, 'HitTest', 'off');
A.gtLbl  = text(axH, NaN, NaN, 'GT', 'Color', T.good, 'FontName', T.mono, 'FontSize', 8, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'HitTest', 'off');
yl = A.Hc + A.Rc;
patch(axH, [-0.05 0.05 0], [yl+0.12 yl+0.12 yl+0.02], 'w', 'EdgeColor', 'none', 'HitTest', 'off');
rectangle(axH, 'Position', [-0.17 0.80 0.34 0.20], 'Curvature', 0.25, 'FaceColor', T.field, 'EdgeColor', T.acft, 'LineWidth', 1.4);
A.hdgTxt = text(axH, 0, 0.90, '---', 'Color', T.text, 'FontName', T.mono, 'FontSize', 13, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'HitTest', 'off');
text(axH, 0.22, 0.90, 'MAG', 'Color', T.good, 'FontName', T.mono, 'FontSize', 8, 'FontWeight', 'bold', 'HitTest', 'off');
rectangle(axH, 'Position', [-1.26 0.16 0.74 0.30], 'Curvature', 0.18, 'FaceColor', T.field, 'EdgeColor', T.data, 'LineWidth', 1.0);
A.windDir = text(axH, -1.20, 0.36, '---\circ', 'Color', T.text, 'FontName', T.mono, 'FontSize', 11, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'left', 'Interpreter', 'tex', 'HitTest', 'off');
A.windSpd = text(axH, -1.20, 0.23, '-- m/s', 'Color', T.data, 'FontName', T.mono, 'FontSize', 9, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'left', 'HitTest', 'off');
text(axH, -0.66, 0.40, 'WIND', 'Color', T.sub, 'FontName', T.mono, 'FontSize', 7, 'FontWeight', 'bold', 'HitTest', 'off');
A.gtHdgTxt  = text(axH, 1.26, 0.40, 'GT  ---', 'Color', T.good, 'FontName', T.mono, 'FontSize', 9, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'right', 'HitTest', 'off');
A.ekfHdgTxt = text(axH, 1.26, 0.24, 'EKF ---', 'Color', T.bad, 'FontName', T.mono, 'FontSize', 9, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'right', 'HitTest', 'off');
P = struct('update', @(roll, pitch, ekf_hdg, gt_hdg, wind_ned) ...
                     pfdUpdate(A, roll, pitch, ekf_hdg, gt_hdg, wind_ned));
end

function pfdUpdate(A, roll, pitch, ekf_hdg, gt_hdg, wind_ned)
R = A.R; kP = A.kP;
u = [-sin(roll); cos(roll)]; t = [cos(roll); sin(roll)];
pu = -pitch / deg2rad(10) * kP;
Psky = halfDiskPoly(R, u, pu);
if isempty(Psky), set(A.sky, 'XData', NaN, 'YData', NaN);
else,             set(A.sky, 'XData', Psky(1,:), 'YData', Psky(2,:)); end
if abs(pu) < R
    Lh = sqrt(R^2 - pu^2); c0 = pu*u; a1 = c0 + Lh*t; b1 = c0 - Lh*t;
    set(A.horizon, 'XData', [a1(1) b1(1)], 'YData', [a1(2) b1(2)]);
else
    set(A.horizon, 'XData', NaN, 'YData', NaN);
end
dvals = [-30 -20 -10 10 20 30]; xs = []; ys = [];
for i = 1:6
    d = dvals(i); off = pu + d/10*kP; c = off*u; a1 = c + 0.15*t; b1 = c - 0.15*t;
    xs = [xs a1(1) b1(1) NaN]; ys = [ys a1(2) b1(2) NaN]; %#ok<AGROW>
    e = (a1-b1)/norm(a1-b1); Lp = a1 + e*0.07; Rp = b1 - e*0.07;
    set(A.lblL(i), 'Position', [Lp(1) Lp(2) 0], 'String', num2str(abs(d)), 'Rotation', -rad2deg(roll));
    set(A.lblR(i), 'Position', [Rp(1) Rp(2) 0], 'String', num2str(abs(d)), 'Rotation', -rad2deg(roll));
end
for d = [-35 -25 -15 -5 5 15 25 35]
    off = pu + d/10*kP; c = off*u; a1 = c + 0.06*t; b1 = c - 0.06*t;
    xs = [xs a1(1) b1(1) NaN]; ys = [ys a1(2) b1(2) NaN]; %#ok<AGROW>
end
set(A.ladder, 'XData', xs, 'YData', ys);
bx = []; by = [];
for a = [-60 -45 -30 -20 -10 0 10 20 30 45 60]
    big = any(a == [0 -30 30 -60 60]); r0 = R*0.86; r1 = r0 + 0.05 + 0.04*big;
    aa = pi/2 - deg2rad(a) + roll;
    bx = [bx r0*cos(aa) r1*cos(aa) NaN]; by = [by r0*sin(aa) r1*sin(aa) NaN]; %#ok<AGROW>
end
set(A.bank, 'XData', bx, 'YData', by);
set(A.rollTxt,  'String', sprintf('%+03d', round(rad2deg(roll))));
set(A.pitchTxt, 'String', sprintf('%+03d', round(rad2deg(pitch))));
cur = ekf_hdg; Hc = A.Hc; Rc = A.Rc; Hf = A.Hf;
xs = []; ys = []; li = 0;
for b = (cur-46):(cur+46)
    if mod(round(b),5) ~= 0, continue; end
    big = mod(round(b),10) == 0; aa = pi/2 - deg2rad(b-cur)*Hf;
    r0 = Rc - (0.10 + 0.10*big);
    xs = [xs r0*cos(aa) Rc*cos(aa) NaN]; ys = [ys Hc+r0*sin(aa) Hc+Rc*sin(aa) NaN]; %#ok<AGROW>
    if big && li < numel(A.hsiLbl)
        li = li + 1; rl = Rc - 0.30;
        set(A.hsiLbl(li), 'Position', [rl*cos(aa) Hc+rl*sin(aa) 0], ...
            'String', sprintf('%02d', mod(round(b/10),36)), 'Visible', 'on');
    end
end
set(A.hsiTicks, 'XData', xs, 'YData', ys);
for j = li+1:numel(A.hsiLbl), set(A.hsiLbl(j), 'Visible', 'off'); end
dg = mod(gt_hdg - cur + 180, 360) - 180;
if abs(dg) <= 46
    aa = pi/2 - deg2rad(dg)*Hf; r0 = Rc - 0.22;
    set(A.gtTick, 'XData', [r0*cos(aa) Rc*cos(aa)], 'YData', [Hc+r0*sin(aa) Hc+Rc*sin(aa)], 'Visible', 'on');
    rl = Rc - 0.40; set(A.gtLbl, 'Position', [rl*cos(aa) Hc+rl*sin(aa) 0], 'Visible', 'on');
else
    set(A.gtTick, 'Visible', 'off'); set(A.gtLbl, 'Visible', 'off');
end
set(A.hdgTxt, 'String', sprintf('%03d', round(mod(cur,360))));
spd = hypot(wind_ned(1), wind_ned(2));
wfrom = mod(rad2deg(atan2(-wind_ned(2), -wind_ned(1))), 360);
set(A.windDir, 'String', sprintf('%03d\\circ', round(wfrom)));
set(A.windSpd, 'String', sprintf('%.1f m/s', spd));
set(A.gtHdgTxt,  'String', sprintf('GT  %03d', round(mod(gt_hdg,360))));
set(A.ekfHdgTxt, 'String', sprintf('EKF %03d', round(mod(cur,360))));
end

function P = halfDiskPoly(R, u, h)
P = [];
if h >= R, return; end
t = [u(2); -u(1)];
th = linspace(0, 2*pi, 240); circ = [R*cos(th); R*sin(th)];
pu_ = u'*circ; pt_ = t'*circ;
keep = pu_ >= h;
if ~any(keep), return; end
a = atan2(pt_(keep), pu_(keep)); pts = circ(:, keep);
[~, ord] = sort(a); P = pts(:, ord);
end
