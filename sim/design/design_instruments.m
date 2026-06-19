function design_instruments()
% DESIGN_INSTRUMENTS  Render redesign comps for the attitude indicator,
% waypoint table, and the dynamic map. Output PNGs in sim/design/.
%
%   adi_glass.png     - round glass-cockpit PFD (sky/ground disk, pitch
%                       ladder, bank arc + slip, digital ROLL/PITCH)
%   adi_tactical.png  - square tactical attitude block (flat, big digits)
%   wptable_pro.png   - professional flight-plan list (type badges, dist/hdg,
%                       totals footer)
%   wptable_tac.png   - dense tactical table (status dots, active-WP marker)
%   dynmap.png        - dynamic map: follow-box, smooth recenter, Cesium
%                       georeference origin pin
%
% Rendered offscreen; real attitude is drawn with hgtransform in the live
% GUI, this just shows the target look at a representative attitude.

here = fileparts(mfilename('fullpath'));
T = palette();

% representative live values
phi = -15; theta = 8; hdg = 47;
wps = sampleWaypoints();

render(here, T, 'adi_glass.png',    640, 680, @(ax) renderADI(ax, T, 'glass',    phi, theta, hdg));
render(here, T, 'adi_tactical.png', 640, 680, @(ax) renderADI(ax, T, 'tactical', phi, theta, hdg));
render(here, T, 'wptable_pro.png',  840, 520, @(ax) renderTablePro(ax, T, wps));
render(here, T, 'wptable_tac.png',  840, 520, @(ax) renderTableTac(ax, T, wps));
render(here, T, 'dynmap.png',      1180, 680, @(ax) renderDynMap(ax, T, wps));
end


% =========================================================================
% ATTITUDE INDICATOR
% =========================================================================
function renderADI(ax, T, style, phi, theta, hdg)
W = 640; H = 680;
cx = W/2; cy = H/2 + 8; R = 236;
k = R*0.150;                              % screen units per 10 deg pitch
phir = deg2rad(phi);
u = [-sin(phir); cos(phir)];              % aircraft-up on screen
t = [ cos(phir); sin(phir)];              % along-horizon
pu = -theta/10 * k;                       % horizon offset along u (pitch up -> down)

if strcmp(style,'glass')
    skyc = T.sky; gndc = T.gnd; bezel = mix(T.panel,T.bg,0.2);
    rrect(ax, cx-R-26, cy-R-26, 2*(R+26), 2*(R+26), 22, bezel, T.edge, 1.5);
    glassDisk(ax, cx, cy, R, u, pu, skyc, gndc);
    ringC(ax, cx, cy, R, T.edge, 2);
else
    skyc = mix(T.sky,T.bg,0.35); gndc = mix(T.gnd,T.bg,0.25); bezel = T.field;
    rrect(ax, cx-R-26, cy-R-26, 2*(R+26), 2*(R+26), 6, bezel, T.data, 1.5);
    squareHorizon(ax, cx, cy, R, phir, pu, skyc, gndc);
    rrect(ax, cx-R, cy-R, 2*R, 2*R, 2, 'none', T.edge, 1);
    brackets(ax, cx-R, cy-R, 2*R, 2*R, 26, T.data, 2);
end

% horizon line + pitch ladder (rolled)
lineU(ax, cx, cy, u, t, pu, R*0.95, 'w', 2.2);
ladderCol = 'w'; if ~strcmp(style,'glass'), ladderCol = [0.85 0.92 0.98]; end
for d = [-30 -20 -10 10 20 30]
    off = pu + d/10*k;
    half = R*0.14; if mod(d,20)~=0 && abs(d)~=10, half = R*0.06; end
    seg = (strcmp(style,'tactical'));
    ladderBar(ax, cx, cy, u, t, off, half, ladderCol, 1.6, num2str(abs(d)), phi, seg);
end
for d = [-5 5 15 -15 25 -25 35 -35]                 % minor
    off = pu + d/10*k;
    ladderBar(ax, cx, cy, u, t, off, R*0.05, ladderCol, 1.0, '', phi, false);
end

% bank arc (top), rotates with roll; fixed amber pointer
bankArc(ax, cx, cy, R, phi, T, style);
% fixed aircraft symbol
acftSymbol(ax, cx, cy, R, T);
% slip/skid
slip(ax, cx, cy, R, T);

% digital ROLL / PITCH readouts
if strcmp(style,'glass')
    digBox(ax, cx-R-26, cy+R-12, 150, 44, 'ROLL', sprintf('%+03d\\circ', round(phi)), T, T.text);
    digBox(ax, cx+R-124, cy+R-12, 150, 44, 'PITCH', sprintf('%+03d\\circ', round(theta)), T, T.text);
    txt(ax, cx, H-26, 'ATTITUDE  \cdot  PRIMARY FLIGHT DISPLAY', T.sub, 11, T.mono, 'bold', 'center');
else
    bigDig(ax, cx-R-26, cy+R-16, 'ROLL',  sprintf('%+03d',round(phi)),  T.data, T);
    bigDig(ax, cx+R-150, cy+R-16, 'PITCH', sprintf('%+03d',round(theta)), T.acft, T);
    txt(ax, cx, H-26, 'ATTITUDE // TACTICAL', T.data, 12, T.mono, 'bold', 'center');
end
% heading readout under instrument
rrect(ax, cx-46, cy-R-58, 92, 30, 4, T.field, T.acft, 1.2);
txt(ax, cx, cy-R-43, sprintf('%03d\\circ', round(hdg)), T.text, 15, T.mono, 'bold', 'center');
txt(ax, cx, cy-R-72, 'HDG', T.sub, 9, T.mono, 'bold', 'center');
end

function glassDisk(ax, cx, cy, R, u, h, skyc, gndc)
th = linspace(0,2*pi,160);
patch(ax,'XData',cx+R*cos(th),'YData',cy+R*sin(th),'FaceColor',gndc,'EdgeColor','none');
P = halfDiskPoly(cx, cy, R, u, h);
if ~isempty(P)
    patch(ax,'XData',P(1,:),'YData',P(2,:),'FaceColor',skyc,'EdgeColor','none');
end
end

function P = halfDiskPoly(cx, cy, R, u, h)
P = [];
if h >= R, return; end
t = [u(2); -u(1)];
th = linspace(0,2*pi,400); circ = [cx+R*cos(th); cy+R*sin(th)];
rel = circ - [cx;cy];
pu_ = u'*rel; pt_ = t'*rel;
keep = pu_ >= h;
if ~any(keep), return; end
a = atan2(pt_(keep), pu_(keep));
pts = circ(:,keep);
[~,ord] = sort(a);
P = pts(:,ord);
end

function squareHorizon(ax, cx, cy, R, phir, h, skyc, gndc)
R2 = R*1.6;
Rm = [cos(phir) -sin(phir); sin(phir) cos(phir)];
% ground = big rect, sky = upper half above line offset h along u
sky = [-R2 R2 R2 -R2; h h R2 R2];
gnd = [-R2 R2 R2 -R2; -R2 -R2 h h];
sky = Rm*sky + [cx;cy]; gnd = Rm*gnd + [cx;cy];
patch(ax,'XData',gnd(1,:),'YData',gnd(2,:),'FaceColor',gndc,'EdgeColor','none');
patch(ax,'XData',sky(1,:),'YData',sky(2,:),'FaceColor',skyc,'EdgeColor','none');
end

function lineU(ax, cx, cy, u, t, off, half, col, lw)
c = [cx;cy] + off*u;
A = c + half*t; B = c - half*t;
line(ax,[A(1) B(1)],[A(2) B(2)],'Color',col,'LineWidth',lw);
end

function ladderBar(ax, cx, cy, u, t, off, half, col, lw, label, phi, seg)
c = [cx;cy] + off*u; A = c + half*t; B = c - half*t;
if seg && half > 1
    n = 5; for i=0:n-1
        p1 = A + (B-A)*(i/n); p2 = A + (B-A)*((i+0.6)/n);
        line(ax,[p1(1) p2(1)],[p1(2) p2(2)],'Color',col,'LineWidth',lw);
    end
else
    line(ax,[A(1) B(1)],[A(2) B(2)],'Color',col,'LineWidth',lw);
end
if ~isempty(label)
    L = A + (A-B)/norm(A-B)*16;
    text(ax,L(1),L(2),label,'Color',col,'FontName','Ubuntu Mono','FontSize',9, ...
        'FontWeight','bold','Rotation',-phi,'HorizontalAlignment','center','VerticalAlignment','middle');
    Rr = B + (B-A)/norm(A-B)*16;
    text(ax,Rr(1),Rr(2),label,'Color',col,'FontName','Ubuntu Mono','FontSize',9, ...
        'FontWeight','bold','Rotation',-phi,'HorizontalAlignment','center','VerticalAlignment','middle');
end
end

function bankArc(ax, cx, cy, R, phi, T, style)
ticks = [-60 -45 -30 -20 -10 0 10 20 30 45 60];
for a = ticks
    big = any(a==[0 -30 30 -60 60]);
    r0 = R*0.86; r1 = r0 + R*(0.05 + 0.045*big);
    th = pi/2 - deg2rad(a) + deg2rad(phi);
    p0 = [cx + r0*cos(th); cy + r0*sin(th)];
    p1 = [cx + r1*cos(th); cy + r1*sin(th)];
    line(ax,[p0(1) p1(1)],[p0(2) p1(2)],'Color','w','LineWidth',1.2+0.6*big);
end
% fixed pointer (amber triangle) at top, pointing down to arc
yp = cy + R*0.86;
patch(ax,'XData',[cx-9 cx+9 cx],'YData',[yp+18 yp+18 yp],'FaceColor',T.acft,'EdgeColor','none');
end

function acftSymbol(ax, cx, cy, R, T)
w = R*0.42; g = R*0.10;
line(ax,[cx-w cx-g],[cy cy],'Color',T.acft,'LineWidth',5);
line(ax,[cx+g cx+w],[cy cy],'Color',T.acft,'LineWidth',5);
line(ax,[cx-g cx-g],[cy cy-R*0.07],'Color',T.acft,'LineWidth',5);
line(ax,[cx+g cx+g],[cy cy-R*0.07],'Color',T.acft,'LineWidth',5);
line(ax,cx,cy,'Marker','o','MarkerSize',5,'MarkerFaceColor',T.acft,'MarkerEdgeColor',T.acft);
end

function slip(ax, cx, cy, R, T)
yb = cy + R*0.70; w = R*0.10;
line(ax,[cx-w cx-w],[yb yb-10],'Color','w','LineWidth',1.2);
line(ax,[cx+w cx+w],[yb yb-10],'Color','w','LineWidth',1.2);
rectangle(ax,'Position',[cx+R*0.02-7 yb-9 14 9],'Curvature',[1 1],'FaceColor','w','EdgeColor','none');
end

function digBox(ax, x, y, w, h, lab, val, T, valcol)
rrect(ax, x, y, w, h, 5, T.field, T.edge, 1.0);
txt(ax, x+10, y+h-11, lab, T.sub, 9, 'Ubuntu Mono', 'bold', 'left');
txt(ax, x+w-12, y+15, val, valcol, 17, 'Ubuntu Mono', 'bold', 'right');
end

function bigDig(ax, x, y, lab, val, col, T)
txt(ax, x+4, y+8, lab, T.sub, 10, T.mono, 'bold', 'left');
txt(ax, x+4, y-22, val, col, 30, T.mono, 'bold', 'left');
end


% =========================================================================
% WAYPOINT TABLE — professional flight-plan list
% =========================================================================
function renderTablePro(ax, T, wps)
W=840; H=520;
rrect(ax, 16, 16, W-32, H-32, 10, mix(T.panel,T.bg,0.2), T.edge, 1.0);
txt(ax, 36, H-40, 'MISSION PLAN', T.text, 14, T.font, 'bold', 'left');
txt(ax, 36, H-60, 'EAGLE-1  \cdot  6 ITEMS', T.data, 9.5, T.mono, 'bold', 'left');
% header
hy = H-92; cols = [56 150 300 392 484 590 700];
hdr = {'#','TYPE','N (m)','E (m)','ALT','DIST','BRG'};
rrect(ax, 28, hy-6, W-56, 28, 4, T.field, 'none', 0.1);
for i=1:numel(cols)
    txt(ax, cols(i), hy+8, hdr{i}, T.sub, 9.5, T.mono, 'bold', 'left');
end
% rows
n = numel(wps); ry = hy-16; rh = 40;
for i=1:n
    yy = ry - (i-1)*rh;
    if mod(i,2)==0, rrect(ax, 28, yy-rh+8, W-56, rh-4, 3, mix(T.panel,T.btn,0.4),'none',0.1); end
    if i==3, rrect(ax, 28, yy-rh+8, 4, rh-4, 1, T.acft, T.acft, 0.1); end  % selected
    w = wps(i);
    txt(ax, cols(1), yy-8, sprintf('%02d',i), T.text, 11, T.mono, 'bold','left');
    badge(ax, cols(2), yy-14, w.type, T);
    txt(ax, cols(3), yy-8, sprintf('%7.1f',w.n), T.text, 11, T.mono, 'normal','left');
    txt(ax, cols(4), yy-8, sprintf('%7.1f',w.e), T.text, 11, T.mono, 'normal','left');
    txt(ax, cols(5), yy-8, sprintf('%4.0f',w.alt), T.text, 11, T.mono, 'normal','left');
    txt(ax, cols(6), yy-8, sprintf('%4.0f m',w.dist), T.sub, 10, T.mono, 'normal','left');
    txt(ax, cols(7), yy-8, sprintf('%03.0f\\circ',w.brg), T.sub, 10, T.mono, 'normal','left');
end
% footer totals
fy = 36;
hline(ax, 28, fy+26, W-28, T.edge, 0.8);
txt(ax, 36, fy+8, 'TOTAL DISTANCE', T.sub, 9.5, T.mono, 'bold','left');
txt(ax, 300, fy+8, '1 420 m', T.good, 12, T.mono, 'bold','left');
txt(ax, 484, fy+8, 'EST TIME', T.sub, 9.5, T.mono, 'bold','left');
txt(ax, 590, fy+8, '04:12', T.good, 12, T.mono, 'bold','left');
end

function badge(ax, x, y, type, T)
switch type
    case 'TAKEOFF', c=T.good; case 'LAND', c=T.bad; otherwise, c=T.data;
end
w = 86;
rrect(ax, x, y, w, 22, 4, mix(c,T.bg,0.78), c, 1.0);
txt(ax, x+w/2, y+11, type, c, 9, 'Ubuntu Mono', 'bold','center');
end


% =========================================================================
% WAYPOINT TABLE — dense tactical
% =========================================================================
function renderTableTac(ax, T, wps)
W=840; H=520;
rrect(ax, 16, 16, W-32, H-32, 2, mix(T.panel,T.bg,0.35), 'none', 0.1);
brackets(ax, 16, 16, W-32, H-32, 18, T.data, 1.8);
txt(ax, 40, H-44, '\diamondsuit WAYPOINT MATRIX // EAGLE-1', T.data, 13, T.mono, 'bold', 'left');
hline(ax, 32, H-58, W-32, T.edge, 0.8);
hdr = {'WP','STAT','NORTH','EAST','ALT','LEG'};
cols = [48 120 230 360 500 620];
for i=1:numel(hdr), txt(ax, cols(i), H-78, hdr{i}, T.sub, 9.5, T.mono, 'bold','left'); end
n=numel(wps); ry=H-96; rh=42;
for i=1:n
    yy = ry-(i-1)*rh;
    active = (i==3);
    if active, rrect(ax, 32, yy-rh+8, W-64, rh-6, 2, mix(T.data,T.bg,0.84), T.data, 1.0); end
    w = wps(i);
    if active, txt(ax, 36, yy-9, '\rightarrow', T.data, 13, T.mono, 'bold','left'); end
    dotc = T.good; if strcmp(w.type,'LAND'), dotc=T.bad; elseif strcmp(w.type,'WP'), dotc=T.acft; end
    rectangle(ax,'Position',[cols(1)+8 yy-13 9 9],'Curvature',[1 1],'FaceColor',dotc,'EdgeColor','none');
    txt(ax, cols(1)+24, yy-9, sprintf('%02d',i), T.text, 11, T.mono,'bold','left');
    txt(ax, cols(2), yy-9, w.type, dotc, 9.5, T.mono,'bold','left');
    txt(ax, cols(3), yy-9, sprintf('%+08.1f',w.n), T.text, 11, T.mono,'normal','left');
    txt(ax, cols(4), yy-9, sprintf('%+08.1f',w.e), T.text, 11, T.mono,'normal','left');
    txt(ax, cols(5), yy-9, sprintf('%4.0f',w.alt), T.acft, 11, T.mono,'bold','left');
    txt(ax, cols(6), yy-9, sprintf('%4.0fm',w.dist), T.sub, 10, T.mono,'normal','left');
end
txt(ax, 40, 34, 'LEG TOTAL 1 420 m   \cdot   NEXT: WP-03   \cdot   XTE 1.2 m', T.good, 10, T.mono, 'bold','left');
end


% =========================================================================
% DYNAMIC MAP
% =========================================================================
function renderDynMap(ax, T, wps)
W=1180; H=680;
x0=24; y0=24; mw=W-48; mh=H-48;
% terrain backdrop
rrect(ax, x0, y0, mw, mh, 6, hx('11281F'), T.edge, 1.0);
patchBlob(ax, x0+200, y0+180, 240, hx('163524'));
patchBlob(ax, x0+760, y0+430, 300, hx('15301F'));
% river
line(ax, [x0+60 x0+360 x0+620 x0+980], [y0+520 y0+360 y0+300 y0+120], ...
     'Color', hx('1C4A63'), 'LineWidth', 9);
% lat/lon grid
for gx = x0+120:160:x0+mw-40
    line(ax,[gx gx],[y0 y0+mh],'Color',[1 1 1 ]*0+mix([1 1 1],hx('11281F'),0.82),'LineWidth',0.4);
end
for gy = y0+110:150:y0+mh-30
    line(ax,[x0 x0+mw],[gy gy],'Color',mix([1 1 1],hx('11281F'),0.82),'LineWidth',0.4);
end

cx = x0+mw*0.46; cy = y0+mh*0.5;
% follow-box (deadzone) — UAV stays inside; crossing it pans the viewport
fw = mw*0.42; fh = mh*0.42;
dashRect(ax, cx-fw/2, cy-fh/2, fw, fh, T.acft, 1.4);
txt(ax, cx-fw/2+8, cy+fh/2-14, 'FOLLOW BOX', T.acft, 9, T.mono, 'bold','left');

% course + waypoints (place around)
px = cx + [-280 -120 60 220 330];
py = cy + [-150 -40 60 150 200];
line(ax, px, py, 'Color', T.nav, 'LineWidth', 2.2);
mk(ax, px(1), py(1), T.good, 13);  txt(ax,px(1)-4,py(1)+20,'LAUNCH',T.good,8.5,'Ubuntu Mono','bold','center');
for i=2:4, mk(ax, px(i), py(i), T.acft, 10); txt(ax,px(i),py(i)+18,sprintf('WP%02d',i),T.acft,8,'Ubuntu Mono','bold','center'); end
mk(ax, px(5), py(5), T.bad, 13);   txt(ax,px(5),py(5)+20,'END',T.bad,8.5,'Ubuntu Mono','bold','center');

% UAV at the follow-box edge, with pan-direction arrow + ghost viewport
ux = cx+fw/2; uy = cy+fh*0.18;
uavIcon(ax, ux, uy, 35, T.acft);
arrow(ax, ux+18, uy, ux+96, uy+30, T.data, 2.4);
txt(ax, ux+104, uy+44, 'VIEWPORT FOLLOWS', T.data, 9.5, T.mono, 'bold','left');
txt(ax, ux+104, uy+28, 'SMOOTH PAN \rightarrow', T.data, 9.5, T.mono, 'bold','left');
dashRect(ax, x0+18+90, y0+18, mw-36, mh-36, mix(T.data,hx('11281F'),0.4), 1.0);  % ghost next viewport

% origin pin (Cesium georeference)
ox = cx-180; oy = cy-30;
line(ax,[ox ox],[oy oy+26],'Color','w','LineWidth',1.5);
mk(ax, ox, oy, 'w', 9);
rrect(ax, ox+10, oy+14, 360, 40, 4, mix(T.bg,hx('11281F'),0.2), T.data, 1.0);
txt(ax, ox+22, oy+40, 'MAP ORIGIN \cdot CESIUM GEOREF', T.sub, 9, T.mono, 'bold','left');
txt(ax, ox+22, oy+22, '40.32214\circN  49.59745\circE', T.text, 11, T.mono, 'bold','left');

% scale bar + north arrow + heading
scaleBar(ax, x0+30, y0+34, 200, '200 m', T);
northArrow(ax, x0+mw-44, y0+mh-60, T);
% title strip
txt(ax, x0+18, y0+mh-26, 'TACTICAL MAP // DYNAMIC FOLLOW', T.data, 13, T.mono, 'bold','left');
txt(ax, x0+mw-18, y0+mh-26, 'N-UP  \cdot  GT/EKF TRAILS LIVE', T.sub, 9.5, T.mono, 'bold','right');
end

function patchBlob(ax, x, y, r, c)
th=linspace(0,2*pi,40); rr = r*(1+0.18*sin(5*th));
patch(ax,'XData',x+rr.*cos(th),'YData',y+rr.*sin(th),'FaceColor',c,'EdgeColor','none');
end
function mk(ax,x,y,c,s)
rectangle(ax,'Position',[x-s/2 y-s/2 s s],'Curvature',[1 1],'FaceColor',c,'EdgeColor','k','LineWidth',1);
end
function uavIcon(ax,x,y,s,c)
line(ax,[x-s/2 x+s/2],[y y],'Color',c,'LineWidth',3.5);
line(ax,[x x],[y-s/3 y+s/3],'Color',c,'LineWidth',3.5);
rectangle(ax,'Position',[x-4 y-4 8 8],'Curvature',[1 1],'FaceColor',c,'EdgeColor','none');
end
function arrow(ax,x1,y1,x2,y2,c,lw)
line(ax,[x1 x2],[y1 y2],'Color',c,'LineWidth',lw);
v=[x2-x1;y2-y1]; v=v/norm(v); n=[-v(2);v(1)]; tip=[x2;y2];
a=tip-14*v+7*n; b=tip-14*v-7*n;
patch(ax,'XData',[tip(1) a(1) b(1)],'YData',[tip(2) a(2) b(2)],'FaceColor',c,'EdgeColor','none');
end
function dashRect(ax,x,y,w,h,c,lw)
dashLine(ax,[x x+w],[y y],c,lw); dashLine(ax,[x x+w],[y+h y+h],c,lw);
dashLine(ax,[x x],[y y+h],c,lw); dashLine(ax,[x+w x+w],[y y+h],c,lw);
end
function dashLine(ax,xx,yy,c,lw)
n=40; t=linspace(0,1,n);
for i=1:2:n-1
    p1=[xx(1)+(xx(2)-xx(1))*t(i); yy(1)+(yy(2)-yy(1))*t(i)];
    p2=[xx(1)+(xx(2)-xx(1))*t(i+1); yy(1)+(yy(2)-yy(1))*t(i+1)];
    line(ax,[p1(1) p2(1)],[p1(2) p2(2)],'Color',c,'LineWidth',lw);
end
end
function scaleBar(ax,x,y,w,lab,T)
line(ax,[x x+w],[y y],'Color','w','LineWidth',2.5);
line(ax,[x x],[y-5 y+5],'Color','w','LineWidth',2.5);
line(ax,[x+w x+w],[y-5 y+5],'Color','w','LineWidth',2.5);
txt(ax,x+w/2,y+14,lab,T.text,9.5,T.mono,'bold','center');
end
function northArrow(ax,x,y,T)
patch(ax,'XData',[x x-9 x+9],'YData',[y+26 y-6 y-6],'FaceColor','w','EdgeColor','none');
patch(ax,'XData',[x x-9 x],'YData',[y+26 y-6 y-6],'FaceColor',T.bad,'EdgeColor','none');
txt(ax,x,y+38,'N',T.text,11,T.mono,'bold','center');
end


% =========================================================================
% Data
% =========================================================================
function w = sampleWaypoints()
T = {'TAKEOFF','WP','WP','WP','WP','LAND'};
N = [0  120  340  340  120  0];
E = [0  0    260  520  520  300];
A = [10 30   40   40   30   0];
D = [0  120  330  280  400  290];
B = [0  0    47   90   180  225];
for i=1:6
    w(i) = struct('type',T{i},'n',N(i),'e',E(i),'alt',A(i),'dist',D(i),'brg',B(i)); %#ok<AGROW>
end
end


% =========================================================================
% Shared primitives / palette
% =========================================================================
function render(here, T, name, W, H, drawfn)
f = figure('Units','pixels','Position',[60 60 W H],'Color','k', ...
           'Visible','off','MenuBar','none','ToolBar','none','InvertHardcopy','off');
ax = axes('Parent',f,'Position',[0 0 1 1]); hold(ax,'on'); axis(ax,'off');
set(ax,'XLim',[0 W],'YLim',[0 H],'YDir','normal','Color','none');
drawfn(ax);
exportgraphics(f, fullfile(here,name), 'Resolution', 130, 'BackgroundColor', T.bg);
close(f); fprintf('wrote %s\n', name);
end

function rrect(ax, x, y, w, h, r, face, edge, lw)
cx = min(1, 2*r/max(w,eps)); cy = min(1, 2*r/max(h,eps));
args = {'Position',[x y max(w,0.1) max(h,0.1)],'Curvature',[cx cy],'LineWidth',lw,'Parent',ax};
if ischar(face)&&strcmp(face,'none'), args=[args {'FaceColor','none'}]; else, args=[args {'FaceColor',face}]; end
if ischar(edge)&&strcmp(edge,'none'), args=[args {'EdgeColor','none'}]; else, args=[args {'EdgeColor',edge}]; end
rectangle(args{:});
end
function txt(ax,x,y,s,col,fs,font,weight,halign)
text(ax,x,y,s,'Color',col,'FontSize',fs,'FontName',font,'FontWeight',weight, ...
     'HorizontalAlignment',halign,'VerticalAlignment','middle','Interpreter','tex');
end
function hline(ax,x1,y,x2,col,lw), line(ax,[x1 x2],[y y],'Color',col,'LineWidth',lw); end
function ringC(ax,cx,cy,R,col,lw)
th=linspace(0,2*pi,200); line(ax,cx+R*cos(th),cy+R*sin(th),'Color',col,'LineWidth',lw);
end
function brackets(ax,x,y,w,h,L,col,lw)
P={[x x+L;y y],[x x;y y+L],[x+w-L x+w;y y],[x+w x+w;y y+L], ...
   [x x+L;y+h y+h],[x x;y+h-L y+h],[x+w-L x+w;y+h y+h],[x+w x+w;y+h-L y+h]};
for i=1:numel(P), line(ax,P{i}(1,:),P{i}(2,:),'Color',col,'LineWidth',lw); end
end
function T = palette()
T.bg=hx('0D1117'); T.panel=hx('151C24'); T.field=hx('0A0E13'); T.btn=hx('1F2935');
T.edge=hx('2B3947'); T.text=hx('E9EEF3'); T.sub=hx('8CA0B3'); T.good=hx('43D29A');
T.warn=hx('FFB02E'); T.bad=hx('FF5252'); T.nav=hx('E060E0'); T.data=hx('6FD3FF');
T.acft=hx('FFD24D'); T.sky=hx('2C5B86'); T.gnd=hx('5A4632');
T.font=pick({'Ubuntu','Noto Sans','DejaVu Sans','Helvetica'});
T.mono=pick({'Ubuntu Mono','DejaVu Sans Mono','Liberation Mono','Consolas'});
end
function c=hx(s), c=[hex2dec(s(1:2)) hex2dec(s(3:4)) hex2dec(s(5:6))]/255; end
function c=mix(a,b,t), c=a*(1-t)+b*t; end
function f=pick(c), av=listfonts; f=c{end}; for i=1:numel(c), if any(strcmpi(av,c{i})), f=c{i}; return; end, end, end
