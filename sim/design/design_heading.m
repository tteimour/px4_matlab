function design_heading()
% DESIGN_HEADING  Mockups for the heading indicator (HSI) + redesigned top
% strip. Output PNGs in sim/design/.
%
%   hsi_rose.png  - north-up compass rose: GT heading needle (green), EKF
%                   heading needle (red), wind arrow + digital wind. Shows
%                   all three references as vectors at once.
%   hsi_arc.png   - heading-up bottom arc (matches the Boeing-PFD reference):
%                   rotating tape, lubber + digital heading, GT tick, mag bug,
%                   wind cell.
%   topstrip.png  - two redesign options for the top telemetry bar
%                   (drops the redundant ALT/VS/GS/HDG/N/E numbers).
%
% Representative live values (real GUI feeds GT yaw, EKF yaw, and the live
% wind vector from the Wind tab).

here = fileparts(mfilename('fullpath'));
T = palette();
gt = 305; ekf = 311; windFrom = 250; windSpd = 7.0;

render(here, T, 'hsi_rose.png', 620, 620, @(ax) renderRose(ax, T, gt, ekf, windFrom, windSpd));
render(here, T, 'hsi_arc.png',  760, 440, @(ax) renderArc (ax, T, gt, ekf, windFrom, windSpd));
render(here, T, 'topstrip.png', 1280, 300, @(ax) renderStrip(ax, T));
end


% =========================================================================
% HSI — north-up rose with GT + EKF needles and wind
% =========================================================================
function renderRose(ax, T, gt, ekf, windFrom, windSpd)
W=620; H=620; cx=W/2; cy=H/2-6; R=232;
rrect(ax, cx-R-30, cy-R-30, 2*(R+30), 2*(R+30), 20, mix(T.panel,T.bg,0.2), T.edge, 1.5);
% rose face
th=linspace(0,2*pi,160);
patch(ax,'XData',cx+R*cos(th),'YData',cy+R*sin(th),'FaceColor',T.field,'EdgeColor','none');
ringC(ax,cx,cy,R,T.edge,2);
% ticks every 10, labels every 30 (north-up: bearing b at screen pi/2 - b)
for b=0:10:350
    ang=pi/2-deg2rad(b); big=mod(b,30)==0;
    r0=R-(8+10*big);
    line(ax,[cx+r0*cos(ang) cx+R*cos(ang)],[cy+r0*sin(ang) cy+R*sin(ang)], ...
         'Color','w','LineWidth',1+0.8*big);
    if big
        lab=compassLabel(b); rl=R-34;
        text(ax,cx+rl*cos(ang),cy+rl*sin(ang),lab,'Color','w','FontName',T.mono, ...
            'FontSize',12,'FontWeight','bold','HorizontalAlignment','center','VerticalAlignment','middle');
    end
end
% wind arrow (from windFrom, blowing toward +180); cyan, tail at upwind side
needle(ax,cx,cy,R*0.86,windFrom+180,T.data,5,'arrow');
windBarb(ax,cx,cy,R*0.86,windFrom,T.data);
% GT + EKF heading needles
needle(ax,cx,cy,R*0.80,gt,T.good,6,'arrow');
needle(ax,cx,cy,R*0.80,ekf,T.bad,4,'arrow');
% centre aircraft
acft(ax,cx,cy,T.acft);
% fixed lubber (top) + digital primary heading (EKF = autopilot feed)
patch(ax,'XData',[cx-10 cx+10 cx],'YData',[cy+R+16 cy+R+16 cy+R+2],'FaceColor',T.acft,'EdgeColor','none');
% legend / digital readouts
digRow(ax, 24,  H-30, 'GT HDG',  sprintf('%03d\\circ',round(gt)),  T.good, T);
digRow(ax, 24,  H-66, 'EKF HDG', sprintf('%03d\\circ',round(ekf)), T.bad,  T);
digRow(ax, W-250, H-30, 'WIND',    sprintf('%03d\\circ / %.1f',round(windFrom),windSpd), T.data, T);
txt(ax, W-250, H-60, sprintf('\\Delta HDG  %+d\\circ (EKF err)',round(ekf-gt)), T.sub, 9.5, T.mono,'bold','left');
txt(ax, cx, 22, 'HEADING // HSI  (NORTH-UP)', T.sub, 11, T.mono,'bold','center');
end


% =========================================================================
% HSI — heading-up bottom arc (matches reference image)
% =========================================================================
function renderArc(ax, T, gt, ekf, windFrom, windSpd)
W=760; H=440;
cx=W/2; cyc=-560; Rc=900;            % arc circle centred far below
rrect(ax, 16, 16, W-32, H-32, 8, mix(T.panel,T.bg,0.3), T.edge, 1.2);
cur = ekf;                           % heading-up uses the autopilot (EKF) heading
% arc band
for b=cur-46:1:cur+46
    ang=pi/2 - deg2rad(b-cur)*0.85;
    if mod(round(b),5)==0
        big=mod(round(b),10)==0;
        r0=Rc-(10+12*big);
        line(ax,[cx+r0*cos(ang) cx+Rc*cos(ang)]-0,[cyc+r0*sin(ang) cyc+Rc*sin(ang)], ...
             'Color','w','LineWidth',1+0.7*big);
        if big
            rl=Rc-40; lab=sprintf('%02d',mod(round(b/10),36));
            text(ax,cx+rl*cos(ang),cyc+rl*sin(ang),lab,'Color','w','FontName',T.mono, ...
                'FontSize',13,'FontWeight','bold','HorizontalAlignment','center','VerticalAlignment','middle','Rotation',(b-cur)*0.85*0);
        end
    end
end
% GT tick (green) on the arc at its own bearing
arcTick(ax,cx,cyc,Rc,gt,cur,T.good,'GT');
% fixed lubber triangle + digital heading (EKF = autopilot feed)
yL=cyc+Rc+6;
patch(ax,'XData',[cx-12 cx+12 cx],'YData',[yL+22 yL+22 yL],'FaceColor','w','EdgeColor','none');
rrect(ax, cx-46, H-78, 92, 40, 5, T.field, T.acft, 1.4);
txt(ax, cx, H-58, sprintf('%03d',round(cur)), T.text, 20, T.mono,'bold','center');
txt(ax, cx+58, H-58, 'MAG', T.good, 10, T.mono,'bold','left');
% wind cell
rrect(ax, 30, 34, 210, 64, 6, T.field, T.data, 1.2);
windBarbBox(ax, 64, 66, windFrom, cur, T.data);
txt(ax, 110, 78, sprintf('%03d\\circ',round(windFrom)), T.text, 15, T.mono,'bold','left');
txt(ax, 110, 52, sprintf('%.1f m/s',windSpd), T.data, 11, T.mono,'bold','left');
txt(ax, 150, 78, 'WIND', T.sub, 9, T.mono,'bold','left');
% GT/EKF digital
txt(ax, W-30, 86, sprintf('GT  %03d\\circ',round(gt)),  T.good, 12, T.mono,'bold','right');
txt(ax, W-30, 62, sprintf('EKF %03d\\circ',round(ekf)), T.bad,  12, T.mono,'bold','right');
txt(ax, W-30, 38, sprintf('\\Delta %+d\\circ',round(ekf-gt)), T.sub, 10, T.mono,'bold','right');
txt(ax, cx, H-22, 'HEADING // HSI  (HEADING-UP ARC)', T.sub, 10.5, T.mono,'bold','center');
end


% =========================================================================
% TOP STRIP — two redesign options
% =========================================================================
function renderStrip(ax, T)
W=1280;
% ---- Option A: minimal annunciator strip ----
y=200; h=78;
stripBar(ax, T, 20, y, W-40, h);
brand(ax, T, 40, y, h);
modeAnn(ax, T, 360, y, h, 'POSITION', T.good);
pill(ax, T, 600, y, h, 'ARMED', T.bad);
pill(ax, T, 700, y, h, 'EKF',   T.good);
pill(ax, T, 800, y, h, 'GPS 11', T.good);
pill(ax, T, 920, y, h, 'CESIUM LINK', T.good);
clockAnn(ax, T, W-200, y, h, 'T+02:14');
txt(ax, 30, y+h+18, 'OPTION A — minimal annunciator bar (drops ALT/VS/GS/HDG/N/E; those live in the PFD/HSI/table)', T.sub, 11, T.font,'bold','left');

% ---- Option B: keep 3 PFD-style annunciators ----
y=70; h=78;
stripBar(ax, T, 20, y, W-40, h);
brand(ax, T, 40, y, h);
modeAnn(ax, T, 360, y, h, 'POSITION', T.good);
ann(ax, T, 600, y, h, 'ALT',  '124.0', 'm',   T.text);
ann(ax, T, 720, y, h, 'GS',   '12.4',  'm/s', T.text);
ann(ax, T, 840, y, h, 'V/S',  '+1.8',  'm/s', T.data);
pill(ax, T, 980, y, h, 'ARMED',  T.bad);
clockAnn(ax, T, W-160, y, h, 'T+02:14');
txt(ax, 30, y+h+18, 'OPTION B — keeps ALT / GS / V/S as boxed annunciators, drops HDG / N / E (shown in HSI + waypoint table)', T.sub, 11, T.font,'bold','left');
end

function stripBar(ax,T,x,y,w,h)
rrect(ax,x,y,w,h,4,mix(T.panel,T.bg,0.25),T.edge,1.0);
end
function brand(ax,T,x,y,h)
rectangle(ax,'Position',[x y+h/2-7 12 14],'Curvature',[1 1],'FaceColor',T.data,'EdgeColor','none');
txt(ax,x+22,y+h-26,'SYNAPLINE GCS',T.text,15,T.font,'bold','left');
txt(ax,x+22,y+20,'MATLAB SIL \cdot PIXHAWK 6X \cdot NEO-M9N',T.sub,8.5,T.mono,'normal','left');
end
function modeAnn(ax,T,x,y,h,m,c)
rrect(ax,x,y+10,210,h-20,5,mix(c,T.bg,0.82),c,1.2);
txt(ax,x+14,y+h-26,'FLIGHT MODE',T.sub,8.5,T.mono,'bold','left');
txt(ax,x+14,y+22,m,c,17,T.mono,'bold','left');
end
function ann(ax,T,x,y,h,cap,val,unit,c)
rrect(ax,x,y+12,112,h-24,4,T.field,T.edge,1.0);
txt(ax,x+12,y+h-22,cap,T.sub,8.5,T.mono,'bold','left');
txt(ax,x+12,y+24,val,c,16,T.mono,'bold','left');
txt(ax,x+100,y+24,unit,T.sub,8,T.mono,'normal','right');
end
function pill(ax,T,x,y,h,s,c)
w=max(60,12*strlength(s)+18);
rrect(ax,x,y+h/2-15,w,30,15,mix(c,T.bg,0.80),c,1.2);
txt(ax,x+w/2,y+h/2,s,c,10.5,T.mono,'bold','center');
end
function clockAnn(ax,T,x,y,h,s)
txt(ax,x,y+h-26,'MISSION TIME',T.sub,8.5,T.mono,'bold','left');
txt(ax,x,y+22,s,T.text,18,T.mono,'bold','left');
end


% =========================================================================
% compass / needle helpers
% =========================================================================
function l = compassLabel(b)
switch b
    case 0,   l='N'; case 90,  l='E'; case 180, l='S'; case 270, l='W';
    otherwise, l=sprintf('%02d',round(b/10));
end
end
function needle(ax,cx,cy,len,b,c,lw,style)
ang=pi/2-deg2rad(b); tip=[cx+len*cos(ang); cy+len*sin(ang)];
line(ax,[cx tip(1)],[cy tip(2)],'Color',c,'LineWidth',lw);
if strcmp(style,'arrow')
    v=[cos(ang);sin(ang)]; n=[-v(2);v(1)]; a=tip-16*v+8*n; bb=tip-16*v-8*n;
    patch(ax,'XData',[tip(1) a(1) bb(1)],'YData',[tip(2) a(2) bb(2)],'FaceColor',c,'EdgeColor','none');
end
end
function windBarb(ax,cx,cy,len,b,c)
ang=pi/2-deg2rad(b); base=[cx+len*cos(ang); cy+len*sin(ang)];
v=[cos(ang);sin(ang)]; n=[-v(2);v(1)];
for k=0:2
    p=base-(8+10*k)*v;
    line(ax,[p(1) p(1)+14*n(1)],[p(2) p(2)+14*n(2)],'Color',c,'LineWidth',2);
end
end
function acft(ax,cx,cy,c)
line(ax,[cx-26 cx+26],[cy cy],'Color',c,'LineWidth',4);
line(ax,[cx cx],[cy-30 cy+18],'Color',c,'LineWidth',4);
line(ax,[cx-10 cx+10],[cy-22 cy-22],'Color',c,'LineWidth',4);
end
function arcTick(ax,cx,cyc,Rc,b,cur,c,lab)
ang=pi/2-deg2rad(b-cur)*0.85; r0=Rc-26;
line(ax,[cx+r0*cos(ang) cx+Rc*cos(ang)],[cyc+r0*sin(ang) cyc+Rc*sin(ang)],'Color',c,'LineWidth',3);
text(ax,cx+(Rc-44)*cos(ang),cyc+(Rc-44)*sin(ang),lab,'Color',c,'FontName','Ubuntu Mono', ...
    'FontSize',10,'FontWeight','bold','HorizontalAlignment','center');
end
function arcBug(ax,cx,cyc,Rc,b,cur,c)
ang=pi/2-deg2rad(b-cur)*0.85; p=[cx+Rc*cos(ang); cyc+Rc*sin(ang)];
v=[cos(ang);sin(ang)]; n=[-v(2);v(1)];
q=p+10*v;
patch(ax,'XData',[q(1)-9*n(1) q(1)+9*n(1) p(1)],'YData',[q(2)-9*n(2) q(2)+9*n(2) p(2)], ...
      'FaceColor',c,'EdgeColor','none');
end
function windBarbBox(ax,x,y,windFrom,cur,c)
% small wind arrow relative to heading-up frame (rotate by -(cur))
rel=deg2rad(windFrom-cur); v=[sin(rel); cos(rel)];   % from-direction in screen
line(ax,[x-12*v(1) x+12*v(1)],[y-12*v(2) y+12*v(2)],'Color',c,'LineWidth',2.5);
tip=[x+12*v(1); y+12*v(2)]; n=[-v(2);v(1)];
patch(ax,'XData',[tip(1) tip(1)-7*v(1)+4*n(1) tip(1)-7*v(1)-4*n(1)], ...
        'YData',[tip(2) tip(2)-7*v(2)+4*n(2) tip(2)-7*v(2)-4*n(2)],'FaceColor',c,'EdgeColor','none');
end
function digRow(ax,x,y,cap,val,c,T)
txt(ax,x,y+10,cap,T.sub,9,T.mono,'bold','left');
txt(ax,x+86,y+10,val,c,15,T.mono,'bold','left');
end


% =========================================================================
% shared primitives / palette
% =========================================================================
function render(here, T, name, W, H, drawfn)
f=figure('Units','pixels','Position',[60 60 W H],'Color','k','Visible','off', ...
         'MenuBar','none','ToolBar','none','InvertHardcopy','off');
ax=axes('Parent',f,'Position',[0 0 1 1]); hold(ax,'on'); axis(ax,'off');
set(ax,'XLim',[0 W],'YLim',[0 H],'YDir','normal','Color','none');
drawfn(ax);
exportgraphics(f, fullfile(here,name), 'Resolution',130,'BackgroundColor',T.bg);
close(f); fprintf('wrote %s\n', name);
end
function rrect(ax,x,y,w,h,r,face,edge,lw)
cx=min(1,2*r/max(w,eps)); cy=min(1,2*r/max(h,eps));
a={'Position',[x y max(w,0.1) max(h,0.1)],'Curvature',[cx cy],'LineWidth',lw,'Parent',ax};
if ischar(face)&&strcmp(face,'none'), a=[a {'FaceColor','none'}]; else, a=[a {'FaceColor',face}]; end
if ischar(edge)&&strcmp(edge,'none'), a=[a {'EdgeColor','none'}]; else, a=[a {'EdgeColor',edge}]; end
rectangle(a{:});
end
function txt(ax,x,y,s,col,fs,font,weight,halign)
text(ax,x,y,s,'Color',col,'FontSize',fs,'FontName',font,'FontWeight',weight, ...
     'HorizontalAlignment',halign,'VerticalAlignment','middle','Interpreter','tex');
end
function ringC(ax,cx,cy,R,col,lw)
th=linspace(0,2*pi,200); line(ax,cx+R*cos(th),cy+R*sin(th),'Color',col,'LineWidth',lw);
end
function T=palette()
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
