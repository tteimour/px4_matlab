function test_sticks_live()
% Offscreen check of the embedded manual-stick pads at their Flight-tab size
% (verbatim makeJoysticks drawing). Renders at the real figure aspect and a
% deflected cap so we can judge sizing.
here = fileparts(mfilename('fullpath'));
f = figure('Units','pixels','Position',[60 60 1400 850],'Color',hx('0D1117'), ...
           'Visible','off','MenuBar','none','ToolBar','none','InvertHardcopy','off');
% mimic the Flight-tab right column backdrop
uipanel(f,'Units','normalized','Position',[0.655 0.02 0.34 0.22],'BackgroundColor',hx('151C24'),'BorderType','line','HighlightColor',hx('2B3947'));
axl = axes('Parent',f,'Units','normalized','Position',[0.658 0.035 0.158 0.170]);
axr = axes('Parent',f,'Units','normalized','Position',[0.834 0.035 0.158 0.170]);
[lh,rh,lt,rt] = makeJoysticks(axl,axr);
set(lh,'XData',-0.4,'YData',0.6); set(lt,'XData',[0 -0.4],'YData',[0 0.6]);   % deflected
set(rh,'XData',0.5,'YData',-0.3); set(rt,'XData',[0 0.5],'YData',[0 -0.3]);
exportgraphics(f, fullfile(here,'sticks_live.png'),'Resolution',130,'BackgroundColor',hx('0D1117'));
close(f); fprintf('wrote sticks_live.png\n');
end

function [left_h,right_h,left_t,right_t] = makeJoysticks(ax_l, ax_r)
T = theme(); caps = {'YAW  /  THROTTLE','ROLL  /  PITCH'}; axs=[ax_l ax_r]; thc=linspace(0,2*pi,90);
for k=1:2
    ax=axs(k); hold(ax,'on'); axis(ax,'equal'); xlim(ax,[-1.18 1.18]); ylim(ax,[-1.45 1.18]);
    set(ax,'XTick',[],'YTick',[],'Box','off','Color',T.bg,'XColor','none','YColor','none');
    rectangle(ax,'Position',[-1.1 -1.1 2.2 2.2],'Curvature',0.18,'FaceColor',T.field,'EdgeColor',T.edge,'LineWidth',1.2);
    for r=[0.5 1.0], plot(ax,r*cos(thc),r*sin(thc),'-','Color',T.edge,'LineWidth',0.8); end
    plot(ax,[-1 1;0 0]',[0 0;-1 1]','-','Color',T.edge,'LineWidth',0.8);
    text(ax,0,-1.32,caps{k},'Color',T.sub,'FontName',T.font,'FontSize',8.5,'FontWeight','bold','HorizontalAlignment','center');
end
left_t  = plot(ax_l,[0 0],[0 0],'-','Color',T.sub,'LineWidth',2.5);
right_t = plot(ax_r,[0 0],[0 0],'-','Color',T.sub,'LineWidth',2.5);
left_h  = plot(ax_l,0,0,'o','MarkerSize',24,'MarkerFaceColor',T.warn,'MarkerEdgeColor',T.text,'LineWidth',1.2);
right_h = plot(ax_r,0,0,'o','MarkerSize',24,'MarkerFaceColor',T.data,'MarkerEdgeColor',T.text,'LineWidth',1.2);
end

function T=theme()
T.bg=hx('0D1117'); T.field=hx('0A0E13'); T.edge=hx('2B3947'); T.text=hx('E9EEF3');
T.sub=hx('8CA0B3'); T.warn=hx('FFB02E'); T.data=hx('6FD3FF');
T.font=pick({'Ubuntu','Noto Sans','DejaVu Sans','Helvetica'});
end
function c=hx(s), c=[hex2dec(s(1:2)) hex2dec(s(3:4)) hex2dec(s(5:6))]/255; end
function fn=pick(c), av=listfonts; fn=c{end}; for i=1:numel(c), if any(strcmpi(av,c{i})), fn=c{i}; return; end, end, end
