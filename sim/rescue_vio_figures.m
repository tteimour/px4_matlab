function rescue_vio_figures()
% AÇIK duran VIO karşılaştırma şekillerini (eski düzenle çizilmiş) yerinde
% düzenler ve teze aktarır; vio_log artık erişilemez olduğunda kullanılır.
%   - Lejantlar sağ üst köşeye alınır, üst ylim büyütülür (çizgi çakışması yok).
%   - Lejantlardaki "VIO (HİZALANMIŞ)" -> "VIO".
%   - Yörünge (üstten) şeklinde gözlemci fazında uçulan kesim, gözlemci faz
%     rengiyle arkadan vurgulanır ve lejanta eklenir.
% Şekillerdeki çizgi verileri vio_plotdata_son.mat olarak da yedeklenir.
%
% Gereksinim: 6 şekil penceresi açık olmalı (vio_cmp_pos, vio_cmp_vel,
% vio_cmp_err, vio_cmp_traj, vio_ctl_pos, vio_ctl_vel etiketli).

thisdir = fileparts(mfilename('fullpath'));
C_OBS = [0.80 0.88 1.00];   % gözlemci faz rengi
C_FUS = [1.00 0.85 0.70];   % füzyon faz rengi

tags = {'vio_cmp_pos', 'vio_cmp_vel', 'vio_cmp_err', 'vio_cmp_traj', ...
        'vio_ctl_pos', 'vio_ctl_vel'};
fh = gobjects(1, numel(tags));
for i = 1:numel(tags)
    f = findobj(0, 'Type', 'figure', 'Tag', tags{i});
    if isempty(f)
        error('Şekil bulunamadı (kapatılmış olabilir): %s', tags{i});
    end
    fh(i) = f(1);
end

% --- Zaman serisi şekilleri: lejant + ylim + patch düzeltmesi -----------------
backup = struct();
for i = [1 2 3 5 6]
    backup.(tags{i}) = restyleTimeFig(fh(i));
end

% --- Füzyon aralıklarını f1'in patch'lerinden çıkar ---------------------------
ax1 = findobj(fh(1), 'Type', 'axes');
ax1 = ax1(end);                          % en üst (ilk çizilen) alt grafik
fusIv = zeros(0, 2);
for p = findobj(ax1, 'Type', 'patch')'
    if max(abs(p.FaceColor - C_FUS)) < 0.05
        fusIv(end+1, :) = [min(p.XData), max(p.XData)]; %#ok<AGROW>
    end
end
gtLine1 = findLine(ax1, '-', [0 0 0]);
vt = gtLine1.XData(:);

% --- Yörünge şekli: gözlemci kesimini arkadan vurgula + lejant ----------------
f4 = fh(4);
ax = findobj(f4, 'Type', 'axes'); ax = ax(end);
hgt  = findLine(ax, '-',  [0 0 0]);
hekf = findLine(ax, '-',  [0 0 1]);
hvio = findLine(ax, '--', []);
gtE = hgt.XData(:); gtN = hgt.YData(:);
if numel(gtE) == numel(vt)
    fus = false(size(vt));
    for k = 1:size(fusIv, 1)
        fus = fus | (vt >= fusIv(k, 1) & vt <= fusIv(k, 2));
    end
    bgE = gtE; bgN = gtN; bgE(fus) = NaN; bgN(fus) = NaN;
    hold(ax, 'on');
    hobs = plot(ax, bgE, bgN, '-', 'Color', C_OBS, 'LineWidth', 9);
    uistack(hobs, 'bottom');
else
    warning(['Yörünge örnek sayısı f1 ile uyuşmuyor (%d vs %d); ' ...
             'gözlemci vurgusu atlandı.'], numel(gtE), numel(vt));
    hobs = [];
end
yl = ylim(ax); ylim(ax, [yl(1), yl(2) + 0.25 * diff(yl)]);
if isempty(hobs)
    legend(ax, [hgt hekf hvio], {'GERÇEK DEĞER', 'EKF', 'VIO'}, ...
           'Location', 'northeast', 'FontSize', 11);
else
    legend(ax, [hgt hekf hvio hobs], ...
           {'GERÇEK DEĞER', 'EKF', 'VIO', 'VIO GÖZLEMCİ'}, ...
           'Location', 'northeast', 'FontSize', 11);
end
backup.(tags{4}) = grabLines(f4);

save(fullfile(thisdir, 'vio_plotdata_son.mat'), 'backup', 'fusIv', 'vt');
fprintf('Şekil verileri yedeklendi: vio_plotdata_son.mat\n');

% --- Teze aktar ----------------------------------------------------------------
imgdir = '/home/teymur/ytu_thesis/thesis/master_ytu/thesisChapters/images';
names = {'fig_vio_pos_ned.png', 'fig_vio_vel_ned.png', 'fig_vio_err_surat.png', ...
         'fig_vio_yorunge.png', 'fig_vio_track_pos.png', 'fig_vio_track_vel.png'};
for i = 1:numel(tags)
    exportgraphics(fh(i), fullfile(imgdir, names{i}), 'Resolution', 150);
end
fprintf('Tez şekilleri kaydedildi: %s\n', imgdir);
end


function bk = restyleTimeFig(f)
% Lejantlı eksenlerde üst ylim'i büyüt, patch'leri yeni ylim'e uzat,
% lejantı sağ üst köşeye al ve "VIO (HİZALANMIŞ)" -> "VIO" düzelt.
axs = findobj(f, 'Type', 'axes');
for ax = axs'
    lgd = ax.Legend;
    if isempty(lgd), continue; end
    yl = ylim(ax);
    yl2 = [yl(1), yl(2) + 0.35 * diff(yl)];
    ylim(ax, yl2);
    for p = findobj(ax, 'Type', 'patch')'
        p.YData = [yl2(1); yl2(1); yl2(2); yl2(2)];
    end
    s = lgd.String;
    s = strrep(s, 'VIO (HİZALANMIŞ)', 'VIO');
    s = strrep(s, 'VIO (hizalanmış)', 'VIO');
    lgd.String = s;
    set(lgd, 'Location', 'northeast', 'FontSize', 11);
end
bk = grabLines(f);
end


function h = findLine(ax, style, color)
% Eksendeki çizgiyi stil (ve istenirse renk) ile bulur.
for q = findobj(ax, 'Type', 'line')'
    if strcmp(q.LineStyle, style) && ...
       (isempty(color) || max(abs(q.Color - color)) < 0.05)
        h = q;
        return;
    end
end
error('Çizgi bulunamadı (stil %s).', style);
end


function bk = grabLines(f)
% Şekildeki tüm çizgi verilerini yedek struct'a kopyalar.
bk = {};
for q = findobj(f, 'Type', 'line')'
    bk{end+1} = struct('x', q.XData, 'y', q.YData, ...
                       'style', q.LineStyle, 'color', q.Color); %#ok<AGROW>
end
end
