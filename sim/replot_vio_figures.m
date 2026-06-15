function replot_vio_figures(L, n)
% Bölüm 8 tez şekillerini vio_log'dan yeniden çizer ve dışa aktarır.
%
% Veri kaynağı önceliği:
%   1) Argüman olarak verilen (L, n)
%   2) Base workspace'teki vio_log / vio_idx (açık simülasyon oturumu)
%   3) sim/vio_log_son.mat (önceki çalıştırmada kaydedilen kopya)
% Base workspace'ten alınan veri vio_log_son.mat olarak saklanır; böylece
% şekiller simülasyonu yeniden koşturmadan güncellenebilir.
%
% run_interactive.m/plotVioComparison ile aynı şekilleri üretir; farklar:
%   - Lejantlar sabit sağ üst köşede ve üst ylim büyütülerek çizgilerin
%     üzerine binmesi engellenir.
%   - Yörünge (üstten görünüm) şeklinde gözlemci fazında uçulan kesim,
%     gözlemci faz rengiyle arkadan vurgulanır ve lejanta eklenir.

thisdir = fileparts(mfilename('fullpath'));
addpath(fullfile(thisdir, '..', 'src', 'math'));
matfile = fullfile(thisdir, 'vio_log_son.mat');

if nargin < 2
    try
        L = evalin('base', 'vio_log');
        n = evalin('base', 'vio_idx');
        save(matfile, 'L', 'n');
        fprintf('vio_log base workspace''ten alındı ve kaydedildi: %s\n', matfile);
    catch
        S = load(matfile);
        L = S.L; n = S.n;
        fprintf('vio_log kayıttan yüklendi: %s\n', matfile);
    end
else
    save(matfile, 'L', 'n');
    fprintf('vio_log kaydedildi: %s\n', matfile);
end

if n < 10
    fprintf('VIO comparison: only %d sample(s) logged; nothing to plot.\n', max(n, 0));
    return;
end
t   = L.t(1:n);          vt   = L.vt(1:n);
gt  = L.gt_pos(1:n, :);  gtv  = L.gt_vel(1:n, :);
ekf = L.ekf_pos(1:n, :); ekfv = L.ekf_vel(1:n, :);
vio = L.vio_pos(1:n, :); viov = L.vio_vel(1:n, :);
vq  = L.vio_q(1:n, :);   fus  = L.fused(1:n);
fp  = L.fus_pos(1:n, :);     % anchor'lı VIO (EKF'e fiilen beslenen) — adil kıyas
cps = L.ctl_pos_sp(1:n, :);  cpf = L.ctl_pos_fb(1:n, :);
cvs = L.ctl_vel_sp(1:n, :);  cvf = L.ctl_vel_fb(1:n, :);
ok  = ~isnan(t) & ~isnan(vt) & all(~isnan(vio), 2) & ...
      all(~isnan(gt), 2) & all(~isnan(ekf), 2);
t = t(ok); vt = vt(ok); gt = gt(ok, :); gtv = gtv(ok, :);
ekf = ekf(ok, :); ekfv = ekfv(ok, :); vio = vio(ok, :); viov = viov(ok, :);
vq = vq(ok, :); fus = fus(ok); cps = cps(ok, :); cpf = cpf(ok, :);
cvs = cvs(ok, :); cvf = cvf(ok, :); fp = fp(ok, :);
if size(vio, 1) < 10
    fprintf('VIO comparison: <10 valid VIO samples. Skipping plot.\n');
    return;
end

% GT/EKF örneklerini VIO zaman damgasına interpole et (plotVioComparison ile aynı).
gt   = interp1(t, gt,   vt, 'linear', 'extrap');
ekf  = interp1(t, ekf,  vt, 'linear', 'extrap');
gtv  = interp1(t, gtv,  vt, 'linear', 'extrap');
ekfv = interp1(t, ekfv, vt, 'linear', 'extrap');
cps  = interp1(t, cps,  vt, 'linear', 'extrap');
cpf  = interp1(t, cpf,  vt, 'linear', 'extrap');
cvs  = interp1(t, cvs,  vt, 'linear', 'extrap');
cvf  = interp1(t, cvf,  vt, 'linear', 'extrap');
fus  = interp1(t, double(fus), vt, 'previous', 'extrap') > 0.5;

% SE3 hizalama (ölçeksiz) ve hata normları.
[R, tt, ate] = umeyama_align(vio.', gt.');
vio_a = (R * vio.' + tt).';
err_ekf = vecnorm(ekf - gt, 2, 2);
err_vio = vecnorm(vio_a - gt, 2, 2);          % SE3-hizalanmış (optimistik, yörünge çizimi için)
err_vio_fair = vecnorm(fp - gt, 2, 2);        % anchor'lı = EKF'in gördüğü VIO (adil; gözlemcide NaN)
sp_gt = vecnorm(gtv, 2, 2); sp_ekf = vecnorm(ekfv, 2, 2); sp_vio = vecnorm(viov, 2, 2);
rmse_ekf = sqrt(mean(err_ekf.^2));

fprintf('\n=== VIO vs EKF vs ground truth (%d samples, %.1f s) ===\n', ...
        size(vio, 1), vt(end) - vt(1));
fprintf('EKF position RMSE: %.3f m  (mean %.3f, max %.3f)\n', ...
        rmse_ekf, mean(err_ekf), max(err_ekf));
fprintf('VIO position ATE : %.3f m  (mean %.3f, max %.3f) after SE3 align\n', ...
        ate, mean(err_vio), max(err_vio));

% Füzyon fazı istatistikleri (alt-örneklemeden ÖNCE, tam çözünürlükte).
fus_stats = [];
if nnz(fus) > 10
    dt_s = [diff(vt); median(diff(vt))];
    ffin = fus & all(isfinite(fp), 2);   % füzyon + anchor'lı ölçüm mevcut
    fus_stats = struct( ...
        'dur',      sum(dt_s(fus)), ...
        'rmse_vio', sqrt(mean(err_vio_fair(ffin).^2)), ...
        'mean_vio', mean(err_vio_fair(ffin)), 'max_vio', max(err_vio_fair(ffin)), ...
        'rmse_ekf', sqrt(mean(err_ekf(fus).^2)), ...
        'mean_ekf', mean(err_ekf(fus)), 'max_ekf', max(err_ekf(fus)), ...
        'obs_rmse_ekf', NaN, 'obs_mean_ekf', NaN, 'obs_max_ekf', NaN);
    if any(~fus)
        fus_stats.obs_rmse_ekf = sqrt(mean(err_ekf(~fus).^2));
        fus_stats.obs_mean_ekf = mean(err_ekf(~fus));
        fus_stats.obs_max_ekf  = max(err_ekf(~fus));
    end
    fprintf(['Füzyon fazı (%.1f s): VIO-GERÇEK RMSE %.3f m (maks %.3f) | ' ...
             'EKF-GERÇEK RMSE %.3f m (maks %.3f) | gözlemci fazı EKF RMSE %.3f m\n'], ...
            fus_stats.dur, fus_stats.rmse_vio, fus_stats.max_vio, ...
            fus_stats.rmse_ekf, fus_stats.max_ekf, fus_stats.obs_rmse_ekf);
end

% Yalnızca çizim için alt-örnekleme.
np = numel(vt); ds = max(1, ceil(np / 3000)); di = 1:ds:np;
vt = vt(di); gt = gt(di, :); ekf = ekf(di, :); vio_a = vio_a(di, :);
gtv = gtv(di, :); ekfv = ekfv(di, :); viov = viov(di, :); vq = vq(di, :);
err_ekf = err_ekf(di); err_vio = err_vio(di); err_vio_fair = err_vio_fair(di);
sp_gt = sp_gt(di); sp_ekf = sp_ekf(di); sp_vio = sp_vio(di);
cps = cps(di, :); cpf = cpf(di, :); cvs = cvs(di, :); cvf = cvf(di, :);
fus = fus(di);

% VIO hızını gövde -> OpenVINS-küresel -> NED'e döndür.
vio_v = nan(numel(vt), 3);
for i = 1:numel(vt)
    qi = vq(i, :).';
    if all(isfinite(qi)) && norm(qi) > 0.5
        vio_v(i, :) = (R * (quat_to_dcm(qi / norm(qi)) * viov(i, :).')).';
    end
end

% Aşağı (D) ekseni pozitif irtifa / pozitif-yukarı dikey hız olarak göster.
gt(:, 3)  = -gt(:, 3);  ekf(:, 3)  = -ekf(:, 3);  vio_a(:, 3) = -vio_a(:, 3);
gtv(:, 3) = -gtv(:, 3); ekfv(:, 3) = -ekfv(:, 3); vio_v(:, 3) = -vio_v(:, 3);
cps(:, 3) = -cps(:, 3); cpf(:, 3) = -cpf(:, 3);
cvs(:, 3) = -cvs(:, 3); cvf(:, 3) = -cvf(:, 3);

poslbl = {'Kuzey [m]', 'Doğu [m]', 'İrtifa [m]'};
vellbl = {'Kuzey hızı [m/s]', 'Doğu hızı [m/s]', 'Dikey hız [m/s]'};
C_OBS = [0.80 0.88 1.00];   % gözlemci faz rengi (addModePatches ile aynı)

% --- Şekil 1: konum, GERÇEK / EKF / VIO ---------------------------------------
f1 = namedFig('vio_cmp_pos', 'VIO/EKF/GERÇEK: konum (NED)', [80 80 900 800]);
for i = 1:3
    ax = subplot(3, 1, i); hold(ax, 'on'); grid(ax, 'on');
    h = plot(ax, vt, gt(:, i),    'k',   'LineWidth', 1.3);
    h(2) = plot(ax, vt, ekf(:, i),   'b',   'LineWidth', 1.0);
    h(3) = plot(ax, vt, vio_a(:, i), 'r--', 'LineWidth', 1.2);
    ylabel(ax, poslbl{i});
    if i == 1, growTop(ax, 0.35); end
    addModePatches(ax, vt, fus);
    if i == 1
        legend(ax, h, {'GERÇEK DEĞER', 'EKF', 'VIO'}, ...
               'Location', 'northeast', 'FontSize', 11);
    end
end
xlabel('Benzetim zamanı [s]');

% --- Şekil 2: hız, GERÇEK / EKF / VIO -----------------------------------------
f2 = namedFig('vio_cmp_vel', 'VIO/EKF/GERÇEK: hız (NED)', [100 80 900 800]);
for i = 1:3
    ax = subplot(3, 1, i); hold(ax, 'on'); grid(ax, 'on');
    h = plot(ax, vt, gtv(:, i),   'k',   'LineWidth', 1.3);
    h(2) = plot(ax, vt, ekfv(:, i),  'b',   'LineWidth', 1.0);
    h(3) = plot(ax, vt, vio_v(:, i), 'r--', 'LineWidth', 1.2);
    ylabel(ax, vellbl{i});
    if i == 1, growTop(ax, 0.35); end
    addModePatches(ax, vt, fus);
    if i == 1
        legend(ax, h, {'GERÇEK DEĞER', 'EKF', 'VIO'}, ...
               'Location', 'northeast', 'FontSize', 11);
    end
end
xlabel('Benzetim zamanı [s]');

% --- Şekil 3: hata normu + sürat ----------------------------------------------
f3 = namedFig('vio_cmp_err', 'VIO/EKF/GERÇEK: hata + sürat', [120 80 900 620]);
ax = subplot(2, 1, 1); hold(ax, 'on'); grid(ax, 'on');
% VIO hatası adil bazda (EKF'in gördüğü anchor'lı ölçüm); füzyon fazında çizilir.
h1 = plot(ax, vt, err_ekf, 'b', 'LineWidth', 1.2);
h2 = plot(ax, vt, err_vio_fair, 'r', 'LineWidth', 1.2);
ylabel(ax, 'Konum hatası [m]');
growTop(ax, 0.35);
addModePatches(ax, vt, fus);
legend(ax, [h1 h2], {'EKF', 'VIO'}, 'Location', 'northeast', 'FontSize', 11);
if ~isempty(fus_stats)
    title(ax, sprintf('Füzyon fazı (GERÇEK''e göre): EKF RMSE %.2f m   |   VIO RMSE %.2f m', ...
                      fus_stats.rmse_ekf, fus_stats.rmse_vio));
else
    title(ax, sprintf('EKF RMSE %.3f m   |   VIO ATE %.3f m', rmse_ekf, ate));
end
ax = subplot(2, 1, 2); hold(ax, 'on'); grid(ax, 'on');
plot(ax, vt, sp_gt,  'k',   'LineWidth', 1.3);
plot(ax, vt, sp_ekf, 'b',   'LineWidth', 1.0);
plot(ax, vt, sp_vio, 'r--', 'LineWidth', 1.2);
ylabel(ax, 'Sürat [m/s]'); xlabel(ax, 'Benzetim zamanı [s]');
growTop(ax, 0.35);
addModePatches(ax, vt, fus);
legend(ax, {'GERÇEK DEĞER', 'EKF', 'VIO'}, 'Location', 'northeast', 'FontSize', 11);

% --- Şekil 4: üstten yörünge (gözlemci fazı arkadan vurgulu) ------------------
f4 = namedFig('vio_cmp_traj', 'VIO/EKF/GERÇEK: yörünge (üstten)', [140 80 800 700]);
hold on; grid on; axis equal;
gtbg = gt; gtbg(fus, :) = NaN;   % gözlemci fazında uçulan kesim
hobs = plot(gtbg(:, 2), gtbg(:, 1), '-', 'Color', C_OBS, 'LineWidth', 9);
hgt  = plot(gt(:, 2),    gt(:, 1),    'k',   'LineWidth', 1.3);
hekf = plot(ekf(:, 2),   ekf(:, 1),   'b',   'LineWidth', 1.0);
hvio = plot(vio_a(:, 2), vio_a(:, 1), 'r--', 'LineWidth', 1.2);
uistack(hobs, 'bottom');
xlabel('Doğu [m]'); ylabel('Kuzey [m]'); title('Yörünge (üstten görünüm)');
growTop(gca, 0.25);
legend([hgt hekf hvio hobs], {'GERÇEK DEĞER', 'EKF', 'VIO', 'VIO GÖZLEMCİ'}, ...
       'Location', 'northeast', 'FontSize', 11);

% --- Şekil 5: konum kontrolcüsü referans takibi -------------------------------
f5 = namedFig('vio_ctl_pos', 'Konum kontrolcüsü: referans takibi (NED)', [160 80 900 800]);
for i = 1:3
    ax = subplot(3, 1, i); hold(ax, 'on'); grid(ax, 'on');
    h1 = plot(ax, vt, cps(:, i), '--', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.2);
    h2 = plot(ax, vt, cpf(:, i), 'Color', [0.00 0.45 0.74], 'LineWidth', 1.0);
    ylabel(ax, poslbl{i});
    if i == 1, growTop(ax, 0.35); end
    addModePatches(ax, vt, fus);
    if i == 1
        legend(ax, [h1 h2], {'REFERANS', 'GERİ BESLEME'}, ...
               'Location', 'northeast', 'FontSize', 11);
        title(ax, 'Konum kontrolcüsü referans takibi');
    end
end
xlabel('Benzetim zamanı [s]');

% --- Şekil 6: hız kontrolcüsü referans takibi ---------------------------------
f6 = namedFig('vio_ctl_vel', 'Hız kontrolcüsü: referans takibi (NED)', [180 80 900 800]);
for i = 1:3
    ax = subplot(3, 1, i); hold(ax, 'on'); grid(ax, 'on');
    h1 = plot(ax, vt, cvs(:, i), '--', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.2);
    h2 = plot(ax, vt, cvf(:, i), 'Color', [0.00 0.45 0.74], 'LineWidth', 1.0);
    ylabel(ax, vellbl{i});
    if i == 1, growTop(ax, 0.35); end
    addModePatches(ax, vt, fus);
    if i == 1
        legend(ax, [h1 h2], {'REFERANS', 'GERİ BESLEME'}, ...
               'Location', 'northeast', 'FontSize', 11);
        title(ax, 'Hız kontrolcüsü referans takibi');
    end
end
xlabel('Benzetim zamanı [s]');

% Kullanıcının şekil biçimlendirmesi ve etkileşimli imleç (varsa).
hf = [f1 f2 f3 f4 f5 f6];
try, format_all_figures(hf); catch ME
    fprintf('format_all_figures skipped: %s\n', ME.message);
end
try, vertical_line(hf); catch ME
    fprintf('vertical_line skipped: %s\n', ME.message);
end

% --- Tez çıktıları ------------------------------------------------------------
imgdir = '/home/teymur/ytu_thesis/thesis/master_ytu/thesisChapters/images';
if isfolder(imgdir)
    exportgraphics(f1, fullfile(imgdir, 'fig_vio_pos_ned.png'),   'Resolution', 150);
    exportgraphics(f2, fullfile(imgdir, 'fig_vio_vel_ned.png'),   'Resolution', 150);
    exportgraphics(f3, fullfile(imgdir, 'fig_vio_err_surat.png'), 'Resolution', 150);
    exportgraphics(f4, fullfile(imgdir, 'fig_vio_yorunge.png'),   'Resolution', 150);
    exportgraphics(f5, fullfile(imgdir, 'fig_vio_track_pos.png'), 'Resolution', 150);
    exportgraphics(f6, fullfile(imgdir, 'fig_vio_track_vel.png'), 'Resolution', 150);
    fprintf('Tez şekilleri kaydedildi: %s\n', imgdir);
    if ~isempty(fus_stats)
        writeVioMetricsTex(fullfile(fileparts(imgdir), 'vio_sonuc_metrikleri.tex'), fus_stats);
        fprintf('Metrik tablosu yazıldı: vio_sonuc_metrikleri.tex\n');
    end
end
end


function f = namedFig(tag, name, pos)
% Aynı etiketli şekil penceresini yeniden kullanır (pencere çoğalmasın).
f = findobj('Type', 'figure', 'Tag', tag);
if isempty(f)
    f = figure('Tag', tag);
else
    f = f(1); clf(f);
end
set(f, 'Name', name, 'NumberTitle', 'off', 'Color', 'w', 'Position', pos);
figure(f);
end


function growTop(ax, frac)
% Lejant için üstte boşluk: üst ylim'i aralığın frac katı kadar büyüt.
yl = ylim(ax);
ylim(ax, [yl(1), yl(2) + frac * diff(yl)]);
end


function addModePatches(ax, tt, fus)
% run_interactive.m/addModePatches ile birebir aynı.
tt = tt(:); fus = logical(fus(:));
if isempty(tt), return; end
edges = [1; find(diff(fus) ~= 0) + 1; numel(tt) + 1];
yl = ylim(ax);
span = max(tt(end) - tt(1), eps);
for k = 1:numel(edges) - 1
    i0 = edges(k); i1 = edges(k + 1) - 1;
    if fus(i0)
        c = [1.00 0.85 0.70]; name = 'VIO FÜZYON (GNSS KAPALI)';
    else
        c = [0.80 0.88 1.00]; name = 'VIO GÖZLEMCİ';
    end
    p = patch(ax, [tt(i0) tt(i1) tt(i1) tt(i0)], [yl(1) yl(1) yl(2) yl(2)], ...
              c, 'FaceAlpha', 0.35, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    uistack(p, 'bottom');
    if tt(i1) - tt(i0) > 0.04 * span
        text(ax, (tt(i0) + tt(i1)) / 2, mean(yl), name, ...
             'HorizontalAlignment', 'center', 'FontSize', 13, ...
             'FontWeight', 'bold', 'Color', [0.25 0.25 0.25], ...
             'Clipping', 'on', 'HandleVisibility', 'off');
    end
    if k > 1
        xline(ax, tt(i0), ':', 'Color', [0.35 0.35 0.35], 'LineWidth', 0.8, ...
              'HandleVisibility', 'off');
    end
end
ylim(ax, yl);
end


function writeVioMetricsTex(path, s)
% Füzyon fazı hata istatistiklerini Bölüm 8'in \input ettiği tabloya yazar.
fid = fopen(path, 'w', 'n', 'UTF-8');
if fid < 0
    warning('Metrik tablosu yazılamadı: %s', path);
    return;
end
fprintf(fid, '%% Otomatik üretildi: replot_vio_figures.m\n');
fprintf(fid, '\\begin{table}[htbp]\n    \\centering\n');
fprintf(fid, ['    \\caption{VIO füzyon fazında (GNSS kapalı, %.1f~s) ' ...
              'gerçek değere göre konum hatası istatistikleri.}\n'], s.dur);
fprintf(fid, '    \\label{tab:vio_fuzyon_hata}\n');
fprintf(fid, '    \\renewcommand{\\arraystretch}{1.3}\n');
% Tam ızgara (tezdeki diğer tablolarla tutarlı): |l|c|c|c| + her satırda \hline.
fprintf(fid, '    \\begin{tabular}{|l|c|c|c|}\n        \\hline\n');
fprintf(fid, ['        \\textbf{Kestirim} & \\textbf{RMSE [m]} & ' ...
              '\\textbf{Ortalama [m]} & \\textbf{Maksimum [m]} \\\\\n        \\hline\n']);
fprintf(fid, '        VIO & %.2f & %.2f & %.2f \\\\\n        \\hline\n', ...
        s.rmse_vio, s.mean_vio, s.max_vio);
fprintf(fid, '        EKF (VIO füzyonu, GNSS kapalı) & %.2f & %.2f & %.2f \\\\\n        \\hline\n', ...
        s.rmse_ekf, s.mean_ekf, s.max_ekf);
if isfinite(s.obs_rmse_ekf)
    fprintf(fid, '        EKF (gözlemci fazı, INS/GNSS) & %.2f & %.2f & %.2f \\\\\n        \\hline\n', ...
            s.obs_rmse_ekf, s.obs_mean_ekf, s.obs_max_ekf);
end
fprintf(fid, '    \\end{tabular}\n\\end{table}\n');
fclose(fid);
end
