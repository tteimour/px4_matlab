% save_flight.m
% Uçuş BİTTİKTEN hemen sonra (disable VIO + simülasyonu durdurduktan sonra),
% HİÇBİR figürü elle yakınlaştırmadan MATLAB komut penceresinde çalıştırın:
%
%     save_flight
%
% Bu, o uçuşun ham logunu güvenli bir dosyaya kaydeder. Figürleri bu logdan
% temiz (tam aralıklı) olarak ben yeniden üreteceğim.

if ~exist('vio_log', 'var') || ~exist('vio_idx', 'var')
    error(['vio_log / vio_idx çalışma alanında yok. Bu scripti, uçuşu ' ...
           'yaptığınız MATLAB oturumunda (run_interactive sonrası) çalıştırın.']);
end

L = vio_log;
n = vio_idx;
outfile = '/home/teymur/git/px4_matlab/sim/vio_log_yeni.mat';
save(outfile, 'L', 'n');
fprintf('Uçuş logu kaydedildi: %s  (%d örnek)\n', outfile, n);
fprintf('Şimdi Claude''a "kaydettim" deyin; figürleri bu logdan üretecek.\n');
