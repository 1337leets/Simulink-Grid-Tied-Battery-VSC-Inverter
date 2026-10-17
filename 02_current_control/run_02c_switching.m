%% run_02c_switching.m
% Stage 2c: does the stage 2b controller work on the switching bridge?
% Runs the stage 2c test, the same test on the stage 2b averaged model for
% comparison, and a short stage 2c run with single samples instead of the
% carrier-period average.

here = fileparts(mfilename('fullpath'));
addpath(here);
m2b = 'vsi_02b_current_ctrl'; m2c = 'vsi_02c_switching';
load_system(fullfile(here, [m2b '.slx'])); load_system(fullfile(here, [m2c '.slx']));

tic
outB = sim(Simulink.SimulationInput(m2b), 'ShowProgress', 'off');
outC = sim(Simulink.SimulationInput(m2c), 'ShowProgress', 'off');
outS = sim(Simulink.SimulationInput(m2c).setVariable('b_avg', 1).setVariable('k_delay', 1.5) ...
    .setVariable('Tstop', 0.10), 'ShowProgress', 'off');
toc
params_02c_switching;      % the workspace values used by the stage 2c runs

B = collect(outB); C = collect(outC); S = collect(outS);

%% Table: switching vs averaged
fprintf('\n--- Stage 2c: switching bridge, same controller as stage 2b ---\n');
win = [0.08 0.10; 0.13 0.15; 0.18 0.20; 0.23 0.25; 0.31 0.34; 0.37 0.40];
lbl = {'P* = 10 kW', 'Q* = -4 kvar', 'after phase jump', 'P* = 15 kW (limit)', 'Vdc = 645 V', 'recovered'};
fprintf('%-20s %-24s %-24s %s\n', '', 'switching (2c)', 'averaged (2b)', 'i_dq switching [A]');
for k = 1:size(win, 1)
    [pc, qc] = pq(C, win(k, :)); [pb, qb] = pq(B, win(k, :));
    ic = mean(C.imeas(C.ti >= win(k,1) & C.ti < win(k,2), :), 1);
    fprintf('%-20s P = %6.2f, Q = %+6.2f   P = %6.2f, Q = %+6.2f   [%5.1f %+5.1f]\n', lbl{k}, pc, qc, pb, qb, ic);
end
fprintf('(P in kW, Q in kvar, averaged over each window from the raw PCC waveforms)\n');

%% Current quality at 10 kW (one grid cycle, 0.08-0.10 s)
k = C.tp >= 0.08 & C.tp < 0.10;
h = harmonics(C.i(k, :), Ts, fg);
r = C.i(k, 1) - movmean(C.i(k, 1), round(Tc/Ts));     % switching ripple
fprintf('At 10 kW: fundamental %.1f A peak; THD %.1f %% = low-order (h2-h40) %.1f %% + switching band %.1f %%; ripple %.1f A pk-pk\n', ...
    h.I1, h.thd, h.low, h.sw, max(r) - min(r));
fprintf('Peak phase current: switching %.1f A, averaged %.1f A (the %.1f A limit applies to the average current)\n', ...
    max(abs(C.i(C.tp > 0.03, :)), [], 'all'), max(abs(B.i(B.tp > 0.03, :)), [], 'all'), I_max);
fprintf('Start-up (PWM enabled at %.0f ms): peak phase current before 20 ms %.1f A (averaged model %.1f A)\n', ...
    1e3*t_en, max(abs(C.i(C.tp < 0.02, :)), [], 'all'), max(abs(B.i(B.tp < 0.02, :)), [], 'all'));

%% Carrier-period average vs single sample
for x = {'period average', C; 'single sample', S}'
    s = x{2}; [p, q] = pq(s, [0.08 0.10]); hs = harmonics(s.i(s.tp >= 0.08 & s.tp < 0.10, :), Ts, fg);
    kk = s.ti >= 0.08 & s.ti < 0.10;
    fprintf('%-15s: controller sees v_d = %3.0f V, i_d* = %4.1f A, i_d = %4.1f A | P = %5.2f kW, Q = %+5.2f kvar, low-order THD %.1f %%, voltage limit active %.0f %%\n', ...
        x{1}, mean(s.vdq(kk, 1)), mean(s.iref(kk, 1)), mean(s.imeas(kk, 1)), p, q, hs.low, 100*mean(s.lim(kk)));
end
fprintf('(PCC voltage peak in the averaged model: %.0f V)\n', mean(B.vdq(B.ti >= 0.08 & B.ti < 0.10, 1)));

%% Figure
fig = figure('Name', 'Stage 2c - switching model', 'Position', [100 40 950 950]);
try theme(fig, 'light'); catch, end
tl = tiledlayout(5, 1, 'TileSpacing', 'compact');
title(tl, 'Stage 2c: switching bridge vs averaged model (same controller, same test)');
nT = round(Tc/Ts); ga = [0.4 0.4 0.4];
nexttile; plot(C.tp, movmean(C.p, nT)/1e3, C.tp, movmean(C.q, nT)/1e3); hold on
plot(B.tp, B.p/1e3, ':', B.tp, B.q/1e3, ':', 'Color', ga, 'LineWidth', 1.2); ylabel('[kW / kvar]'); grid on
legend('P switching', 'Q switching', 'averaged', 'Location', 'eastoutside')
nexttile; plot(C.ti, C.imeas); hold on; plot(B.ti, B.imeas, ':', 'Color', ga, 'LineWidth', 1.2); ylabel('i_{dq} [A]'); grid on
legend('i_d switching', 'i_q switching', 'averaged', 'Location', 'eastoutside')
nexttile; z = C.tp >= 0.090 & C.tp <= 0.096; zb = B.tp >= 0.090 & B.tp <= 0.096;
plot(C.tp(z)*1e3, C.i(z, :)); hold on; plot(B.tp(zb)*1e3, B.i(zb, :), ':', 'Color', ga, 'LineWidth', 1.2)
ylabel('i_{abc} [A]'); grid on; legend('a', 'b', 'c', 'averaged', 'Location', 'eastoutside'); xlabel('Time [ms]')
nexttile; va = filter(b_avg, 1, C.v(:, 1)); ts = (0.090:Tc:0.096)';
plot(C.tp(z)*1e3, C.v(z, 1), 'Color', [0.75 0.75 0.75]); hold on; plot(C.tp(z)*1e3, va(z), 'b');
plot(ts*1e3, interp1(C.tp, C.v(:, 1), ts), 'ro', ts*1e3, interp1(C.tp, va, ts), 'bo');
ylabel('v_{a,PCC} [V]'); grid on; xlabel('Time [ms]')
legend('raw', 'period average', 'single samples', 'averaged samples', 'Location', 'eastoutside')
nexttile; stairs(C.ti, C.Vdc); hold on; stairs(C.ti, 650 + 30*C.lim); xlim([0.26 0.40])
ylabel('[V]'); xlabel('Time [s]'); legend('V_{dc} measured', 'limit active', 'Location', 'eastoutside'); grid on

%% ---------------- helpers ----------------
function s = collect(out)
L = out.logsout; g = @(n) L.get(n).Values;
s.v = asCols(g('Vabc_pcc')); s.i = asCols(g('Iabc_pcc')); s.tp = g('Vabc_pcc').Time;
s.p = sum(s.v.*s.i, 2);
s.q = ((s.v(:,2)-s.v(:,3)).*s.i(:,1) + (s.v(:,3)-s.v(:,1)).*s.i(:,2) + (s.v(:,1)-s.v(:,2)).*s.i(:,3))/sqrt(3);
s.iref = asCols(g('idq_ref')); s.imeas = asCols(g('idq_meas')); s.ti = g('idq_meas').Time;
s.vdq = asCols(g('vdq')); s.lim = asCols(g('v_limit_active'));
s.Vdc = interp1(g('Vdc').Time, asCols(g('Vdc')), s.ti, 'previous');
end

function [P, Q] = pq(s, w)
k = s.tp >= w(1) & s.tp < w(2);
P = mean(s.p(k))/1e3; Q = mean(s.q(k))/1e3;
end

function h = harmonics(x, Ts, f0)
% x: whole grid cycles of the three phase currents; percentages of the fundamental, mean over phases
n = size(x, 1); X = abs(fft(x - mean(x)))/n*2; nc = round(n*Ts*f0);
a1 = X(nc + 1, :); lo = X(nc*(2:40) + 1, :);
hi = X(41*nc + 1:floor(n/2), :);
h.I1 = mean(a1);
h.low = 100*mean(sqrt(sum(lo.^2, 1))./a1);
h.sw  = 100*mean(sqrt(sum(hi.^2, 1))./a1);
h.thd = hypot(h.low, h.sw);
end

function d = asCols(ts)
d = squeeze(ts.Data);
if size(d, 1) ~= numel(ts.Time), d = d.'; end
d = double(d);
end
