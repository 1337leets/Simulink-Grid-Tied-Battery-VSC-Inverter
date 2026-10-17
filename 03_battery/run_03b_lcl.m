%% run_03b_lcl.m
% Step 3b: what the LCL filter buys, and what it costs.
% Same short test for every case: P* ramp to 10 kW (0.02-0.05 s), Q* = -4 kvar at 0.12 s.
%   L filter   step 3a model (reference)
%   A  grid-side current feedback, no damping       -> stable (f_res > f_crit)
%   B  grid-side current feedback, Rd = Rd_opt      -> passive damping (model default)
%   C  inverter-side current feedback, no damping   -> unstable (f_res > f_crit)
%   D  inverter-side feedback, Rd = Rd_opt, no Q compensation -> Q off by Q_C
%   E  inverter-side feedback, Rd = Rd_opt, Q compensation

here = fileparts(mfilename('fullpath'));
addpath(here);
m3a = 'vsi_03a_battery'; m = 'vsi_03b_lcl';
load_system(fullfile(here, [m3a '.slx'])); load_system(fullfile(here, [m '.slx']));
params_03b_lcl;            % test profile and filter values used below

short = @(in) in.setVariable('T_h', 3600).setVariable('AH_sim', AH_bat).setVariable('SOC0', SOC0) ...
    .setVariable('Vdc', Vdc).setVariable('Tstop', Tstop).setVariable('t_P', t_P).setVariable('P_P', P_P) ...
    .setVariable('t_Q', t_Q).setVariable('Q_Q', Q_Q);
cas = @(fb, rd, qc) Simulink.SimulationInput(m).setVariable('fb_side', fb).setVariable('Rd', rd).setVariable('q_comp', qc);
runs = {'L filter (3a)',            short(Simulink.SimulationInput(m3a))
        'A grid fb, Rd = 0',        cas(1, Rd_min, 0)
        'B grid fb, Rd = Rd_opt',   cas(1, Rd_opt, 0)
        'C inverter fb, Rd = 0',    cas(2, Rd_min, 0).setVariable('Tstop', 0.10)
        'D inv. fb, Rd, no comp.',  cas(2, Rd_opt, 0)
        'E inv. fb, Rd, Q comp.',   cas(2, Rd_opt, 1)};
tic
out = cellfun(@(in) sim(in, 'ShowProgress', 'off'), runs(:, 2), 'UniformOutput', false);
fprintf('%d runs in %.0f s\n', numel(out), toc);
R = cellfun(@collect, out);

%% Table
fprintf('\n--- Step 3b: LCL filter (f_res = %.0f Hz, f_crit = 1/(8 Tc) = %.0f Hz, Rd_opt = %.1f Ohm) ---\n', f_res, f_crit, Rd_opt);
fprintf('%-26s %-15s %-15s %8s %8s %8s %8s %8s\n', '', 'P*=10 kW', 'Q*=-4 kvar', 'THD', 'low', 'sw', 'res.', 'pk |i|');
for k = 1:numel(R)
    s = R(k);
    [p1, q1] = pq(s, [0.08 0.10]); [p2, q2] = pq(s, [0.17 0.19]);
    kk = s.t >= 0.08 & s.t < 0.10; h = harmonics(s.i(kk, :), Ts, fg, f_res);
    pk = max(abs(s.i(s.t > 0.03, :)), [], 'all');
    if s.t(end) < 0.15, p2 = NaN; q2 = NaN; end
    fprintf('%-26s %5.2f %+6.2f   %5.2f %+6.2f   %6.1f%% %6.1f%% %6.2f%% %6.1f%% %6.1f A\n', ...
        runs{k, 1}, p1, q1, p2, q2, h.thd, h.low, h.sw, h.res, pk);
end
fprintf('(P kW, Q kvar at the PCC; THD of the grid current at 10 kW = low order h2-h40 + switching band;\n');
fprintf(' res. = component within +-300 Hz of f_res; pk = peak phase current after 30 ms)\n');

% damping losses and the ripple the capacitor takes
s = R(3); kk = s.t >= 0.08 & s.t < 0.10; ic = s.ii(kk, :) - s.i(kk, :);
fprintf('B: losses in Rd %.0f W (%.2f %% of 10 kW), capacitor current %.2f A rms (50 Hz part %.2f A)\n', ...
    Rd_opt*mean(sum(ic.^2, 2)), Rd_opt*mean(sum(ic.^2, 2))/100, sqrt(mean(ic(:, 1).^2)), Vg_ph*wg*Cf);
r = @(x) max(x) - min(x);
fprintf('Ripple at 10 kW (pk-pk, phase a): inverter side %.1f A, grid side %.2f A (L filter: %.1f A)\n', ...
    r(s.ii(kk, 1) - movmean(s.ii(kk, 1), round(Tc/Ts))), r(s.i(kk, 1) - movmean(s.i(kk, 1), round(Tc/Ts))), ...
    r(R(1).i(kk, 1) - movmean(R(1).i(kk, 1), round(Tc/Ts))));
fprintf('Cf generates Q_C = %.0f var at the grid voltage (the Q compensation of case E)\n', Q_C);

%% Figure
fig = figure('Name', 'Step 3b - LCL filter', 'Position', [100 40 1000 900]);
try theme(fig, 'light'); catch, end
tl = tiledlayout(3, 2, 'TileSpacing', 'compact');
title(tl, sprintf('Step 3b: LCL filter (resonance %.0f Hz)', f_res));
z = @(s) s.t >= 0.090 & s.t <= 0.100;
for k = [1 2 3]
    nexttile; s = R(k); plot(s.t(z(s))*1e3, s.i(z(s), :)); grid on
    title(runs{k, 1}, 'Interpreter', 'none'); ylabel('i_{grid} [A]'); xlabel('Time [ms]'); ylim([-30 30]);
end
nexttile; s = R(4); plot(s.t*1e3, s.i); grid on      % the unstable case over the whole run
title(runs{4, 1}, 'Interpreter', 'none'); ylabel('i_{grid} [A]'); xlabel('Time [ms]');
nexttile([1 2]); hold on
for k = [1 2 3]
    s = R(k); kk = s.t >= 0.08 & s.t < 0.10; x = s.i(kk, 1); n = numel(x);
    X = abs(fft(x - mean(x)))/n*2; f = (0:n-1)'/(n*Ts);
    semilogy(f(f <= 12e3)/1e3, max(X(f <= 12e3), 1e-3)/max(X)*100);
end
set(gca, 'YScale', 'log'); grid on; xline(f_res/1e3, ':', 'f_{res}'); xline(fsw/1e3, ':', 'f_{sw}');
xlabel('Frequency [kHz]'); ylabel('% of fundamental'); ylim([1e-3 100]);
legend(runs{[1 2 3], 1}, 'Location', 'eastoutside', 'Interpreter', 'none'); title('Grid current spectrum at 10 kW');

%% ---------------- helpers ----------------
function s = collect(out)
L = out.logsout; g = @(n) L.get(n).Values;
s.v = asCols(g('Vabc_pcc')); s.i = asCols(g('Iabc_pcc')); s.t = g('Vabc_pcc').Time;
s.p = sum(s.v.*s.i, 2);
s.q = ((s.v(:,2)-s.v(:,3)).*s.i(:,1) + (s.v(:,3)-s.v(:,1)).*s.i(:,2) + (s.v(:,1)-s.v(:,2)).*s.i(:,3))/sqrt(3);
if any(strcmp(L.getElementNames, 'Iabc_inv')), s.ii = asCols(g('Iabc_inv')); else, s.ii = s.i; end
end

function [P, Q] = pq(s, w)
k = s.t >= w(1) & s.t < w(2);
P = mean(s.p(k))/1e3; Q = mean(s.q(k))/1e3;
end

function h = harmonics(x, Ts, f0, fr)
% x: whole grid cycles of the three phase currents; percentages of the fundamental, mean over phases
n = size(x, 1); X = abs(fft(x - mean(x)))/n*2; nc = round(n*Ts*f0); df = 1/(n*Ts);
a1 = X(nc + 1, :);
lo = X(nc*(2:40) + 1, :); hi = X(41*nc + 1:floor(n/2), :);
rs = X(round((fr - 300)/df) + 1:round((fr + 300)/df) + 1, :);
h.low = 100*mean(sqrt(sum(lo.^2, 1))./a1);
h.sw  = 100*mean(sqrt(sum(hi.^2, 1))./a1);
h.res = 100*mean(sqrt(sum(rs.^2, 1))./a1);
h.thd = hypot(h.low, h.sw);
end

function d = asCols(ts)
d = squeeze(ts.Data);
if size(d, 1) ~= numel(ts.Time), d = d.'; end
d = double(d);
end
