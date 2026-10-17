%% run_02b_current_ctrl.m
% Stage 2b: does the dq current loop deliver P*/Q* exactly, respect the
% current limit, and recover cleanly from the voltage limit? The test is run
% twice, with and without anti-windup.

here = fileparts(mfilename('fullpath'));
addpath(here);
mdl = 'vsi_02b_current_ctrl';
params_02b_current_ctrl;
load_system(fullfile(here, [mdl '.slx']));
in = [Simulink.SimulationInput(mdl), Simulink.SimulationInput(mdl).setVariable('aw_on', 0)];
out = sim(in, 'ShowProgress', 'off');

R = cell(1, 2);
for r = 1:2
    L = out(r).logsout;
    g = @(n) L.get(n).Values;
    s.Vp = g('Vabc_pcc'); s.Ip = g('Iabc_pcc');
    s.PQ = g('PQ'); s.PQr = g('PQ_ref'); s.iref = g('idq_ref'); s.imeas = g('idq_meas');
    s.lim = g('v_limit_active'); s.Vdc = g('Vdc');
    % P, Q from the raw plant waveforms (independent of the controller's dq math)
    v = asCols(s.Vp); i = asCols(s.Ip);
    s.tp = s.Vp.Time;
    s.p = sum(v.*i, 2);
    s.q = ((v(:,2)-v(:,3)).*i(:,1) + (v(:,3)-v(:,1)).*i(:,2) + (v(:,1)-v(:,2)).*i(:,3))/sqrt(3);
    s.ipk = max(abs(i), [], 2);
    R{r} = s;
end

s = R{1};
at = @(ts, t) interp1(ts.Time, asCols(ts), t, 'previous');
avg = @(x, t1, t2) mean(x(s.tp >= t1 & s.tp < t2, :), 1);
fprintf('\n--- Stage 2b: dq current control (anti-windup on) ---\n');
fprintf('PI: Kp = %.2f V/A, Ki = %.0f V/(A s), fc = %d Hz;  I_max = %.1f A peak\n', Kp_i, Ki_i, fc_i, I_max);
win = [0.08 0.10; 0.13 0.15; 0.18 0.20; 0.23 0.25; 0.31 0.34; 0.37 0.40];
lbl = {'P* = 10 kW', 'Q* = -4 kvar', 'after phase jump', 'P* = 15 kW (limit)', 'Vdc = 645 V', 'recovered'};
for k = 1:size(win, 1)
    t = win(k, 2) - 1e-4; pr = at(s.PQr, t); ir = at(s.iref, t); im = at(s.imeas, t);
    fprintf('%-20s P* = %5.1f kW, Q* = %+5.1f kvar | P = %6.2f kW, Q = %+6.2f kvar | i_dq* = [%5.1f %+5.1f], i_dq = [%5.1f %+5.1f] A\n', ...
        lbl{k}, pr(1)/1e3, pr(2)/1e3, avg(s.p, win(k,1), win(k,2))/1e3, avg(s.q, win(k,1), win(k,2))/1e3, ir, im);
end
fprintf('Peak phase current: whole run %.1f A, at the phase jump %.1f A (limit %.1f A)\n', max(s.ipk), ...
    max(s.ipk(s.tp > 0.15 & s.tp < 0.17)), I_max);
for r = 1:2
    e = asCols(R{r}.iref) - asCols(R{r}.imeas);
    k = R{r}.iref.Time > 0.34 & R{r}.iref.Time < 0.40;
    fprintf('Anti-windup %s: voltage limit active %.1f ms; after Vdc returns, peak current error %.1f A, peak phase current %.1f A\n', ...
        string(r == 1), 1e3*Tc*sum(asCols(R{r}.lim)), max(vecnorm(e(k, :), 2, 2)), max(R{r}.ipk(R{r}.tp > 0.34)));
end

figure('Name', 'Stage 2b - current control', 'Position', [100 60 950 900]);
tl = tiledlayout(5, 1, 'TileSpacing', 'compact');
title(tl, 'Stage 2b: P*/Q* tracking, current limit, phase jump, Vdc sag (anti-windup on vs off)');
nexttile; plot(s.tp, s.p/1e3, s.tp, s.q/1e3); hold on
stairs(s.PQr.Time, asCols(s.PQr)/1e3, 'k--'); ylabel('[kW / kvar]'); legend('P', 'Q', 'P*, Q*', 'Location', 'eastoutside'); grid on
nexttile; plot(s.iref.Time, asCols(s.iref), '--', s.imeas.Time, asCols(s.imeas)); ylabel('[A]');
legend('i_d*', 'i_q*', 'i_d', 'i_q', 'Location', 'eastoutside'); grid on
nexttile; plot(s.tp, asCols(s.Ip)); yline([I_max -I_max], 'k:'); ylabel('i_{abc} [A]'); grid on
legend('a', 'b', 'c', 'Location', 'eastoutside')
nexttile; plot(R{1}.imeas.Time, asCols(R{1}.imeas), R{2}.imeas.Time, asCols(R{2}.imeas), ':'); xlim([0.26 0.40])
ylabel('[A]'); legend('i_d AW on', 'i_q AW on', 'i_d AW off', 'i_q AW off', 'Location', 'eastoutside'); grid on
nexttile; stairs(s.Vdc.Time, asCols(s.Vdc)); hold on; stairs(s.lim.Time, 650 + 30*asCols(s.lim)); xlim([0.26 0.40])
ylabel('[V]'); xlabel('Time [s]'); legend('V_{dc}', 'limit active', 'Location', 'eastoutside'); grid on

function d = asCols(ts)
d = squeeze(ts.Data);
if size(d, 1) ~= numel(ts.Time), d = d.'; end
d = double(d);
end
