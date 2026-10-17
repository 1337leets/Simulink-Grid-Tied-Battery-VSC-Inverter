%% run_03a_battery.m
% Step 3a: one (time-compressed) day of the BESS. The EMS charges 4 kW around
% midday and discharges 10 kW in the evening; the SOC limits of the BMS stop
% both before the battery is over- or under-charged, while Q* is still delivered.

here = fileparts(mfilename('fullpath'));
addpath(here);
m = 'vsi_03a_battery';
load_system(fullfile(here, [m '.slx']));

tic
out = sim(Simulink.SimulationInput(m), 'ShowProgress', 'off');
fprintf('Simulated 06:00-24:00 (%.2f s) in %.0f s\n', Tstop, toc);
params_03a_battery;        % the workspace values used by the run

%% Signals (hour of the day as time axis)
L = out.logsout; g = @(n) L.get(n).Values;
v = asCols(g('Vabc_pcc')); i = asCols(g('Iabc_pcc')); tp = g('Vabc_pcc').Time;
p = sum(v.*i, 2);
q = ((v(:,2)-v(:,3)).*i(:,1) + (v(:,3)-v(:,1)).*i(:,2) + (v(:,1)-v(:,2)).*i(:,3))/sqrt(3);
Vdc_ = asCols(g('Vdc')); Ib = asCols(g('I_bat')); tb = g('I_bat').Time;
SOC = asCols(g('SOC')); ts = g('SOC').Time;
Pems = asCols(g('PQ_ems')); te = g('PQ_ems').Time;
Pref = asCols(g('PQ_ref')); tc = g('PQ_ref').Time;
hour = @(t) h0 + t/T_h;
mw = @(x, t, w) mean(x(t >= hr(w(1)) & t < hr(w(2)), :), 1);        % mean over hours w

%% Table: snapshots during the day
fprintf('\n--- Step 3a: one day of the BESS (P > 0 discharge, P < 0 charge) ---\n');
fprintf('%-32s %7s %7s %7s %7s %6s %6s %7s\n', 'hour', 'P*_EMS', 'P*_BMS', 'P', 'Q', 'SOC', 'Vdc', 'I_bat');
snap = {[7.0 7.5], 'idle'; [10.0 10.5], 'charging'; [13.5 14.0], 'full, Q* = -3 kvar'
        [17.0 17.5], 'idle'; [18.75 19.25], 'discharging'; [21.0 21.5], 'empty, Q* = +4 kvar'};
for k = 1:size(snap, 1)
    w = snap{k, 1}; pe = mw(Pems, te, w); pr = mw(Pref, tc, w);
    fprintf('%s-%s %-20s %7.2f %7.2f %7.2f %+7.2f %5.1f%% %6.0f %+7.1f\n', hhmm(w(1)), hhmm(w(2)), snap{k, 2}, ...
        pe(1)/1e3, pr(1)/1e3, mw(p, tp, w)/1e3, mw(q, tp, w)/1e3, 100*mw(SOC, ts, w), mw(Vdc_, tb, w), mw(Ib, tb, w));
end
fprintf('(P in kW, Q in kvar, Vdc in V, I_bat in A; P and Q from the raw PCC waveforms, averaged over each window)\n');

%% SOC limits
kf = find(SOC >= SOC_max - 0.005, 1); ke = find(SOC <= SOC_min + 0.005, 1);
fprintf('SOC: start %.1f %%, 89.5 %% at %s, max %.2f %%; 10.5 %% at %s, min %.2f %%; end %.2f %%\n', ...
    100*SOC(1), hhmm(hour(ts(kf))), 100*max(SOC), hhmm(hour(ts(ke))), 100*min(SOC(ts > hr(18))), 100*SOC(end));

%% Energy balance (kWh of the real day: P*dt/T_h)
% Time compression makes the battery voltage fall 72000x faster than in reality,
% so the DC-link capacitor releases C*dV/dt (~0.3 A, ~200 W at 10 kW discharge).
% That is an artefact of the compression; it is removed from the DC-side power
% with the capacitor energy 0.5*C*V^2 at the window edges.
dt  = Ts/T_h/1e3;                                   % sample [s] -> real hours, W -> kW
Pdc = Vdc_.*Ib;                                     % battery terminal power
OCV = interp1(SOC_bp, V0_bat, SOC);                 % open-circuit voltage
Pch = OCV.*Ib;                                      % chemical power (OCV * i)
nT  = round(Tc/Ts);
Ecap = @(kb) 0.5*C_dc*(mean(Vdc_(find(kb, nT, 'last')))^2 - mean(Vdc_(find(kb, nT)))^2);   % [J]
fprintf(['Energy [kWh] (+ = towards the grid)   grid  battery terminals  chemical (OCV*i)' ...
    ' | losses: inverter  battery R0 | capacitor (artefact)\n']);
for x = {'charge', [9 18]; 'discharge', [18 24]}'
    k = tp >= hr(x{2}(1)) & tp < hr(x{2}(2)); kb = tb >= hr(x{2}(1)) & tb < hr(x{2}(2));
    Eac = sum(p(k))*dt; Ebt = sum(Pdc(kb))*dt; Ech = sum(Pch(kb))*dt; Ec = -Ecap(kb)/T_h/1e3;
    fprintf('  %-9s %s-%s             %+6.2f      %+6.2f           %+6.2f   |      %5.2f       %5.2f   |   %+5.2f\n', ...
        x{1}, hhmm(x{2}(1)), hhmm(x{2}(2)), Eac, Ebt, Ech, Ebt + Ec - Eac, Ech - Ebt, Ec);
end
w = [18.75 19.25]; kb = tb >= hr(w(1)) & tb < hr(w(2));
Pbr = mean(Pdc(kb)) - Ecap(kb)/(hr(w(2)) - hr(w(1)));
fprintf('Inverter efficiency at 10 kW discharge: %.1f %% (losses %.0f W: Rf %.0f W, rest IGBT/diode drops)\n', ...
    100*mw(p, tp, w)/Pbr, Pbr - mw(p, tp, w), Rf*mw(sum(i.^2, 2), tp, w));

%% DC-link ripple at 10 kW
k = tb >= hr(19) & tb < hr(19) + 0.02;
fprintf('At 10 kW: Vdc ripple %.1f V pk-pk, battery current %.1f A mean, %.1f A pk-pk ripple\n', ...
    max(Vdc_(k)) - min(Vdc_(k)), mean(Ib(k)), max(Ib(k)) - min(Ib(k)));

%% Figure
fig = figure('Name', 'Step 3a - battery day', 'Position', [100 40 950 900]);
try theme(fig, 'light'); catch, end
tl = tiledlayout(4, 1, 'TileSpacing', 'compact');
title(tl, 'Step 3a: one day of the BESS (time-compressed, 1 h = 50 ms)');
nexttile; plot(hour(te), Pems(:, 1)/1e3, 'k:', hour(tc), Pref(:, 1)/1e3, 'k--', 'LineWidth', 1.2); hold on
plot(hour(tp), movmean(p, nT)/1e3, hour(tp), movmean(q, nT)/1e3);
ylabel('[kW / kvar]'); grid on; legend('P* EMS', 'P* after SOC limits', 'P', 'Q', 'Location', 'eastoutside')
nexttile; plot(hour(ts), 100*SOC); yline(100*[SOC_min SOC_max], '--'); ylim([0 100])
ylabel('SOC [%]'); grid on; legend('SOC', 'limits', 'Location', 'eastoutside')
nexttile; plot(hour(tb), movmean(Vdc_, nT)); ylabel('V_{dc} [V]'); grid on
legend('V_{dc} (period mean)', 'Location', 'eastoutside')
nexttile; plot(hour(tb), movmean(Ib, nT)); ylabel('i_{bat} [A]'); grid on; xlabel('Hour of the day')
legend('i_{bat} (period mean)', 'Location', 'eastoutside')
for ax = findobj(fig, 'Type', 'axes')', xlim(ax, [h0 24]); xticks(ax, h0:2:24); end

%% ---------------- helpers ----------------
function s = hhmm(h)
s = sprintf('%02d:%02d', floor(h), round(60*(h - floor(h))));
end

function d = asCols(ts)
d = squeeze(ts.Data);
if size(d, 1) ~= numel(ts.Time), d = d.'; end
d = double(d);
end
