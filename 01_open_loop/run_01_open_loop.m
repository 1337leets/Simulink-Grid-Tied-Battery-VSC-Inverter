%% run_01_open_loop.m
% Simulates vsi_01_open_loop.slx and compares the measured steady-state P/Q
% at the PCC with the open-loop targets from params_01_open_loop.m.

here = fileparts(mfilename('fullpath'));
addpath(here);
mdl = 'vsi_01_open_loop';
params_01_open_loop;
out = sim(fullfile(here, [mdl '.slx']));
logs = out.logsout;

P    = logs.get('P').Values;
Q    = logs.get('Q').Values;
Vpcc = logs.get('Vabc_pcc').Values;
Ipcc = logs.get('Iabc_pcc').Values;
G    = logs.get('gates').Values;      % [a+ a- b+ b- c+ c-]

% Steady-state window: last two fundamental cycles
tw = Tstop - 2/fg;
ss   = @(ts) mean(winData(ts, tw), 1);
rmsw = @(ts) sqrt(mean(winData(ts, tw).^2, 1));

fprintf('\n--- Open-loop operating point ---\n');
fprintf('Modulation index m_a = %.3f, delta = %.2f deg\n', m_a, rad2deg(delta));
fprintf('P: target %8.0f W,   measured %8.0f W\n', P_ref, ss(P));
fprintf('Q: target %8.0f var, measured %8.0f var\n', Q_ref, ss(Q));
fprintf('I_pcc RMS (a,b,c) = %s A (expected %.1f A)\n', mat2str(rmsw(Ipcc), 4), abs(I_ph));

% Inverter line-to-line voltage reconstructed from the gate signals: v_ab = (S_a - S_b)*Vdc
g = double(asCols(G));
vab_inv = (g(:, 1) - g(:, 3))*Vdc;

figure('Name', 'Step 1 - Open-loop VSI', 'Position', [100 100 900 750]);
tl = tiledlayout(4, 1, 'TileSpacing', 'compact');
title(tl, sprintf('Open-loop VSI: P_{ref} = %.0f kW, Q_{ref} = %.0f kvar, m_a = %.2f', ...
    P_ref/1e3, Q_ref/1e3, m_a));
zw = G.Time >= Tstop - 2/fg;
nexttile; plot(G.Time(zw), vab_inv(zw));
ylabel('v_{ab,inv} [V]'); title('Inverter line-to-line voltage (PWM, from gate signals)'); grid on
nexttile; plot(Vpcc.Time, asCols(Vpcc));
ylabel('v_{abc,pcc} [V]'); title('PCC phase voltages'); grid on
nexttile; plot(Ipcc.Time, asCols(Ipcc));
ylabel('i_{abc} [A]'); title('Grid currents (positive = into the grid)'); grid on
nexttile; plot(P.Time, asCols(P)/1e3, Q.Time, asCols(Q)/1e3);
yline(P_ref/1e3, '--'); yline(Q_ref/1e3, ':');
ylabel('[kW / kvar]'); xlabel('Time [s]'); legend('P', 'Q', 'P_{ref}', 'Q_{ref}', 'Location', 'best');
title('Active / reactive power at PCC'); grid on

function d = asCols(ts)
% logged data as [time x channels], whatever the logging layout
d = squeeze(ts.Data);
if size(d, 1) ~= numel(ts.Time), d = d.'; end
end

function d = winData(ts, tw)
d = asCols(ts);
d = d(ts.Time >= tw, :);
end
