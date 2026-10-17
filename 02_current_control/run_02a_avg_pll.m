%% run_02a_avg_pll.m
% Stage 2a: does the PLL track the grid, and does a PLL-synchronised reference
% keep P at its set-point after a phase jump and a frequency step?

here = fileparts(mfilename('fullpath'));
addpath(here);
mdl = 'vsi_02a_avg_pll';
params_02a_avg_pll;
out = sim(fullfile(here, [mdl '.slx']));
L = out.logsout;

Vp  = L.get('Vabc_pcc').Values;  th = L.get('theta_pll').Values;
fp  = L.get('f_pll').Values;     vdq = L.get('vdq').Values;
P   = L.get('P').Values;         Q  = L.get('Q').Values;

% Measured PCC voltage angle: phase a = V sin(theta)  ->  theta = atan2(v_alpha, -v_beta)
% compared at the controller sample instants (theta_pll is held between samples)
v = asCols(Vp);
valpha = (2/3)*(v(:,1) - 0.5*v(:,2) - 0.5*v(:,3));
vbeta  = (v(:,2) - v(:,3))/sqrt(3);
th_pcc = interp1(Vp.Time, unwrap(atan2(valpha, -vbeta)), th.Time);
err_deg = rad2deg(angle(exp(1j*(th_pcc - asCols(th)))));
tc = th.Time;

at = @(ts, t) interp1(ts.Time, asCols(ts), t, 'previous');
fprintf('\n--- Stage 2a: averaged inverter + SRF-PLL ---\n');
fprintf('PLL gains: Kp = %.1f, Ki = %.0f  (f_n = %d Hz, zeta = %.3f)\n', Kp_pll, Ki_pll, f_pll, zeta_pll);
for t = [t_phase - 0.005, t_freq - 0.005, Tstop - 0.005]
    d = at(vdq, t);
    fprintf('t = %.3f s:  f_pll = %.3f Hz,  vd = %.1f V, vq = %+.2f V,  P = %.0f W, Q = %+.0f var,  angle error = %+.2f deg\n', ...
        t, at(fp, t), d(1), d(2), at(P, t), at(Q, t), interp1(tc, err_deg, t, 'previous'));
end
win = tc > t_phase & tc < t_freq;
fprintf('Phase jump: peak angle error %.1f deg; error < 1 deg after %.1f ms\n', max(abs(err_deg(win))), ...
    1e3*(tc(find(win & abs(err_deg) > 1, 1, 'last')) - t_phase));

figure('Name', 'Stage 2a - PLL', 'Position', [100 100 900 760]);
tl = tiledlayout(4, 1, 'TileSpacing', 'compact');
title(tl, sprintf('SRF-PLL (f_n = %d Hz): +%d^\\circ phase jump at %.2f s, +%.1f Hz at %.2f s', ...
    f_pll, dphi_deg, t_phase, df_step, t_freq));
nexttile; plot(fp.Time, asCols(fp)); yline(fg, ':'); yline(fg + df_step, ':');
ylabel('f_{PLL} [Hz]'); grid on
nexttile; plot(tc, err_deg); ylabel('\theta_{PCC} - \theta_{PLL} [deg]'); grid on
nexttile; plot(vdq.Time, asCols(vdq)); ylabel('[V]'); legend('v_d', 'v_q', 'Location', 'east'); grid on
nexttile; plot(P.Time, asCols(P)/1e3, Q.Time, asCols(Q)/1e3); yline(P_ref/1e3, '--');
ylabel('[kW / kvar]'); xlabel('Time [s]'); legend('P', 'Q', 'P_{ref}', 'Location', 'east'); grid on

function d = asCols(ts)
d = squeeze(ts.Data);
if size(d, 1) ~= numel(ts.Time), d = d.'; end
end
