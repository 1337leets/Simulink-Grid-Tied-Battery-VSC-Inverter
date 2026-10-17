%% params_03b_lcl.m
% Step 3b: LCL filter instead of the L filter of step 3a.
%
% Project definition: 10 kW grid-tied battery energy storage system (BESS),
% 400 V / 50 Hz grid, battery directly on the DC link (~700 V), grid-following.
% An EMS sends P*/Q* (P > 0: discharge into the grid, P < 0: charge); the
% inverter delivers them within its current limit (1.1 pu) and the SOC limits
% of the BMS (10-90 %).

% Battery, BMS limits, controller, PWM and measurements are shared with step 3a
run(fullfile(fileparts(mfilename('fullpath')), 'params_03a_battery.m'));

%% Short test instead of the day profile (no time compression)
T_h    = 3600;              % 1 h = 3600 s: real time
AH_sim = AH_bat;
SOC0   = 0.5;
Vdc    = interp1(SOC_bp, V0_bat, SOC0);
Tstop  = 0.20;
e = 1e-6;
t_P = [0  0.02  0.05  1];       P_P = [0  0  10e3  10e3];     % ramp to 10 kW
t_Q = [0  0.12  0.12+e  1];     Q_Q = [0  0  -4e3  -4e3];     % absorb 4 kvar

%% LCL filter
% The L filter (Lf = 5 mH) is split into an inverter-side L1 and a grid-side L2
% with a star-connected capacitor in between. L1 + L2 = Lf and R1 + R2 = Rf, so
% at 50 Hz the plant of the current controller is unchanged (same gains).
% Cf is 5 % of the base capacitance: it generates ~0.5 kvar at the grid voltage.
% The switching ripple of i1 flows mostly into Cf; L2 + Lg attenuates the rest
% by 1/|1 - w^2*(L2 + Lg)*Cf| (about 1/29 at fsw).
L1 = 3e-3;   R1 = 0.3;          % inverter side [H, Ohm]
L2 = 2e-3;   R2 = 0.2;          % grid side [H, Ohm]
Cf = 10e-6;                     % filter capacitor per phase, star [F]
% Resonance (the grid inductance Lg adds to L2): ~1.3 kHz, between 10*fg and fsw/2
w_res = sqrt((L1 + L2 + Lg)/(L1*(L2 + Lg)*Cf));
f_res = w_res/(2*pi);
Q_C   = 3*Vg_ph^2*wg*Cf;        % reactive power of Cf at the grid voltage [var]

%% Damping and current feedback (the cases compared in run_03b_lcl)
% Resonance vs. delay: with a total loop delay of 2*Tc the critical frequency is
% f_crit = 1/(8*Tc) = 625 Hz (1/(6*Tc) for the usual 1.5*Tc). Above it (here: 1.3 kHz)
%   - grid-side current feedback (fb_side = 1) is stable without damping: the
%     delay turns the loop into an "inherent" active damping,
%   - inverter-side current feedback (fb_side = 2) is unstable without damping.
% (Below f_crit it is the other way round.) Passive damping: Rd in series with
% Cf, Rd = 1/(3*w_res*Cf) is the usual choice; it costs ~30 W here and keeps
% the filter stable whatever the feedback and the grid inductance.
f_crit  = 1/(8*Tc);
Rd_opt  = 1/(3*w_res*Cf);       % [Ohm] (~4 Ohm)
Rd_min  = 1e-3;                 % 'undamped' (the series RC block needs R > 0) [Ohm]
Rd      = Rd_opt;               % damping resistor [Ohm]
fb_side = 1;                    % current fed back: 1 = grid side (PCC), 2 = inverter side
% With inverter-side feedback the controller regulates i1 = i2 + i_C: Q at the
% PCC comes out Q_C too high. The EMS path lowers Q* by Q_C when q_comp = 1
% (feedforward). Grid-side feedback needs no correction.
q_comp  = 1;
