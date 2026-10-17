%% params_02b_current_ctrl.m
% Stage 2b: averaged inverter + SRF-PLL + dq current control.
%
% Project definition: 10 kW grid-tied battery energy storage system (BESS),
% 400 V / 50 Hz grid, battery directly on the DC link (~700 V), grid-following.
% An EMS sends P*/Q* (P > 0: discharge into the grid); the inverter delivers
% them within its current limit. In this stage P*/Q* and Vdc come from a test
% profile; step 3 replaces Vdc with a battery model.
%
% Plant seen by the current loop (PCC voltage measured, dq frame of the PLL):
%   v_inv,d = v_pcc,d + Rf*i_d + Lf*di_d/dt - w*Lf*i_q
%   v_inv,q = v_pcc,q + Rf*i_q + Lf*di_q/dt + w*Lf*i_d
% The controller cancels v_pcc (feedforward) and the w*Lf cross terms
% (decoupling), so each axis becomes 1/(Lf*s + Rf) and gets its own PI.

% Plant, PLL, sample times and delay compensation are shared with stage 2a
run(fullfile(fileparts(mfilename('fullpath')), 'params_02a_avg_pll.m'));
Tstop = 0.40;               % [s]

%% Ratings
S_rated  = 10e3;                        % [VA]
I_rated  = S_rated/(3*Vg_ph);           % rated phase current [A rms] (14.4 A)
I_max    = 1.1*sqrt(2)*I_rated;         % current limit, peak = dq vector magnitude [A] (22.4 A)

%% Current controller (PI per axis, pole-zero cancellation)
% PI(s) = Kp + Ki/s with Kp = Lf*wc, Ki = Rf*wc cancels the plant pole at Rf/Lf,
% leaving the open loop wc/s -> first-order closed loop with bandwidth fc_i.
% The ~1.5*Tc delay limits fc_i: 300 Hz leaves about 58 deg phase margin.
fc_i = 300;                             % current loop bandwidth [Hz]
wc_i = 2*pi*fc_i;
Kp_i = Lf*wc_i;                         % [V/A]   (9.4)
Ki_i = Rf*wc_i;                         % [V/A/s] (942)

%% Anti-windup (back-calculation through the PID block's tracking input)
% Integrator input = Ki*e + Kt*(v_applied - v_unlimited). Kt sets how fast the
% integrator is pulled back while the voltage limit is active.
% The block uses Kt = aw_on*Kt_i, so run_02b can switch it off with one variable.
aw_on = 1;                              % 1: anti-windup on, 0: off (for comparison)
Kt_i  = wc_i;                           % [1/s]

%% Voltage limit
% Linear SPWM: phase peak <= Vdc/2, i.e. |v_dq| <= Vdc/2. The PCC peak is
% ~327 V, so below Vdc ~ 655 V the inverter cannot even match the grid voltage.
% v_d is mostly the grid-voltage feedforward and v_q is small (~ w*Lf*i_d). The
% limiter keeps v_q and cuts v_d: the d current loop gives way (P drops) while
% the q loop keeps Q. Cutting v_q first collapses both loops (P went to -1 kW).

%% Test profile (EMS commands and DC-link voltage)
% Breakpoints 1 us apart make steps in a linear lookup table.
e = 1e-6;
t_P = [0  0.02  0.07   0.20   0.20+e  0.25   0.25+e  1];
P_P = [0  0     10e3   10e3   15e3    15e3   10e3    10e3];   % ramp, over-request
t_Q = [0  0.10  0.10+e 1];
Q_Q = [0  0     -4e3   -4e3];                               % absorb 4 kvar (10.8 kVA < 1.1 pu)
t_V = [0  0.28  0.28+e 0.34   0.34+e  1];
V_V = [Vdc Vdc  645    645    Vdc     Vdc];                 % DC-link sag below the SPWM limit

% Grid events: +10 deg phase jump at 0.15 s, no frequency step in this stage
t_phase = 0.15;
t_freq  = inf;
