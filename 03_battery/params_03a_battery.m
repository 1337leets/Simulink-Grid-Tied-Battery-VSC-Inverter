%% params_03a_battery.m
% Step 3a: the battery replaces the ideal DC source of stage 2c.
%
% Project definition: 10 kW grid-tied battery energy storage system (BESS),
% 400 V / 50 Hz grid, battery directly on the DC link (~700 V), grid-following.
% An EMS sends P*/Q* (P > 0: discharge into the grid, P < 0: charge); the
% inverter delivers them within its current limit (1.1 pu) and the SOC limits
% of the BMS (10-90 %).

% Controller, PWM and measurements are shared with stage 2c
run(fullfile(fileparts(mfilename('fullpath')), '..', '02_current_control', 'params_02c_switching.m'));

%% Time compression
% A day cannot be simulated with a 1 us step, so the EMS day is compressed:
% one hour of the day lasts T_h seconds of simulation, and the battery capacity
% is scaled by the same factor, so the SOC moves as it would in real time.
% Valid because everything else (PLL, current loop, PWM) settles in milliseconds;
% the battery has no RC dynamics or thermal model here (their time constants
% would have to be scaled too).
T_h   = 0.05;               % simulated seconds per hour of the day [s/h]
h0    = 6;                  % the simulation starts at 06:00
Tstop = (24 - h0)*T_h;      % ... and ends at 24:00 (0.9 s)

%% Battery pack: 224 LFP cells in series, 28 Ah (20 kWh)
% LFP has a flat open-circuit voltage: 717-750 V between 10 and 90 % SOC, always
% above the ~655 V the inverter needs for the full grid voltage (stage 2b).
Ns       = 224;                                             % cells in series
AH_bat   = 28;                                              % capacity [Ah]
SOC_bp   = [0    0.05 0.10 0.20 0.30 0.50 0.70 0.80 0.90 0.95 1];
OCV_cell = [2.80 3.10 3.20 3.25 3.28 3.30 3.32 3.33 3.35 3.40 3.55];   % [V]
V0_bat   = Ns*OCV_cell;                                     % pack OCV [V]
R0_bat   = Ns*1.5e-3;                                       % series resistance [Ohm] (0.34)
E_bat    = Ns*3.2*AH_bat;                                   % nominal energy [Wh] (20.1 kWh)
AH_sim   = AH_bat*T_h/3600;                                 % compressed capacity [Ah]
SOC0     = 0.30;                                            % SOC at 06:00

%% DC link
% The film capacitor next to the bridge carries the switching ripple current,
% the cable inductance keeps it out of the battery. C_dc is precharged to the
% battery voltage (a real system has a precharge resistor and contactor).
C_dc    = 1e-3;             % [F]
L_cable = 20e-6;            % [H]
Vdc     = interp1(SOC_bp, V0_bat, SOC0);   % initial DC voltage [V] (735)

%% BMS power limits (derating near the SOC limits)
% The allowed discharge power falls linearly to zero between 15 and 10 % SOC,
% the allowed charge power between 85 and 90 %. A hard cut at the limit would
% make P* jump on and off; the ramp lets the SOC settle at the limit.
% Beyond the limits the curves change sign (forced charge below 10 %, forced
% discharge above 90 %): with P* = 0 the inverter losses and the Q exchange
% would otherwise keep draining an empty battery.
SOC_min = 0.10;  SOC_max = 0.90;  dSOC = 0.05;
P_bms   = 1.2*S_rated;      % limit away from the SOC limits [W] (above the rating:
                            % there the inverter current limit acts first)
P_hold  = 1e3;              % forced charge / discharge 2 % beyond the limits [W]
SOC_dis = [SOC_min - 0.02, SOC_min, SOC_min + dSOC];  P_dis = [-P_hold, 0, P_bms];  % P* <= P_dis(SOC)
SOC_ch  = [SOC_max - dSOC, SOC_max, SOC_max + 0.02];  P_ch  = [-P_bms, 0, P_hold];  % P* >= P_ch(SOC)

%% EMS day profile (hours of the day -> simulation time)
% Charge 4 kW around midday (e.g. solar surplus), discharge 10 kW in the
% evening peak. Both requests exceed the usable energy on purpose, so the
% SOC limits act. Q* is requested while the battery is full and while it is
% empty: reactive power needs no stored energy.
hr  = @(h) (h - h0)*T_h;
h_P = [6  9  9.25  16  16.25  18  18.25  22  22.25  24];
P_h = [0  0  -4e3  -4e3  0    0   10e3   10e3  0    0];
h_Q = [6  13  13.1  15  15.1  20.5  20.6  23  23.1  24];
Q_h = [0  0   -3e3  -3e3  0   0     4e3   4e3  0    0];
t_P = hr(h_P);  P_P = P_h;
t_Q = hr(h_Q);  Q_Q = Q_h;
t_phase = inf;              % no grid events in this step
