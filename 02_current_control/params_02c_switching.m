%% params_02c_switching.m
% Stage 2c: the stage 2b controller (unchanged) drives the switching IGBT
% bridge of step 1 instead of the averaged inverter.
%
% Project definition: 10 kW grid-tied battery energy storage system (BESS),
% 400 V / 50 Hz grid, battery directly on the DC link (~700 V), grid-following.
% An EMS sends P*/Q* (P > 0: discharge into the grid); the inverter delivers
% them within its current limit.

% Controller gains, limits and the test profile are shared with stage 2b
run(fullfile(fileparts(mfilename('fullpath')), 'params_02b_current_ctrl.m'));

%% Plant step
% The switching bridge needs a fine step: Tc/200 = 1 us gives the PWM a
% resolution of 1/100 of the half carrier period.
Ts = Tc/200;                % [s]

%% PWM (carrier comparison)
% Symmetric triangle, one period per controller period Tc. The carrier starts
% at its peak, so every controller sample (t = k*Tc) falls on a carrier peak:
% there the current ripple crosses its average value (synchronous sampling),
% and the reference updated at that instant is held for one full carrier period.
N_carrier   = round(Tc/Ts);                     % carrier samples per period (200)
n           = 0:N_carrier-1;
carrier_seq = 2*abs(1 - 2*n/N_carrier) - 1;     % +1 -> -1 -> +1
clear n

%% Start-up
% The gates are blocked until t_en, so the PLL locks and the filters settle
% first. With the bridge started at t = 0, the first two controller periods
% apply 0 V against the full grid voltage (about 20 A through Lf).
t_en = 0.01;                % PWM enable time [s]

%% Measurement: average over one carrier period (oversampling ADC)
% The PCC voltage jumps with every switching state (the L filter and the grid
% inductance form a voltage divider): a single sample on the carrier peak
% always falls in the zero vector and reads only ~5/6 of the grid voltage.
% Averaging the samples over one carrier period removes every switching
% harmonic exactly, for the voltage and the current alike.
% (A first-order RC filter instead, sampled only on the carrier peaks, leaves an
% error that differs between the two half-cycles: 3.6 % 2nd harmonic in the current.)
b_avg = ones(1, N_carrier)/N_carrier;           % FIR coefficients (moving average)
% The average is half a period old: 0.5*Tc measurement + 1*Tc computation
% + 0.5*Tc zero-order hold = 2*Tc for the delay compensation.
k_delay = 2;
