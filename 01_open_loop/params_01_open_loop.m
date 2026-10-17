%% params_01_open_loop.m
% Parameters for vsi_01_open_loop.slx: DC-fed, grid-connected two-level VSI
% driven by open-loop sinusoidal PWM.
%
% The open-loop references are computed from a desired (P_ref, Q_ref)
% operating point using the steady-state phasor model of the L filter:
%
%   E = Vg + Z_tot * I,     I = conj(S/3) / conj(Vg)
%   Z_tot = (Rf + Rg) + j*wg*(Lf + Lg)   (filter + grid source impedance)
%
% With Rf ~ 0 this reduces to the classic power-transfer equations
%   P = 3*Vg*E*sin(delta)/X,   Q = 3*Vg*(E*cos(delta) - Vg)/X
% so delta (phase) mainly sets P and |E| (amplitude) mainly sets Q.
% Convention: P > 0 means power flows from the inverter into the grid.

%% Grid
Vg_ll = 400;                % line-to-line RMS voltage [V]
fg    = 50;                 % grid frequency [Hz]
wg    = 2*pi*fg;            % [rad/s]
Vg_ph = Vg_ll/sqrt(3);      % phase RMS voltage [V]
Rg    = 0.1;                % grid source internal resistance [Ohm]
Lg    = 1e-3;               % grid source internal inductance [H]

%% DC source
Vdc = 700;                  % DC-link voltage [V]

%% L filter (inverter side to PCC)
Rf = 0.5;                   % [Ohm]
Lf = 5e-3;                  % [H]
Xf = wg*Lf;                 % [Ohm]

%% Switching devices
fsw    = 5e3;               % PWM carrier frequency [Hz]
Ron_sw = 1e-3;              % IGBT / diode on-resistance [Ohm]
Vf_sw  = 0.8;               % IGBT / diode forward voltage [V]

%% Simulation
Ts    = 1e-6;               % fixed step for solver, Simscape local solver and PWM [s]
Tstop = 0.2;                % [s]
tau_pq = 5e-3;              % P/Q measurement low-pass time constant [s]

%% Open-loop operating point
P_ref = 10e3;               % [W]
Q_ref = 0;                  % [var]

I_ph  = conj((P_ref + 1j*Q_ref)/3) / Vg_ph;  % current phasor (Vg at 0 rad)
Z_tot = (Rf + Rg) + 1j*wg*(Lf + Lg);         % filter + grid source impedance
E_ph  = Vg_ph + Z_tot*I_ph;                  % required inverter EMF phasor
Vinv_pk = sqrt(2)*abs(E_ph);                 % phase peak reference [V]
delta   = angle(E_ph);                       % load angle [rad]
m_a     = Vinv_pk/(Vdc/2);                   % SPWM modulation index (linear if <= 1)

if m_a > 1
    warning('params_01_open_loop:overmodulation', ...
        'm_a = %.3f > 1: SPWM is in overmodulation, raise Vdc or lower Q_ref.', m_a);
end
