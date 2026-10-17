%% params_02a_avg_pll.m
% Stage 2a: averaged inverter + SRF-PLL. The inverter voltage is still an
% open-loop reference, but it is now generated in the dq frame and rotated
% with the PLL angle, so it follows the grid phase and frequency.
%
% Angle convention (same as step 1): phase a = V*sin(theta).
% Park transform "Q-axis" alignment then gives, when the PLL is locked,
%   v_d = grid voltage amplitude,  v_q = 0.

% Plant, filter and operating point are shared with step 1
run(fullfile(fileparts(mfilename('fullpath')), '..', '01_open_loop', 'params_01_open_loop.m'));

%% Sample times
Tc    = 1/fsw;              % controller period, one PWM carrier period [s] (200 us)
Ts    = Tc/20;              % plant step for the averaged model [s] (10 us)
Tstop = 0.35;               % [s]

%% SRF-PLL
% Linearised loop: v_q / Vg_pk ~ (theta_grid - theta_pll), so with v_q normalised
% the PLL is  PI(s) * 1/s  ->  a 2nd-order system with
%   Kp = 2*zeta*wn,  Ki = wn^2
Vg_pk    = sqrt(2)*Vg_ph;   % nominal phase peak voltage [V]
f_pll    = 20;              % PLL natural frequency [Hz]
zeta_pll = 0.707;
wn_pll   = 2*pi*f_pll;
Kp_pll   = 2*zeta_pll*wn_pll;   % [rad/s per rad]
Ki_pll   = wn_pll^2;            % [rad/s^2 per rad]
dw_max   = 2*pi*5;              % PLL frequency correction limit, +-5 Hz [rad/s]

%% Open-loop reference in the dq frame
% The PLL locks to the PCC voltage, not to the grid EMF, so the reference is
% expressed relative to the PCC phasor (Vpcc = Vg + Zg*I at the design point).
% sqrt(2)|E| sin(theta + delta_pcc) = vd*sin(theta) + vq*cos(theta)
Vpcc_ph = Vg_ph + (Rg + 1j*wg*Lg)*I_ph;          % PCC voltage phasor [V rms]
E_pcc   = E_ph * conj(Vpcc_ph)/abs(Vpcc_ph);     % inverter EMF seen from the PCC frame
vd_ref  = sqrt(2)*real(E_pcc);                   % [V]
vq_ref  = sqrt(2)*imag(E_pcc);                   % [V]

%% Delay compensation
% Sampling + one period of computation + zero-order hold on the PWM reference
% delay the applied voltage by about 1.5*Tc (5.4 deg at 50 Hz). The inverse
% Park uses theta + 1.5*Tc*w to rotate the reference that much further ahead.
k_delay = 1.5;

%% Grid disturbance scenario (programmable grid source)
t_phase  = 0.10;            % phase jump time [s]
dphi_deg = 10;              % phase jump [deg]
t_freq   = 0.20;            % frequency step time [s]
df_step  = 0.5;             % frequency step [Hz]
