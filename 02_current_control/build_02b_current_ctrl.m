function build_02b_current_ctrl()
%BUILD_02B_CURRENT_CTRL Stage 2b: averaged inverter + SRF-PLL + dq current control.
% Power stage (left to right): averaged inverter -> L filter -> PCC sensor -> grid.
% Feedback path (right to left): sensor -> PS-S -> controller -> inverter.
% EMS commands (P*, Q*) and the DC-link voltage enter the controller from the right.

here = fileparts(mfilename('fullpath'));
m = 'vsi_02b_current_ctrl';
nl = newline;
run(fullfile(here, 'params_02b_current_ctrl.m'));

if bdIsLoaded(m), close_system(m, 0); end
f = fullfile(here, [m '.slx']);
if isfile(f), delete(f); end
new_system(m);
cellfun(@load_system, {'ee_lib', 'fl_lib', 'nesl_utility', 'ee_sl_lib'});

LIB.pgrid = findLib('ee_lib/Sources', '^Programmable.*Three');
LIB.cvs   = findLib('ee_lib/Sources', '^Controlled\sVoltage.*Three');
LIB.rlc  = 'ee_lib/Passive/RLC Assemblies/RLC (Three-Phase)';
LIB.vi   = ['ee_lib/Sensors &' nl 'Transducers/Current and Voltage' nl 'Sensor (Three-Phase)'];
LIB.ref  = 'fl_lib/Electrical/Electrical Elements/Electrical Reference';
LIB.sol  = ['nesl_utility/Solver' nl 'Configuration'];
LIB.ps2s = ['nesl_utility/PS-Simulink' nl 'Converter'];
LIB.s2ps = ['nesl_utility/Simulink-PS' nl 'Converter'];
LIB.park = ['ee_sl_lib/Mathematical' nl 'Transforms/Park Transform'];
LIB.ipark = ['ee_sl_lib/Mathematical' nl 'Transforms/Inverse' nl 'Park Transform'];
LIB.wrapint = ['ee_sl_lib/General Control/Integrator with' nl 'Wrapped State' nl '(Discrete or Continuous)'];
LIB.piaw = ['ee_sl_lib/General Control/PI Controller with' nl 'Integral Anti-Windup' nl '(Discrete or Continuous)'];
COMPOSITE = 'ee.enum.threePhasePort.composite';
blk = @(name) [m '/' name];

%% ---------------- Power stage ----------------
Y0 = 420;
inv = blk('Averaged Inverter');
add_block(LIB.cvs, inv, 'Position', [200 Y0-35 270 Y0+35], 'FRated', 'fg');
ac = portPos(inv, 'RConn', 1); yLine = ac(2);
vin = portPos(inv, 'LConn', 1); nIn = portPos(inv, 'LConn', 2);
add_block(LIB.ref, blk('Ground inv'), 'Position', [nIn(1)-45 nIn(2)+35 nIn(1)-15 nIn(2)+60]);
gi = portPos(blk('Ground inv'), 'LConn', 1);
add_line(m, [nIn; gi(1) nIn(2); gi]);
add_block(LIB.s2ps, blk('Vref S-PS'), 'Position', [vin(1)-65 vin(2)-10 vin(1)-35 vin(2)+10], ...
    'FilteringAndDerivatives', 'zero');
wire(m, blk('Vref S-PS'), 'RConn', 1, inv, 'LConn', 1);

add_block(LIB.rlc, blk('L Filter'), 'Position', [340 0 400 40], 'port_option', COMPOSITE, ...
    'component_structure', '4', 'R', 'Rf', 'L', 'Lf');
alignPort(blk('L Filter'), 'LConn', 1, [0 yLine]);
add_block(LIB.vi, blk('PCC V-I'), 'Position', [470 0 530 75], 'port_option', COMPOSITE, ...
    'vMeasurementType', '2', 'outputUnit', '2');
alignPort(blk('PCC V-I'), 'LConn', 1, [0 yLine]);
p2 = portPos(blk('PCC V-I'), 'RConn', 3);

grid = blk('Programmable Grid');
add_block(LIB.pgrid, grid, 'Position', [650 0 710 60], 'BlockMirror', 'on', ...
    'vline_rms', 'Vg_ll', 'freq', 'fg', 'shift', '0', ...
    'phase_type', 'ee.enum.timechanging.step', 'phase_step', 'dphi_deg', 'phase_t1', 't_phase', 'phase_t2', 'inf', ...
    'impedance_option', '4', 'R', 'Rg', 'L', 'Lg');
alignPort(grid, 'RConn', 1, p2);
gn = portPos(grid, 'LConn', 1);
add_block(LIB.ref, blk('Ground'), 'Position', [gn(1)+30 gn(2)+70 gn(1)+60 gn(2)+95]);
gr = portPos(blk('Ground'), 'LConn', 1);
add_block(LIB.sol, blk('Solver Configuration'), 'Position', [gr(1)+40 gn(2)+20 gr(1)+85 gn(2)+50], ...
    'UseLocalSolver', 'on', 'LocalSolverChoice', 'NE_BACKWARD_EULER_ADVANCER', ...
    'LocalSolverSampleTime', 'Ts', 'DoFixedCost', 'off', 'BlockMirror', 'on');

wire(m, inv, 'RConn', 1, blk('L Filter'), 'LConn', 1);
wire(m, blk('L Filter'), 'RConn', 1, blk('PCC V-I'), 'LConn', 1);
wire(m, blk('PCC V-I'), 'RConn', 3, grid, 'RConn', 1);
add_line(m, [gn; gr(1) gn(2); gr]);
sp = portPos(blk('Solver Configuration'), 'RConn', 1);
add_line(m, [sp; gr(1) sp(2)]);

%% ---------------- Controller (mirrored: inputs right, outputs left) ----------------
% Height 360 puts the 4 inputs 90 px apart and the 6 outputs 60 px apart, so the
% sources and scopes below can sit exactly on the port rows (straight wires).
vP = portPos(blk('PCC V-I'), 'RConn', 1); iP = portPos(blk('PCC V-I'), 'RConn', 2);
xr = vP(1); yTop = yLine + 85;
ctrl = blk('Controller');
add_block('simulink/Ports & Subsystems/Subsystem', ctrl, 'Position', [xr-280 yTop xr-110 yTop+360]);
buildController(ctrl, LIB);
set_param(ctrl, 'BlockMirror', 'on');
yIn = arrayfun(@(k) getfield(portPos(ctrl, 'Inport', k), {2}), 1:4); %#ok<GFLD>

% measurements: PS-S converters (mirrored, output on the left) on input rows 1 and 2
add_block(LIB.ps2s, blk('I PS-S'), 'Position', [xr-30 yIn(1)-10 xr yIn(1)+10], 'VectorFormat', '1-D array', 'BlockMirror', 'on');
add_block(LIB.ps2s, blk('V PS-S'), 'Position', [xr-30 yIn(2)-10 xr yIn(2)+10], 'VectorFormat', '1-D array', 'BlockMirror', 'on');
add_line(m, [iP; iP(1)+12 iP(2); iP(1)+12 yIn(1); xr yIn(1)]);
add_line(m, [vP; vP(1)+25 vP(2); vP(1)+25 yIn(2); xr yIn(2)]);
li = wireSig(m, blk('I PS-S'), 1, ctrl, 1); set_param(li, 'Name', 'Iabc_pcc');
lv = wireSig(m, blk('V PS-S'), 1, ctrl, 2); set_param(lv, 'Name', 'Vabc_pcc');

% EMS commands and DC-link voltage (test profiles) on input rows 3 and 4
ems = blk('EMS commands');
add_block('simulink/Ports & Subsystems/Subsystem', ems, 'Position', [xr-60 yIn(3)-22 xr yIn(3)+22]);
buildProfile(ems, {'P* profile', 't_P', 'P_P'; 'Q* profile', 't_Q', 'Q_Q'}, 'PQ_ref');
set_param(ems, 'BlockMirror', 'on', 'MaskDisplay', sprintf('disp(''EMS\\nP*, Q*'');'));
vdc = blk('DC link');
add_block('simulink/Ports & Subsystems/Subsystem', vdc, 'Position', [xr-60 yIn(4)-22 xr yIn(4)+22]);
buildProfile(vdc, {'Vdc profile', 't_V', 'V_V'}, 'Vdc');
set_param(vdc, 'BlockMirror', 'on', 'MaskDisplay', sprintf('disp(''Vdc\\n(step 3:\\nbattery)'');'));
le = wireSig(m, ems, 1, ctrl, 3); set_param(le, 'Name', 'PQ_ref');
ld = wireSig(m, vdc, 1, ctrl, 4); set_param(ld, 'Name', 'Vdc');

% controller output 1 -> inverter (lane on the far left)
o = arrayfun(@(k) portPos(ctrl, 'Outport', k), 1:6, 'UniformOutput', false);
sIn = portPos(blk('Vref S-PS'), 'Inport', 1);
xl = sIn(1) - 40;
lr = add_line(m, [o{1}; xl o{1}(2); xl sIn(2); sIn]); set_param(lr, 'Name', 'vabc_ref');

% scopes on the output rows (mirrored)
d = o{3}(2) - o{2}(2);
xs = xl + 45;
% (the two scopes touch, so the upper one shows its name on top)
add_block('simulink/Sinks/Scope', blk('PLL'), 'NumInputPorts', '2', 'BlockMirror', 'on', ...
    'Position', [xs o{2}(2)-d/2 xs+35 o{3}(2)+d/2], 'NamePlacement', 'alternate');
add_block('simulink/Sinks/Scope', blk('Current loop'), 'NumInputPorts', '3', 'BlockMirror', 'on', ...
    'Position', [xs o{4}(2)-d/2 xs+35 o{6}(2)+d/2]);
set_param(ctrl, 'ShowName', 'off');
names = {'', 'f_pll', 'vdq', 'idq', 'PQ', 'vdq_inv'};
dst = {'', 'PLL', 'PLL', 'Current loop', 'Current loop', 'Current loop'};
dpt = [0 1 2 1 2 3];
ls = zeros(1, 6);
for k = 2:6
    ls(k) = wireSig(m, ctrl, k, blk(dst{k}), dpt(k)); set_param(ls(k), 'Name', names{k});
end

for s = [{'Vabc_pcc', lv; 'Iabc_pcc', li; 'vabc_ref', lr; 'PQ_ref', le; 'Vdc', ld}; names(2:6)', num2cell(ls(2:6))']'
    set_param(get_param(s{2}, 'SrcPortHandle'), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', s{1});
end

%% ---------------- Model settings ----------------
set_param(m, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', 'FixedStep', 'Ts', ...
    'SolverMode', 'SingleTasking', 'StopTime', 'Tstop', ...
    'SignalLogging', 'on', 'SignalLoggingName', 'logsout', 'ReturnWorkspaceOutputs', 'on', ...
    'SaveTime', 'off', 'SaveOutput', 'off', 'SimscapeLogType', 'none');
cb = sprintf('run(fullfile(fileparts(get_param(''%s'',''FileName'')), ''params_02b_current_ctrl.m''));', m);
set_param(m, 'PostLoadFcn', cb, 'InitFcn', cb);

note(m, [40 Y0-200], sprintf(['Stage 2b - Averaged inverter + SRF-PLL + dq current control\n' ...
    '10 kW grid-tied BESS: the EMS sends P*/Q*, the inverter injects the matching current (limited to 1.1 pu).\n' ...
    'The averaged inverter has no DC side; Vdc only sets the controller''s voltage limit |v| <= Vdc/2.\n' ...
    'Test: P* ramp 0.02-0.07 s, Q* = -4 kvar at 0.10 s, +10 deg grid phase jump at 0.15 s,\n' ...
    'P* = 15 kW (over the limit) 0.20-0.25 s, Vdc sag to 645 V 0.28-0.34 s.']));
note(m, [gr(1)+100 gn(2)+25], sprintf('Every Simscape network needs\none Solver Configuration (f(x)=0)'));
note(m, [xr+40 yIn(3)-10], sprintf('Test profiles (lookup tables).\nStep 3 replaces Vdc with a battery.'));

set_param(m, 'ZoomFactor', 'FitSystem');
save_system(m, f);
fprintf('Saved %s\n', f);
end

%% ======================= controller =======================
function buildController(c, LIB)
% Controller top level, everything at Tc. The PLL output is a bus (theta, f, vdq):
% one line carries the grid angle, frequency and voltage to every block that needs them.
%   Vabc -> ADC -> SRF-PLL ------------------------------------- pll
%   Iabc -> ADC -> Current dq (Park with theta) ----------------- i_dq
%   pll, i_dq, P*/Q*, Vdc -> Current controller ----------------- v_dq
%   pll, v_dq -> Modulator (inverse Park, delay compensation, DAC) -> vabc_ref
delete_line(c, 'In1/1', 'Out1/1');
set_param([c '/In1'], 'Name', 'Iabc', 'Position', [20 163 50 177]);
set_param([c '/Out1'], 'Name', 'vabc_ref', 'Position', [1110 233 1140 247]);
add_block('simulink/Sources/In1', [c '/Vabc'], 'Position', [20 53 50 67]);
add_block('simulink/Sources/In1', [c '/PQ_ref'], 'Position', [20 358 50 372]);
add_block('simulink/Sources/In1', [c '/Vdc'], 'Position', [20 408 50 422]);
rt = {'OutPortSampleTime', 'Tc'};
add_block('simulink/Signal Attributes/Rate Transition', [c '/ADC V'], 'Position', [90 45 130 75], rt{:});
add_block('simulink/Signal Attributes/Rate Transition', [c '/ADC I'], 'Position', [90 155 130 185], rt{:});

sub = @(name, pos) add_block('simulink/Ports & Subsystems/Subsystem', [c '/' name], 'Position', pos);
sub('SRF-PLL', [170 30 270 90]);              buildPLL([c '/SRF-PLL'], LIB);
sub('Current dq', [340 125 440 185]);         buildCurrentDq([c '/Current dq'], LIB);
sub('Current controller', [580 240 720 440]); buildCurrentCtrl([c '/Current controller']);
sub('Modulator', [800 190 900 290]);          buildModulator([c '/Modulator'], LIB);
add_block('simulink/Signal Routing/Bus Selector', [c '/f, vdq'], 'Position', [950 40 955 80], ...
    'OutputSignals', 'f,vdq');
outs = {'f_pll', 50; 'vdq', 70; 'idq', 315; 'PQ', 365; 'vdq_inv', 415};
for k = 1:size(outs, 1)
    add_block('simulink/Sinks/Out1', [c '/' outs{k, 1}], 'Position', [1110 outs{k, 2}-7 1140 outs{k, 2}+7]);
end

add_line(c, 'Vabc/1', 'ADC V/1'); add_line(c, 'ADC V/1', 'SRF-PLL/1');
add_line(c, 'Iabc/1', 'ADC I/1'); add_line(c, 'ADC I/1', 'Current dq/2');
% pll bus along the top, branches drop down to their consumers
lp = add_line(c, 'SRF-PLL/1', 'f, vdq/1'); set_param(lp, 'Name', 'pll');
add_line(c, [300 60; 300 140; 340 140]);
add_line(c, [550 60; 550 265; 580 265]);
add_line(c, [780 60; 780 215; 800 215]);
li = add_line(c, [440 155; 520 155; 520 315; 580 315]); set_param(li, 'Name', 'i_dq');
add_line(c, 'PQ_ref/1', 'Current controller/3');
add_line(c, 'Vdc/1', 'Current controller/4');
lv = add_line(c, 'Current controller/1', 'Modulator/2'); set_param(lv, 'Name', 'v_dq');
add_line(c, 'Modulator/1', 'vabc_ref/1');
add_line(c, 'f, vdq/1', 'f_pll/1'); add_line(c, 'f, vdq/2', 'vdq/1');
add_line(c, 'Current controller/2', 'idq/1');
add_line(c, 'Current controller/3', 'PQ/1');
add_line(c, 'Current controller/4', 'vdq_inv/1');
set_param(c, 'ContentPreviewEnabled', 'off', 'MaskDisplay', sprintf('disp(''Controller\\nPLL + dq current\\nTc = 200 us'');'));
end

function buildPLL(p, LIB)
% v_abc -> Park(theta) -> v_q / Vg_pk -> PI -> + w_g -> integrate (wrapped 0..2pi) -> theta
% Output: bus pll = {theta, f, vdq}
delete_line(p, 'In1/1', 'Out1/1');
set_param([p '/In1'], 'Name', 'Vabc', 'Position', [20 83 50 97]);
set_param([p '/Out1'], 'Name', 'pll', 'Position', [820 143 850 157]);
add_block(LIB.park, [p '/Park'], 'Position', [90 70 160 120]);
add_block('simulink/Signal Routing/Demux', [p '/Demux'], 'Outputs', '3', 'Position', [190 65 195 125]);
add_block('simulink/Math Operations/Gain', [p '/normalise'], 'Position', [240 80 300 100], 'Gain', '1/Vg_pk');
add_block(LIB.piaw, [p '/PI'], 'Position', [340 70 420 110], ...
    'Kp', 'Kp_pll', 'Ki', 'Ki_pll', 'B', 'dw_max', 'A', '-dw_max', 'Ts', 'Tc');
add_block('simulink/Sources/Constant', [p '/no reset'], 'Position', [290 120 310 140], 'Value', '0', 'SampleTime', 'Tc');
add_block('simulink/Sources/Constant', [p '/w_g nominal'], 'Position', [420 135 480 155], 'Value', 'wg', 'SampleTime', 'Tc');
add_block('simulink/Math Operations/Sum', [p '/Sum'], 'Inputs', '++', 'Position', [520 80 540 100]);
add_block(LIB.wrapint, [p '/Integrator (wrap 0..2pi)'], 'Position', [600 70 680 110], 'Ts', 'Tc');
add_block('simulink/Math Operations/Gain', [p '/to Hz'], 'Position', [600 140 660 160], 'Gain', '1/(2*pi)');
add_block('simulink/Signal Routing/Mux', [p '/Mux vdq'], 'Inputs', '2', 'Position', [240 190 245 230]);
add_block('simulink/Sinks/Terminator', [p '/zero seq'], 'Position', [240 250 255 265]);
add_block('simulink/Signal Routing/Bus Creator', [p '/Bus'], 'Inputs', '3', 'Position', [760 60 765 240]);

add_line(p, 'Vabc/1', 'Park/1');
add_line(p, 'Park/1', 'Demux/1');
add_line(p, 'Demux/2', 'normalise/1');
add_line(p, 'normalise/1', 'PI/1');
add_line(p, 'no reset/1', 'PI/2');
add_line(p, 'PI/1', 'Sum/1');
add_line(p, 'w_g nominal/1', 'Sum/2', 'autorouting', 'smart');
add_line(p, 'Sum/1', 'Integrator (wrap 0..2pi)/1');
add_line(p, 'Sum/1', 'to Hz/1', 'autorouting', 'smart');
lt = add_line(p, 'Integrator (wrap 0..2pi)/1', 'Bus/1');
% theta feedback to the Park transform, over the top (branch near the bus keeps the label clear)
it = portPos([p '/Integrator (wrap 0..2pi)'], 'Outport', 1); pk = portPos([p '/Park'], 'Inport', 2);
add_line(p, [740 it(2); 740 20; 70 20; 70 pk(2); pk]);
lf = add_line(p, 'to Hz/1', 'Bus/2');
add_line(p, 'Demux/1', 'Mux vdq/1', 'autorouting', 'smart');
add_line(p, 'Demux/2', 'Mux vdq/2', 'autorouting', 'smart');
add_line(p, 'Demux/3', 'zero seq/1', 'autorouting', 'smart');
lq = add_line(p, 'Mux vdq/1', 'Bus/3');
add_line(p, 'Bus/1', 'pll/1');
set_param(lt, 'Name', 'theta'); set_param(lf, 'Name', 'f'); set_param(lq, 'Name', 'vdq');
set_param(p, 'ContentPreviewEnabled', 'off', 'MaskDisplay', 'disp(''SRF-PLL'');');
end

function buildCurrentDq(s, LIB)
% i_abc -> Park(theta) -> [i_d; i_q]
delete_line(s, 'In1/1', 'Out1/1');
set_param([s '/In1'], 'Name', 'pll', 'Position', [20 33 50 47]);
set_param([s '/Out1'], 'Name', 'i_dq', 'Position', [340 78 370 92]);
add_block('simulink/Sources/In1', [s '/Iabc'], 'Position', [20 73 50 87]);
add_block('simulink/Signal Routing/Bus Selector', [s '/theta'], 'Position', [90 30 95 50], 'OutputSignals', 'theta');
add_block(LIB.park, [s '/Park'], 'Position', [150 60 220 110]);
add_block('simulink/Signal Routing/Selector', [s '/d,q'], 'Position', [260 70 290 100], ...
    'IndexParamArray', {'[1 2]'}, 'InputPortWidth', '3');
add_line(s, 'pll/1', 'theta/1');
add_line(s, 'Iabc/1', 'Park/1', 'autorouting', 'smart');
add_line(s, 'theta/1', 'Park/2', 'autorouting', 'smart');
add_line(s, 'Park/1', 'd,q/1');
add_line(s, 'd,q/1', 'i_dq/1', 'autorouting', 'smart');
set_param(s, 'ContentPreviewEnabled', 'off', 'MaskDisplay', ...
    sprintf('port_label(''input'',1,''pll'');port_label(''input'',2,''Iabc'');disp(''abc -> dq'');'));
end

function buildCurrentCtrl(s)
% i_dq* = f(P*, Q*, v_d), limited to I_max                   (reference)
% v_unl = PI(i_dq* - i_dq) + v_pcc,dq + w*Lf*[-i_q; i_d]      (PI + feedforward + decoupling)
% v_dq  = limit(v_unl, Vdc/2)                                 (voltage limit)
% PI tracking input = v_dq - feedforward                      (anti-windup by back-calculation)
% Layout: vdq rail along the top, i_dq rail in the middle, main path
% (reference -> error -> PI -> + feedforward -> limit) below, anti-windup
% feedback underneath. Some inports sit next to their consumer to keep wires short.
b = @(n) [s '/' n];
rect = {'IconShape', 'rectangular'};
delete_line(s, 'In1/1', 'Out1/1');
set_param(b('In1'), 'Name', 'pll', 'Position', [20 33 50 47]);
set_param(b('Out1'), 'Name', 'v_dq', 'Position', [1000 303 1030 317]);
add_block('simulink/Sources/In1', b('i_dq'), 'Position', [120 203 150 217]);
add_block('simulink/Sources/In1', b('PQ_ref'), 'Position', [115 318 145 332]);
add_block('simulink/Sources/In1', b('Vdc'), 'Position', [630 343 660 357]);
add_block('simulink/Signal Routing/Bus Selector', b('vdq'), 'Position', [70 30 75 50], 'OutputSignals', 'vdq');

% current reference with limiter
add_block('simulink/User-Defined Functions/MATLAB Function', b('Current reference'), 'Position', [160 315 290 355]);
setScript(b('Current reference'), strjoin({
    'function idq_ref = fcn(PQ_ref, vdq, I_max, Vg_pk)'
    '% i_d* = 2P*/(3 v_d),  i_q* = -2Q*/(3 v_d)   (v_q = 0 when the PLL is locked)'
    'vd = max(vdq(1), 0.5*Vg_pk);              % guard against a collapsed grid voltage'
    'id = 2*PQ_ref(1)/(3*vd);'
    'iq = -2*PQ_ref(2)/(3*vd);'
    '% |i_dq| <= I_max with d-axis (active power) priority'
    'id = min(max(id, -I_max), I_max);'
    'iq_max = sqrt(I_max^2 - id^2);'
    'iq = min(max(iq, -iq_max), iq_max);'
    'idq_ref = [id; iq];'}, newline), {'I_max', 'Vg_pk'});

% error = i_dq* - i_dq (measurement on the upper port)
add_block('simulink/Math Operations/Sum', b('error'), 'Inputs', '-+', rect{:}, 'Position', [340 305 360 345]);
% PI: one vectorised block for both axes, back-calculation through the tracking input
add_block('simulink/Discrete/Discrete PID Controller', b('PI dq'), 'Position', [400 310 480 370], ...
    'Controller', 'PI', 'P', 'Kp_i', 'I', 'Ki_i', 'SampleTime', 'Tc', 'IntegratorMethod', 'Forward Euler', ...
    'TrackingMode', 'on', 'Kt', 'aw_on*Kt_i');

% feedforward: PCC voltage + decoupling w*Lf*[-i_q; i_d]
add_block('simulink/Math Operations/Gain', b('decoupling'), 'Position', [400 195 480 225], ...
    'Gain', 'wg*Lf*[0 -1; 1 0]', 'Multiplication', 'Matrix(K*u)');
add_block('simulink/Math Operations/Sum', b('feedforward'), 'Inputs', '++', rect{:}, 'Position', [520 180 540 220]);
add_block('simulink/Math Operations/Sum', b('v unlimited'), 'Inputs', '++', rect{:}, 'Position', [600 310 620 350]);

% voltage limit (v_q kept, v_d cut) + anti-windup signal
add_block('simulink/User-Defined Functions/MATLAB Function', b('Voltage limit'), 'Position', [680 300 800 360]);
setScript(b('Voltage limit'), strjoin({
    'function [v, active, tr] = fcn(v_ff, v_unl, Vdc)'
    '% Linear SPWM: phase peak <= Vdc/2, i.e. |v_dq| <= Vdc/2.'
    '% v_d is mostly the grid-voltage feedforward, v_q is small (~ w*Lf*i_d).'
    '% v_q is kept first and v_d is cut: the d current loop gives way (P drops)'
    '% while the q loop keeps control. Cutting v_q first instead leaves both'
    '% coupled loops without margin and P collapses (try it).'
    'vmax = Vdc/2;'
    'vq = min(max(v_unl(2), -vmax), vmax);'
    'vd_max = sqrt(vmax^2 - vq^2);'
    'vd = min(max(v_unl(1), -vd_max), vd_max);'
    'v = [vd; vq];'
    'active = double(vd ~= v_unl(1) || vq ~= v_unl(2));'
    '% Anti-windup: the PI part that was actually applied. The PI block pulls its'
    '% integrator toward it (tracking input) whenever the limit is active.'
    'tr = v - v_ff;'}, newline), {});
add_block('simulink/Sinks/Terminator', b('limit active'), 'Position', [850 323 865 337]);

% monitoring outputs
add_block('simulink/User-Defined Functions/MATLAB Function', b('P, Q'), 'Position', [820 29 920 73]);
setScript(b('P, Q'), strjoin({
    'function PQ = fcn(vdq, idq)'
    '% amplitude-invariant dq: P = 1.5(v_d i_d + v_q i_q), Q = 1.5(v_q i_d - v_d i_q)'
    'PQ = 1.5*[vdq(1)*idq(1) + vdq(2)*idq(2); vdq(2)*idq(1) - vdq(1)*idq(2)];'}, newline), {});
add_block('simulink/Signal Routing/Mux', b('meas & ref'), 'Inputs', '2', 'Position', [880 100 885 140]);
add_block('simulink/Sinks/Out1', b('i_mon'), 'Position', [1000 113 1030 127]);
add_block('simulink/Sinks/Out1', b('PQ'), 'Position', [1000 44 1030 58]);
add_block('simulink/Sinks/Out1', b('vdq_inv'), 'Position', [1000 433 1030 447]);

% snap blocks so that the straight connections are exactly horizontal
snap = {'vdq', 'Inport', 1, 'pll', 'Outport', 1
        'P, Q', 'Inport', 1, 'vdq', 'Outport', 1
        'PQ', 'Inport', 1, 'P, Q', 'Outport', 1
        'decoupling', 'Inport', 1, 'i_dq', 'Outport', 1
        'feedforward', 'Inport', 2, 'decoupling', 'Outport', 1
        'PQ_ref', 'Outport', 1, 'Current reference', 'Inport', 1
        'error', 'Inport', 2, 'Current reference', 'Outport', 1
        'PI dq', 'Inport', 1, 'error', 'Outport', 1
        'v unlimited', 'Inport', 2, 'PI dq', 'Outport', 1
        'Voltage limit', 'Inport', 2, 'v unlimited', 'Outport', 1
        'Vdc', 'Outport', 1, 'Voltage limit', 'Inport', 3
        'v_dq', 'Inport', 1, 'Voltage limit', 'Outport', 1
        'limit active', 'Inport', 1, 'Voltage limit', 'Outport', 2
        'i_mon', 'Inport', 1, 'meas & ref', 'Outport', 1};
for k = 1:size(snap, 1)
    alignPort(b(snap{k, 1}), snap{k, 2}, snap{k, 3}, portPos(b(snap{k, 4}), snap{k, 5}, snap{k, 6}));
end

% straight connections
lr = add_line(s, 'Current reference/1', 'error/2');
for c = {'pll/1', 'vdq/1'; 'vdq/1', 'P, Q/1'; 'P, Q/1', 'PQ/1'; 'i_dq/1', 'decoupling/1'
         'decoupling/1', 'feedforward/2'; 'PQ_ref/1', 'Current reference/1'; 'error/1', 'PI dq/1'
         'PI dq/1', 'v unlimited/2'; 'v unlimited/1', 'Voltage limit/2'; 'Vdc/1', 'Voltage limit/3'
         'Voltage limit/1', 'v_dq/1'; 'meas & ref/1', 'i_mon/1'}'
    add_line(s, c{1}, c{2});
end
la = add_line(s, 'Voltage limit/2', 'limit active/1');

% branches: start on an existing line, run vertically at x, end at the port
yOf = @(blk, side, k) getfield(portPos(b(blk), side, k), {2}); %#ok<GFLD>
yV = yOf('vdq', 'Outport', 1); yI = yOf('i_dq', 'Outport', 1);
branch(s, [100 yV], b('Current reference'), 2);
branch(s, [500 yV], b('feedforward'), 1);
branch(s, [320 yI], b('error'), 1);
branch(s, [260 yI], b('P, Q'), 2);
branch(s, [800 yOf('P, Q', 'Inport', 2)], b('meas & ref'), 1);
branch(s, [300 yOf('Current reference', 'Outport', 1)], b('meas & ref'), 2);
ff = portPos(b('feedforward'), 'Outport', 1);
branch(s, ff, b('v unlimited'), 1, 580);
branch(s, [580 ff(2)], b('Voltage limit'), 1, 650);
branch(s, [900 yOf('Voltage limit', 'Outport', 1)], b('vdq_inv'), 1);
% anti-windup feedback underneath the main path
tr = portPos(b('Voltage limit'), 'Outport', 3); ti = portPos(b('PI dq'), 'Inport', 2);
yFb = tr(2) + 50;
add_line(s, [tr; 820 tr(2); 820 yFb; 380 yFb; 380 ti(2); ti]);

% internal signals worth logging
logAs(get_param(lr, 'SrcPortHandle'), 'idq_ref');
ph = get_param([s '/i_dq'], 'PortHandles'); logAs(ph.Outport(1), 'idq_meas');
logAs(get_param(la, 'SrcPortHandle'), 'v_limit_active');
set_param(s, 'ContentPreviewEnabled', 'off', 'MaskDisplay', sprintf(['port_label(''input'',1,''pll'');' ...
    'port_label(''input'',2,''i_dq'');port_label(''input'',3,''P*,Q*'');port_label(''input'',4,''Vdc'');' ...
    'port_label(''output'',1,''v_dq'');port_label(''output'',2,''i ref/meas'');port_label(''output'',3,''P,Q'');' ...
    'port_label(''output'',4,''v_dq'');disp(''dq current\\ncontroller'');']));
end

function buildModulator(s, LIB)
% [v_d; v_q; 0] -> inverse Park(theta + k_delay*Tc*w) -> compute delay -> DAC (ZOH at Ts)
delete_line(s, 'In1/1', 'Out1/1');
set_param([s '/In1'], 'Name', 'pll', 'Position', [20 33 50 47]);
set_param([s '/Out1'], 'Name', 'vabc_ref', 'Position', [600 113 630 127]);
set_param([s '/pll'], 'Position', [20 178 50 192]);
add_block('simulink/Sources/In1', [s '/v_dq'], 'Position', [20 88 50 102]);
add_block('simulink/Signal Routing/Mux', [s '/v_dq0'], 'Inputs', '2', 'Position', [250 85 255 125]);
add_block('simulink/Sources/Constant', [s '/zero seq'], 'Position', [200 105 220 125], 'Value', '0');
add_block('simulink/Signal Routing/Bus Selector', [s '/theta, f'], 'Position', [70 165 75 205], 'OutputSignals', 'theta,f');
add_block('simulink/Math Operations/Gain', [s '/k_delay*Tc*w'], 'Position', [130 185 200 205], 'Gain', 'k_delay*Tc*2*pi');
add_block('simulink/Math Operations/Sum', [s '/theta comp'], 'Inputs', '++', 'IconShape', 'rectangular', ...
    'Position', [240 165 260 205]);
add_block(LIB.ipark, [s '/Inverse Park'], 'Position', [310 90 380 150]);
add_block('simulink/Discrete/Unit Delay', [s '/Compute delay'], 'Position', [420 105 460 135], 'SampleTime', 'Tc');
% non-deterministic slow-to-fast transfer = plain zero-order hold (the deterministic
% default would add another full Tc of delay)
add_block('simulink/Signal Attributes/Rate Transition', [s '/DAC'], 'Position', [500 105 540 135], ...
    'OutPortSampleTime', 'Ts', 'Deterministic', 'off');
add_line(s, 'pll/1', 'theta, f/1');
add_line(s, 'theta, f/1', 'theta comp/1');
add_line(s, 'theta, f/2', 'k_delay*Tc*w/1');
add_line(s, 'k_delay*Tc*w/1', 'theta comp/2');
add_line(s, 'v_dq/1', 'v_dq0/1');
add_line(s, 'zero seq/1', 'v_dq0/2');
add_line(s, 'v_dq0/1', 'Inverse Park/1');
add_line(s, [260 185; 285 185; 285 135; 310 135]);
add_line(s, 'Inverse Park/1', 'Compute delay/1');
add_line(s, 'Compute delay/1', 'DAC/1');
add_line(s, 'DAC/1', 'vabc_ref/1');
set_param(s, 'ContentPreviewEnabled', 'off', 'MaskDisplay', ...
    sprintf('port_label(''input'',1,''pll'');port_label(''input'',2,''v_dq'');disp(''dq -> abc\\n+ delay comp.'');'));
end

function branch(s, start, dst, k, xv)
% Branch line: from a point on an existing line, horizontally to x = xv,
% vertically to the inport's row, then into inport k of dst.
if nargin < 5, xv = start(1); end
p = portPos(dst, 'Inport', k);
pts = unique([start; xv start(2); xv p(2); p], 'rows', 'stable');
add_line(s, pts);
end

function logAs(ph, name)
set_param(ph, 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', name);
end

function buildProfile(s, rows, outName)
% Digital Clock (Tc) -> one 1-D lookup table per row {name, breakpoints, values} -> Mux -> Out
delete_line(s, 'In1/1', 'Out1/1');
delete_block([s '/In1']);
n = size(rows, 1);
add_block('simulink/Sources/Digital Clock', [s '/t'], 'SampleTime', 'Tc', 'Position', [20 20 + 30*n 50 40 + 30*n]);
for k = 1:n
    y = 20 + 60*(k-1);
    add_block('simulink/Lookup Tables/1-D Lookup Table', [s '/' rows{k, 1}], 'Position', [110 y 170 y+40], ...
        'BreakpointsForDimension1', rows{k, 2}, 'Table', rows{k, 3}, 'ExtrapMethod', 'Clip');
    add_line(s, 't/1', [rows{k, 1} '/1'], 'autorouting', 'smart');
end
set_param([s '/Out1'], 'Name', outName, 'Position', [300 18 + 30*n 330 32 + 30*n]);
if n > 1
    add_block('simulink/Signal Routing/Mux', [s '/Mux'], 'Inputs', num2str(n), 'Position', [230 15 235 45 + 60*(n-1)]);
    for k = 1:n, add_line(s, [rows{k, 1} '/1'], sprintf('Mux/%d', k)); end
    add_line(s, 'Mux/1', [outName '/1']);
else
    add_line(s, [rows{1, 1} '/1'], [outName '/1']);
end
set_param(s, 'ContentPreviewEnabled', 'off');
end

function setScript(b, code, params)
ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', b);
ch.Script = code;
for k = 1:numel(params)
    dobj = ch.find('-isa', 'Stateflow.Data', 'Name', params{k});
    dobj.Scope = 'Parameter';
end
end

%% ======================= helpers =======================
function p = findLib(parent, rx)
b = find_system(parent, 'SearchDepth', 1, 'Regexp', 'on', 'Name', rx);
p = b{1};
end

function p = portPos(b, side, k)
pc = get_param(b, 'PortHandles');
p = get_param(pc.(side)(k), 'Position');
end

function alignPort(b, side, k, target)
p = portPos(b, side, k); pos = get_param(b, 'Position');
dy = target(2) - p(2);
set_param(b, 'Position', pos + [0 dy 0 dy]);
end

function wire(m, b1, s1, k1, b2, s2, k2)
a = portPos(b1, s1, k1); b = portPos(b2, s2, k2);
if a(2) == b(2)
    pts = [a; b];
else
    xm = round((a(1) + b(1))/2); pts = [a; xm a(2); xm b(2); b];
end
add_line(m, pts);
end

function l = wireSig(m, b1, k1, b2, k2)
% signal line from outport k1 of b1 to inport k2 of b2 with one elbow
a = portPos(b1, 'Outport', k1); b = portPos(b2, 'Inport', k2);
if a(2) == b(2)
    pts = [a; b];
else
    xm = round((a(1) + b(1))/2); pts = [a; xm a(2); xm b(2); b];
end
l = add_line(m, pts);
end

function note(m, xy, txt)
a = Simulink.Annotation(m, txt);
a.Position = [xy xy + [10 10]];
a.HorizontalAlignment = 'left';
a.FontSize = 10;
end
