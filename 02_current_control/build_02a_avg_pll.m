function build_02a_avg_pll()
%BUILD_02A_AVG_PLL Stage 2a: averaged inverter + SRF-PLL on a programmable grid.
% Power stage (left to right): averaged inverter -> L filter -> PCC sensor -> grid.
% Feedback path (bottom row, right to left): sensor -> PS-S -> controller -> inverter.

here = fileparts(mfilename('fullpath'));
m = 'vsi_02a_avg_pll';
nl = newline;
run(fullfile(here, 'params_02a_avg_pll.m'));

if bdIsLoaded(m), close_system(m, 0); end
f = fullfile(here, [m '.slx']);
if isfile(f), delete(f); end
new_system(m);
cellfun(@load_system, {'ee_lib', 'fl_lib', 'nesl_utility', 'ee_sl_lib'});

LIB.cvs  = ['ee_lib/Sources/Controlled Voltage' nl 'Source' nl '(Three-Phase)'];
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
    'frequency_type', 'ee.enum.timechanging.step', 'freq_step', 'df_step', 'freq_t1', 't_freq', 'freq_t2', 'inf', ...
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

%% ---------------- Measurements (dropping below the power wire) ----------------
vP = portPos(blk('PCC V-I'), 'RConn', 1); iP = portPos(blk('PCC V-I'), 'RConn', 2);
xr = vP(1); yI = yLine + 110; yV = yLine + 160;
% PS-S converters mirrored: input on the right, output on the left (feedback flows right to left)
add_block(LIB.ps2s, blk('I PS-S'), 'Position', [xr-30 yI-10 xr yI+10], 'VectorFormat', '1-D array', 'BlockMirror', 'on');
add_block(LIB.ps2s, blk('V PS-S'), 'Position', [xr-30 yV-10 xr yV+10], 'VectorFormat', '1-D array', 'BlockMirror', 'on');
add_line(m, [iP; iP(1)+12 iP(2); iP(1)+12 yI; xr yI]);
add_line(m, [vP; vP(1)+25 vP(2); vP(1)+25 yV; xr yV]);

%% ---------------- Controller (mirrored: inputs right, outputs left) ----------------
ctrl = blk('Controller');
add_block('simulink/Ports & Subsystems/Subsystem', ctrl, 'Position', [xr-270 yI-40 xr-100 yV+70]);
buildController(ctrl, LIB);
set_param(ctrl, 'BlockMirror', 'on');
li = wireSig(m, blk('I PS-S'), 1, ctrl, 1); set_param(li, 'Name', 'Iabc_pcc');
lv = wireSig(m, blk('V PS-S'), 1, ctrl, 2); set_param(lv, 'Name', 'Vabc_pcc');

% controller output -> inverter (lane on the far left)
o1 = portPos(ctrl, 'Outport', 1); sIn = portPos(blk('Vref S-PS'), 'Inport', 1);
xl = sIn(1) - 40;
lr = add_line(m, [o1; xl o1(2); xl sIn(2); sIn]); set_param(lr, 'Name', 'vabc_ref');

% PLL scope (mirrored, left of the controller)
o2 = portPos(ctrl, 'Outport', 2);
add_block('simulink/Sinks/Scope', blk('PLL'), 'NumInputPorts', '3', 'BlockMirror', 'on', ...
    'Position', [xl+30 o2(2)-15 xl+60 o2(2)+75]);
lt = wireSig(m, ctrl, 2, blk('PLL'), 1); set_param(lt, 'Name', 'theta_pll');
lf = wireSig(m, ctrl, 3, blk('PLL'), 2); set_param(lf, 'Name', 'f_pll');
lq = wireSig(m, ctrl, 4, blk('PLL'), 3); set_param(lq, 'Name', 'vdq');

%% ---------------- P/Q (third row) ----------------
pq = blk('PQ Calculation');
add_block('simulink/Ports & Subsystems/Subsystem', pq, 'Position', [xr+40 yV+50 xr+130 yV+130]);
buildPQ(pq);
add_line(m, 'V PS-S/1', 'PQ Calculation/1', 'autorouting', 'smart');
add_line(m, 'I PS-S/1', 'PQ Calculation/2', 'autorouting', 'smart');
q = get_param(pq, 'Position');
add_block('simulink/Sinks/Scope', blk('P & Q'), 'NumInputPorts', '2', 'Position', [q(3)+60 q(2) q(3)+90 q(4)]);
lp = add_line(m, 'PQ Calculation/1', 'P & Q/1'); set_param(lp, 'Name', 'P');
lqq = add_line(m, 'PQ Calculation/2', 'P & Q/2'); set_param(lqq, 'Name', 'Q');

for s = {'Vabc_pcc', lv; 'Iabc_pcc', li; 'vabc_ref', lr; 'theta_pll', lt; 'f_pll', lf; 'vdq', lq; 'P', lp; 'Q', lqq}'
    set_param(get_param(s{2}, 'SrcPortHandle'), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', s{1});
end

%% ---------------- Model settings ----------------
set_param(m, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', 'FixedStep', 'Ts', ...
    'SolverMode', 'SingleTasking', 'StopTime', 'Tstop', ...
    'SignalLogging', 'on', 'SignalLoggingName', 'logsout', 'ReturnWorkspaceOutputs', 'on', ...
    'SaveTime', 'off', 'SaveOutput', 'off', 'SimscapeLogType', 'none');
cb = sprintf('run(fullfile(fileparts(get_param(''%s'',''FileName'')), ''params_02a_avg_pll.m''));', m);
set_param(m, 'PostLoadFcn', cb, 'InitFcn', cb);

note(m, [40 Y0-190], sprintf(['Stage 2a - Averaged inverter + SRF-PLL\n' ...
    'The inverter is an ideal controlled voltage source (= PWM averaged over one carrier period).\n' ...
    'Reference: fixed (vd_ref, vq_ref), rotated with the PLL angle -> it follows the grid phase/frequency.\n' ...
    'Grid events: +10 deg phase jump at 0.10 s, +0.5 Hz frequency step at 0.20 s.']));
note(m, [gr(1)+100 gn(2)+25], sprintf('Every Simscape network needs\none Solver Configuration (f(x)=0)'));
note(m, [xr-270 yV+85], sprintf('Controller runs every Tc = 200 us (one carrier period);\nthe plant runs every Ts = 10 us.'));

set_param(m, 'ZoomFactor', 'FitSystem');
save_system(m, f);
fprintf('Saved %s\n', f);
end

%% ======================= controller =======================
function buildController(c, LIB)
delete_line(c, 'In1/1', 'Out1/1');
set_param([c '/In1'], 'Name', 'Iabc', 'Position', [20 63 50 77]);
set_param([c '/Out1'], 'Name', 'vabc_ref', 'Position', [760 238 790 252]);
add_block('simulink/Sources/In1', [c '/Vabc'], 'Position', [20 163 50 177]);
rt = {'OutPortSampleTime', 'Tc'};
add_block('simulink/Signal Attributes/Rate Transition', [c '/ADC I'], 'Position', [90 55 130 85], rt{:});
add_block('simulink/Signal Attributes/Rate Transition', [c '/ADC V'], 'Position', [90 155 130 185], rt{:});
add_line(c, 'Iabc/1', 'ADC I/1'); add_line(c, 'Vabc/1', 'ADC V/1');
add_block('simulink/Sinks/Terminator', [c '/i (used in 2b)'], 'Position', [170 62 185 78]);
add_line(c, 'ADC I/1', 'i (used in 2b)/1');

% PLL
pll = [c '/SRF-PLL'];
add_block('simulink/Ports & Subsystems/Subsystem', pll, 'Position', [210 140 330 220]);
buildPLL(pll, LIB);
add_line(c, 'ADC V/1', 'SRF-PLL/1');

% open-loop dq reference rotated with the PLL angle
add_block('simulink/Sources/Constant', [c '/v_dq0 ref'], 'Position', [400 228 500 252], ...
    'Value', '[vd_ref; vq_ref; 0]');
add_block(LIB.ipark, [c '/Inverse Park'], 'Position', [540 225 610 275]);
add_block('simulink/Discrete/Unit Delay', [c '/Compute delay'], 'Position', [640 230 680 260], 'SampleTime', 'Tc');
% non-deterministic slow-to-fast transfer = plain zero-order hold (the deterministic
% default would add another full Tc of delay)
add_block('simulink/Signal Attributes/Rate Transition', [c '/DAC'], 'Position', [700 230 740 260], ...
    'OutPortSampleTime', 'Ts', 'Deterministic', 'off');
% delay compensation: theta + k_delay*Tc*w
add_block('simulink/Math Operations/Gain', [c '/k_delay*Tc*w'], 'Position', [400 285 470 305], 'Gain', 'k_delay*Tc*2*pi');
add_block('simulink/Math Operations/Sum', [c '/theta comp'], 'Inputs', '++', 'Position', [500 270 520 290]);
add_line(c, 'v_dq0 ref/1', 'Inverse Park/1');
add_line(c, 'SRF-PLL/2', 'k_delay*Tc*w/1', 'autorouting', 'smart');
add_line(c, 'SRF-PLL/1', 'theta comp/1', 'autorouting', 'smart');
add_line(c, 'k_delay*Tc*w/1', 'theta comp/2', 'autorouting', 'smart');
add_line(c, 'theta comp/1', 'Inverse Park/2', 'autorouting', 'smart');
add_line(c, 'Inverse Park/1', 'Compute delay/1'); add_line(c, 'Compute delay/1', 'DAC/1'); add_line(c, 'DAC/1', 'vabc_ref/1');

add_block('simulink/Sinks/Out1', [c '/theta_pll'], 'Position', [760 98 790 112]);
add_block('simulink/Sinks/Out1', [c '/f_pll'], 'Position', [760 133 790 147]);
add_block('simulink/Sinks/Out1', [c '/vdq'], 'Position', [760 168 790 182]);
add_line(c, 'SRF-PLL/1', 'theta_pll/1', 'autorouting', 'smart');
add_line(c, 'SRF-PLL/2', 'f_pll/1', 'autorouting', 'smart');
add_line(c, 'SRF-PLL/3', 'vdq/1', 'autorouting', 'smart');
set_param(c, 'ContentPreviewEnabled', 'off', 'MaskDisplay', 'disp(''Controller\nTc = 200 us'');');
end

function buildPLL(p, LIB)
% v_abc -> Park(theta) -> v_q / Vg_pk -> PI -> + w_g -> integrate (wrapped 0..2pi) -> theta
delete_line(p, 'In1/1', 'Out1/1');
set_param([p '/In1'], 'Name', 'Vabc', 'Position', [20 83 50 97]);
set_param([p '/Out1'], 'Name', 'theta', 'Position', [820 83 850 97]);
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
add_block('simulink/Signal Routing/Mux', [p '/Mux vdq'], 'Inputs', '2', 'Position', [240 195 245 235]);
add_block('simulink/Sinks/Terminator', [p '/zero seq'], 'Position', [240 250 255 265]);
add_block('simulink/Sinks/Out1', [p '/f'], 'Position', [820 143 850 157]);
add_block('simulink/Sinks/Out1', [p '/vdq'], 'Position', [820 208 850 222]);

add_line(p, 'Vabc/1', 'Park/1');
add_line(p, 'Park/1', 'Demux/1');
add_line(p, 'Demux/2', 'normalise/1');
add_line(p, 'normalise/1', 'PI/1');
add_line(p, 'no reset/1', 'PI/2');
add_line(p, 'PI/1', 'Sum/1');
add_line(p, 'w_g nominal/1', 'Sum/2', 'autorouting', 'smart');
add_line(p, 'Sum/1', 'Integrator (wrap 0..2pi)/1');
add_line(p, 'Sum/1', 'to Hz/1', 'autorouting', 'smart');
add_line(p, 'Integrator (wrap 0..2pi)/1', 'theta/1');
add_line(p, 'Integrator (wrap 0..2pi)/1', 'Park/2', 'autorouting', 'smart');
add_line(p, 'to Hz/1', 'f/1');
add_line(p, 'Demux/1', 'Mux vdq/1', 'autorouting', 'smart');
add_line(p, 'Demux/2', 'Mux vdq/2', 'autorouting', 'smart');
add_line(p, 'Demux/3', 'zero seq/1', 'autorouting', 'smart');
add_line(p, 'Mux vdq/1', 'vdq/1');
set_param(p, 'ContentPreviewEnabled', 'off', 'MaskDisplay', 'disp(''SRF-PLL'');');
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

function buildPQ(pq)
delete_line(pq, 'In1/1', 'Out1/1');
set_param([pq '/In1'], 'Name', 'Vabc', 'Position', [20 43 50 57]);
set_param([pq '/Out1'], 'Name', 'P', 'Position', [560 48 590 62]);
add_block('simulink/Sources/In1', [pq '/Iabc'], 'Position', [20 113 50 127]);
add_block('simulink/Signal Routing/Mux', [pq '/Mux'], 'Inputs', '2', 'Position', [100 30 105 140]);
add_block('simulink/User-Defined Functions/Fcn', [pq '/p (instantaneous)'], 'Position', [160 40 380 70], ...
    'Expression', 'u(1)*u(4)+u(2)*u(5)+u(3)*u(6)');
add_block('simulink/User-Defined Functions/Fcn', [pq '/q (instantaneous)'], 'Position', [160 100 380 130], ...
    'Expression', '((u(2)-u(3))*u(4)+(u(3)-u(1))*u(5)+(u(1)-u(2))*u(6))/sqrt(3)');
lpf = {'Numerator', '[1-exp(-Ts/tau_pq)]', 'Denominator', '[1 -exp(-Ts/tau_pq)]', 'SampleTime', 'Ts'};
add_block('simulink/Discrete/Discrete Transfer Fcn', [pq '/LPF P'], 'Position', [420 35 500 75], lpf{:});
add_block('simulink/Discrete/Discrete Transfer Fcn', [pq '/LPF Q'], 'Position', [420 95 500 135], lpf{:});
add_block('simulink/Sinks/Out1', [pq '/Q'], 'Position', [560 108 590 122]);
add_line(pq, 'Vabc/1', 'Mux/1'); add_line(pq, 'Iabc/1', 'Mux/2');
add_line(pq, 'Mux/1', 'p (instantaneous)/1'); add_line(pq, 'Mux/1', 'q (instantaneous)/1', 'autorouting', 'smart');
add_line(pq, 'p (instantaneous)/1', 'LPF P/1'); add_line(pq, 'q (instantaneous)/1', 'LPF Q/1');
add_line(pq, 'LPF P/1', 'P/1'); add_line(pq, 'LPF Q/1', 'Q/1');
set_param(pq, 'ContentPreviewEnabled', 'off', 'MaskDisplay', ...
    sprintf('port_label(''input'',1,''Vabc'');port_label(''input'',2,''Iabc'');port_label(''output'',1,''P'');port_label(''output'',2,''Q'');disp(''P / Q\\ncalc'');'));
end
