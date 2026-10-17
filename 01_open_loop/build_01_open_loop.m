function build_01_open_loop()
%BUILD_01_OPEN_LOOP Builds vsi_01_open_loop.slx from Simscape Electrical library blocks.
% Three-phase blocks use composite ports, so every three-phase connection is
% a single wire. Parameters come from params_01_open_loop.m.

here = fileparts(mfilename('fullpath'));
m = 'vsi_01_open_loop';
nl = newline;
run(fullfile(here, 'params_01_open_loop.m'));

if bdIsLoaded(m), close_system(m, 0); end
f = fullfile(here, [m '.slx']);
if isfile(f), delete(f); end
new_system(m); load_system('ee_lib'); load_system('fl_lib'); load_system('nesl_utility'); load_system('eePwmGeneratorTwoLevel');

LIB.grid = ['ee_lib/Sources/Voltage' nl 'Source' nl '(Three-Phase)'];
LIB.rlc  = 'ee_lib/Passive/RLC Assemblies/RLC (Three-Phase)';
LIB.vi   = ['ee_lib/Sensors &' nl 'Transducers/Current and Voltage' nl 'Sensor (Three-Phase)'];
LIB.conv = ['ee_lib/Semiconductors &' nl 'Converters/Converters/Converter' nl '(Three-Phase)'];
LIB.dc   = 'fl_lib/Electrical/Electrical Sources/DC Voltage Source';
LIB.ref  = 'fl_lib/Electrical/Electrical Elements/Electrical Reference';
LIB.sol  = ['nesl_utility/Solver' nl 'Configuration'];
LIB.ps2s = ['nesl_utility/PS-Simulink' nl 'Converter'];
LIB.s2ps = ['nesl_utility/Simulink-PS' nl 'Converter'];
LIB.pwm  = ['eePwmGeneratorTwoLevel/PWM Generator' nl '(Three-phase, Two-level)'];
COMPOSITE = 'ee.enum.threePhasePort.composite';

%% ---------------- Power stage, left to right: DC -> bridge -> filter -> PCC -> grid ----------------
Y0 = 420;
blk = @(name) [m '/' name];
br = blk('Two-Level IGBT Bridge');

% Bridge mirrored: DC ports (+,-) on the left, gate G and AC port on the right
add_block(LIB.conv, br, 'Position', [200 Y0-55 300 Y0+55], 'port_option', COMPOSITE, ...
    'device_type', 'ee.enum.converters.switchingdevice.igbt', ...
    'Vth', '0.5', 'Vf', 'Vf_sw', 'Ron', 'Ron_sw', 'Goff', '1e-6', ...
    'diode_param', 'ee.enum.converters.protectiondiode.nodynamics', ...
    'diode_Vf', 'Vf_sw', 'diode_Ron', 'Ron_sw', 'diode_Goff', '1e-6', 'snubber_type', '0', ...
    'BlockMirror', 'on');
ac = portPos(br, 'LConn', 2); yLine = ac(2);
gPort = portPos(br, 'LConn', 1);

bp = portPos(br, 'RConn', 1); bn = portPos(br, 'RConn', 2);
xdc = bp(1) - 90;
add_block(LIB.dc, blk('DC Source'), 'Position', [xdc-15 bp(2)+10 xdc+15 bn(2)-10], 'v0', 'Vdc');

add_block(LIB.rlc, blk('L Filter'), 'Position', [470 0 530 40], 'port_option', COMPOSITE, ...
    'component_structure', '4', 'R', 'Rf', 'L', 'Lf');
alignPort(blk('L Filter'), 'LConn', 1, [0 yLine]);

% PCC sensor: ~1 (left) filter side, ~2 (right, bottom) grid side -> current > 0 into the grid
add_block(LIB.vi, blk('PCC V-I'), 'Position', [600 0 660 75], 'port_option', COMPOSITE, ...
    'vMeasurementType', '2', 'outputUnit', '2');
alignPort(blk('PCC V-I'), 'LConn', 1, [0 yLine]);
p2 = portPos(blk('PCC V-I'), 'RConn', 3);

% Grid mirrored: three-phase port on the left, neutral on the right
add_block(LIB.grid, blk('Grid'), 'Position', [760 0 820 60], ...
    'vline_rms', 'Vg_ll', 'freq', 'fg', 'shift', '0', 'impedance_option', '4', 'R', 'Rg', 'L', 'Lg', ...
    'BlockMirror', 'on');
alignPort(blk('Grid'), 'RConn', 1, p2);
gn = portPos(blk('Grid'), 'LConn', 1);
add_block(LIB.ref, blk('Ground'), 'Position', [gn(1)+30 gn(2)+70 gn(1)+60 gn(2)+95]);
gr = portPos(blk('Ground'), 'LConn', 1);
add_block(LIB.sol, blk('Solver Configuration'), 'Position', [gr(1)+40 gn(2)+20 gr(1)+85 gn(2)+50], ...
    'UseLocalSolver', 'on', 'LocalSolverChoice', 'NE_BACKWARD_EULER_ADVANCER', ...
    'LocalSolverSampleTime', 'Ts', 'DoFixedCost', 'off', 'BlockMirror', 'on');

% wires
dp = portPos(blk('DC Source'), 'LConn', 1); dn = portPos(blk('DC Source'), 'RConn', 1);
add_line(m, [dp; dp(1) bp(2); bp]);
add_line(m, [dn; dn(1) bn(2); bn]);
wire(m, br, 'LConn', 2, blk('L Filter'), 'LConn', 1);
wire(m, blk('L Filter'), 'RConn', 1, blk('PCC V-I'), 'LConn', 1);
wire(m, blk('PCC V-I'), 'RConn', 3, blk('Grid'), 'RConn', 1);
add_line(m, [gn; gr(1) gn(2); gr]);
sp = portPos(blk('Solver Configuration'), 'RConn', 1);
assert(sp(2) > gn(2) && sp(2) < gr(2), 'solver branch must land on the vertical ground wire');
add_line(m, [sp; gr(1) sp(2)]);                      % T-junction on the grid-to-ground wire

%% ---------------- Open-loop modulation (above the bridge) ----------------
gi = blk('Gate Interface');
% mirrored as well: gates in on the right, physical G out on the left (towards the bridge)
add_block('simulink/Ports & Subsystems/Subsystem', gi, 'Position', [gPort(1)+50 gPort(2)-20 gPort(1)+120 gPort(2)+20]);
buildGateInterface(gi, LIB);
set_param(gi, 'BlockMirror', 'on');
alignPort(gi, 'RConn', 1, gPort);
wire(m, gi, 'RConn', 1, br, 'LConn', 1);

yc = gPort(2) - 150;
add_block('simulink/Sources/Sine Wave', blk('Vref_abc'), 'Position', [80 yc-20 130 yc+20], ...
    'Amplitude', 'Vinv_pk', 'Frequency', 'wg', 'Phase', '[delta, delta-2*pi/3, delta+2*pi/3]', 'SampleTime', 'Ts');
add_block('simulink/Sources/Constant', blk('Vdc_nom'), 'Position', [80 yc+40 130 yc+70], 'Value', 'Vdc', 'SampleTime', 'Ts');
add_block(LIB.pwm, blk('PWM Generator'), 'Position', [200 yc-25 310 yc+75], 'fsw', 'fsw', 'Ts', 'Ts');
add_block('simulink/Sinks/Terminator', blk('ModWave'), 'Position', [345 yc+45 360 yc+60]);
add_line(m, 'Vref_abc/1', 'PWM Generator/1');
add_line(m, 'Vdc_nom/1', 'PWM Generator/2');
add_line(m, 'PWM Generator/2', 'ModWave/1');
gOut = portPos(blk('PWM Generator'), 'Outport', 1); gIn = portPos(gi, 'Inport', 1);
lg = add_line(m, [gOut; gIn(1)+25 gOut(2); gIn(1)+25 gIn(2); gIn]); set_param(lg, 'Name', 'gates');

%% ---------------- Measurements and P/Q (below the power row) ----------------
vP = portPos(blk('PCC V-I'), 'RConn', 1); iP = portPos(blk('PCC V-I'), 'RConn', 2);
xs = vP(1) + 45; yV = yLine + 170; yI = yLine + 220;
add_block(LIB.ps2s, blk('V PS-S'), 'Position', [xs yV-10 xs+30 yV+10], 'VectorFormat', '1-D array');
add_block(LIB.ps2s, blk('I PS-S'), 'Position', [xs yI-10 xs+30 yI+10], 'VectorFormat', '1-D array');
% sensor outputs drop below the power wire (signal wires cross it, no junction)
add_line(m, [vP; vP(1)+25 vP(2); vP(1)+25 yV; xs yV]);
add_line(m, [iP; iP(1)+12 iP(2); iP(1)+12 yI; xs yI]);
pq = blk('PQ Calculation');
x0 = xs + 130;
add_block('simulink/Ports & Subsystems/Subsystem', pq, 'Position', [x0 yV-25 x0+90 yI+25]);
buildPQ(pq);
lv = add_line(m, 'V PS-S/1', 'PQ Calculation/1', 'autorouting', 'smart'); set_param(lv, 'Name', 'Vabc_pcc');
li = add_line(m, 'I PS-S/1', 'PQ Calculation/2', 'autorouting', 'smart'); set_param(li, 'Name', 'Iabc_pcc');
q = get_param(pq, 'Position');
add_block('simulink/Sinks/Scope', blk('P & Q'), 'NumInputPorts', '2', 'Position', [q(3)+70 q(2) q(3)+100 q(4)]);
lp = add_line(m, 'PQ Calculation/1', 'P & Q/1'); set_param(lp, 'Name', 'P');
lq = add_line(m, 'PQ Calculation/2', 'P & Q/2'); set_param(lq, 'Name', 'Q');
add_block('simulink/Sinks/Scope', blk('Grid V & I'), 'NumInputPorts', '2', 'Position', [x0+30 q(4)+25 x0+60 q(4)+85]);
add_line(m, 'V PS-S/1', 'Grid V & I/1', 'autorouting', 'smart');
add_line(m, 'I PS-S/1', 'Grid V & I/2', 'autorouting', 'smart');

for s = {'Vabc_pcc', lv; 'Iabc_pcc', li; 'P', lp; 'Q', lq; 'gates', lg}'
    src = get_param(s{2}, 'SrcPortHandle');
    set_param(src, 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', s{1});
end

%% ---------------- Model settings ----------------
set_param(m, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', 'FixedStep', 'Ts', ...
    'StopTime', 'Tstop', 'SignalLogging', 'on', 'SignalLoggingName', 'logsout', 'ReturnWorkspaceOutputs', 'on', ...
    'SaveTime', 'off', 'SaveOutput', 'off', ...
    'SimscapeLogType', 'none');   % R2026a default 'all' logs every Simscape variable at 1 us -> out of memory
% Load parameters from the file next to the model, whatever the current folder is.
cb = sprintf('run(fullfile(fileparts(get_param(''%s'',''FileName'')), ''params_01_open_loop.m''));', m);
set_param(m, 'PostLoadFcn', cb, 'InitFcn', cb);

%% ---------------- Notes on the canvas ----------------
note(m, [60 yc-110], sprintf(['Step 1 - Open-loop grid-connected VSI\n' ...
    'Vref_abc is computed in params_01_open_loop.m from (P_ref, Q_ref) with the phasor model\n' ...
    '   E = Vg + Z I,   Z = (Rf+Rg) + j wg (Lf+Lg)   ->   amplitude = sqrt(2)|E|,  phase = delta = angle(E)\n' ...
    'Try changing P_ref, Vdc or Lf only in the model (not in the params file): P/Q drift away -> step 2 adds current control.']));
note(m, [gr(1)+100 gn(2)+25], sprintf('Every Simscape network needs\none Solver Configuration (f(x)=0)'));
note(m, [60 yc+85], sprintf('Open-loop modulation: fixed sine references,\nno feedback from the grid.'));
note(m, [q(3)+130 q(2)], sprintf('p = va ia + vb ib + vc ic\nq = [(vb-vc) ia + (vc-va) ib + (va-vb) ic] / sqrt(3)\nboth low-pass filtered (tau_pq)'));

set_param(m, 'ZoomFactor', 'FitSystem');
save_system(m, f);
fprintf('Saved %s\n', f);
end

%% ======================= helpers =======================
function p = portPos(b, side, k)
pc = get_param(b, 'PortHandles');
p = get_param(pc.(side)(k), 'Position');
end

function y = portPos2(b, side, k)
p = portPos(b, side, k); y = p(2);
end

function alignPort(b, side, k, target)
% shift block b vertically so that port (side,k) has y = target(2)
p = portPos(b, side, k); pos = get_param(b, 'Position');
dy = target(2) - p(2);
set_param(b, 'Position', pos + [0 dy 0 dy]);
end

function wire(m, b1, s1, k1, b2, s2, k2, varargin)
a = portPos(b1, s1, k1); b = portPos(b2, s2, k2);
if numel(varargin) == 2
    pts = varargin{2}(a, b);
elseif a(2) == b(2)
    pts = [a; b];
else
    xm = round((a(1) + b(1))/2); pts = [a; xm a(2); xm b(2); b];
end
add_line(m, pts);
end

function note(m, xy, txt)
a = Simulink.Annotation(m, txt);
a.Position = [xy xy + [10 10]];
a.HorizontalAlignment = 'left';
a.FontSize = 10;
end

function buildGateInterface(gi, LIB)
% boolean gate vector [a+ a- b+ b- c+ c-] -> six physical signals -> converter gate port G
nl = newline;
delete_line(gi, 'In1/1', 'Out1/1');
delete_block([gi '/Out1']);
set_param([gi '/In1'], 'Name', 'gates', 'Position', [20 133 50 147]);
add_block('simulink/Signal Attributes/Data Type Conversion', [gi '/to double'], ...
    'Position', [80 125 120 155], 'OutDataTypeStr', 'double');
add_block('simulink/Signal Routing/Demux', [gi '/Demux'], 'Outputs', '6', 'Position', [150 40 155 240]);
add_line(gi, 'gates/1', 'to double/1'); add_line(gi, 'to double/1', 'Demux/1');
names = {'a+','a-','b+','b-','c+','c-'};
add_block([strtok(LIB.conv, '/') '/Semiconductors &' nl 'Converters/Converters/Six-Pulse Gate' nl 'Multiplexer'], ...
    [gi '/Six-Pulse Gate Multiplexer'], 'Position', [320 40 380 240], 'port_option', 'ee.enum.controlport.ps');
pm = get_param([gi '/Six-Pulse Gate Multiplexer'], 'PortHandles');
for k = 1:6
    y = get_param(pm.LConn(k), 'Position'); y = y(2);
    b = [gi '/S-PS ' names{k}];
    add_block(LIB.s2ps, b, 'Position', [220 y-10 250 y+10], 'FilteringAndDerivatives', 'zero');
    add_line(gi, sprintf('Demux/%d', k), ['S-PS ' names{k} '/1'], 'autorouting', 'smart');
    pb = get_param(b, 'PortHandles');
    add_line(gi, pb.RConn(1), pm.LConn(k));
end
add_block('nesl_utility/Connection Port', [gi '/G'], 'Position', [430 133 460 147], 'Side', 'Right');
pg = get_param([gi '/G'], 'PortHandles');
add_line(gi, pm.RConn(1), [pg.LConn pg.RConn]);
set_param(gi, 'ContentPreviewEnabled', 'off', 'MaskDisplay', 'disp(''Gate\ninterface'');', ...
    'MaskDescription', 'Converts the boolean PWM gate vector [a+ a- b+ b- c+ c-] into the converter gate port (Q1..Q6).');
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
set_param(pq, 'MaskDescription', 'Instantaneous three-phase active/reactive power with first-order low-pass filter.');
end
