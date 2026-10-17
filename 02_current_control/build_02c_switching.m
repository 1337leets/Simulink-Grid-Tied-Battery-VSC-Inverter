function build_02c_switching()
%BUILD_02C_SWITCHING Stage 2c: stage 2b controller + switching IGBT bridge of step 1.
% The bridge, the gate interface and the controller are copied unchanged from
% vsi_01_open_loop and vsi_02b_current_ctrl: the controller does not know
% whether it drives an averaged or a switching inverter.
% New here: DC link as a Simscape source with a measured Vdc, carrier-based PWM
% with a start-up enable, measurements averaged over one carrier period.
% (Only a parameter differs: k_delay = 2 instead of 1.5, see params_02c.)

here = fileparts(mfilename('fullpath'));
m = 'vsi_02c_switching';
nl = newline;
run(fullfile(here, 'params_02c_switching.m'));

% source models (built if missing)
src1 = 'vsi_01_open_loop'; src2 = 'vsi_02b_current_ctrl';
f1 = fullfile(here, '..', '01_open_loop', [src1 '.slx']);
f2 = fullfile(here, [src2 '.slx']);
if ~isfile(f1), run(fullfile(here, '..', '01_open_loop', 'build_01_open_loop.m')); end
if ~isfile(f2), build_02b_current_ctrl(); end
load_system(f1); load_system(f2);

if bdIsLoaded(m), close_system(m, 0); end
f = fullfile(here, [m '.slx']);
if isfile(f), delete(f); end
new_system(m);
cellfun(@load_system, {'ee_lib', 'fl_lib', 'nesl_utility'});

LIB.pgrid = findLib('ee_lib/Sources', '^Programmable.*Three');
LIB.rlc  = 'ee_lib/Passive/RLC Assemblies/RLC (Three-Phase)';
LIB.vi   = ['ee_lib/Sensors &' nl 'Transducers/Current and Voltage' nl 'Sensor (Three-Phase)'];
LIB.ref  = 'fl_lib/Electrical/Electrical Elements/Electrical Reference';
LIB.cvs  = ['fl_lib/Electrical/Electrical Sources/Controlled Voltage' nl 'Source'];
LIB.vs   = 'fl_lib/Electrical/Electrical Sensors/Voltage Sensor';
LIB.sol  = ['nesl_utility/Solver' nl 'Configuration'];
LIB.ps2s = ['nesl_utility/PS-Simulink' nl 'Converter'];
LIB.s2ps = ['nesl_utility/Simulink-PS' nl 'Converter'];
LIB.port = 'nesl_utility/Connection Port';
COMPOSITE = 'ee.enum.threePhasePort.composite';
blk = @(name) [m '/' name];

%% ---------------- Power stage: DC link -> bridge -> filter -> PCC -> grid ----------------
br = blk('Two-Level IGBT Bridge');
add_block([src1 '/Two-Level IGBT Bridge'], br);          % mirrored: DC on the left, G and AC on the right
ac = portPos(br, 'LConn', 2); yLine = ac(2);
gPort = portPos(br, 'LConn', 1);
bp = portPos(br, 'RConn', 1); bn = portPos(br, 'RConn', 2);

dcl = blk('DC link');
add_block('simulink/Ports & Subsystems/Subsystem', dcl, 'Position', [bp(1)-150 bp(2)-60 bp(1)-80 bn(2)+20]);
buildDCLink(dcl, LIB);
alignPort(dcl, 'RConn', 1, bp);
dp = portPos(dcl, 'RConn', 1); dn = portPos(dcl, 'RConn', 2);
add_line(m, [dp; bp]);
xm = round((dn(1) + bn(1))/2);
add_line(m, [dn; xm dn(2); xm bn(2); bn]);

add_block(LIB.rlc, blk('L Filter'), 'Position', [470 0 530 40], 'port_option', COMPOSITE, ...
    'component_structure', '4', 'R', 'Rf', 'L', 'Lf');
alignPort(blk('L Filter'), 'LConn', 1, [0 yLine]);
add_block(LIB.vi, blk('PCC V-I'), 'Position', [600 0 660 75], 'port_option', COMPOSITE, ...
    'vMeasurementType', '2', 'outputUnit', '2');
alignPort(blk('PCC V-I'), 'LConn', 1, [0 yLine]);
p2 = portPos(blk('PCC V-I'), 'RConn', 3);

grid = blk('Programmable Grid');
add_block(LIB.pgrid, grid, 'Position', [780 0 840 60], 'BlockMirror', 'on', ...
    'vline_rms', 'Vg_ll', 'freq', 'fg', 'shift', '0', ...
    'phase_type', 'ee.enum.timechanging.step', 'phase_step', 'dphi_deg', 'phase_t1', 't_phase', 'phase_t2', 'inf', ...
    'impedance_option', '4', 'R', 'Rg', 'L', 'Lg');
alignPort(grid, 'RConn', 1, p2);
gn = portPos(grid, 'LConn', 1);
add_block(LIB.ref, blk('Ground'), 'Position', [gn(1)+30 gn(2)+70 gn(1)+60 gn(2)+95]);
gr = portPos(blk('Ground'), 'LConn', 1);
% Partitioning local solver: splits the network into small linear pieces per
% switch state; about 20x faster than Backward Euler for this switching model.
add_block(LIB.sol, blk('Solver Configuration'), 'Position', [gr(1)+40 gn(2)+20 gr(1)+85 gn(2)+50], ...
    'UseLocalSolver', 'on', 'LocalSolverChoice', 'NE_PARTITIONING_ADVANCER', ...
    'LocalSolverSampleTime', 'Ts', 'DoFixedCost', 'off', 'BlockMirror', 'on');

wire(m, br, 'LConn', 2, blk('L Filter'), 'LConn', 1);
wire(m, blk('L Filter'), 'RConn', 1, blk('PCC V-I'), 'LConn', 1);
wire(m, blk('PCC V-I'), 'RConn', 3, grid, 'RConn', 1);
add_line(m, [gn; gr(1) gn(2); gr]);
sp = portPos(blk('Solver Configuration'), 'RConn', 1);
add_line(m, [sp; gr(1) sp(2)]);

% measured Vdc is distributed with Goto/From (one tag, three users)
vo = portPos(dcl, 'Outport', 1);
add_block('simulink/Signal Routing/Goto', blk('Goto Vdc'), 'GotoTag', 'Vdc', 'ShowName', 'off', ...
    'Position', [vo(1)+15 vo(2)-8 vo(1)+50 vo(2)+8]);
ld = add_line(m, [vo; vo(1)+15 vo(2)]);     % logged as Vdc (no label: the line is short)

%% ---------------- Modulation (above the bridge) ----------------
gi = blk('Gate Interface');
add_block([src1 '/Gate Interface'], gi);                 % mirrored: gates in on the right, G out on the left
alignPort(gi, 'RConn', 1, gPort);
wire(m, gi, 'RConn', 1, br, 'LConn', 1);

yc = gPort(2) - 150;
pwm = blk('PWM');
add_block('simulink/Ports & Subsystems/Subsystem', pwm, 'Position', [bp(1)+15 yc-25 bp(1)+125 yc+75]);
buildPWM(pwm);
gOut = portPos(pwm, 'Outport', 1); gIn = portPos(gi, 'Inport', 1);
lg = add_line(m, [gOut; gIn(1)+25 gOut(2); gIn(1)+25 gIn(2); gIn]); set_param(lg, 'Name', 'gates');
pv = portPos(pwm, 'Inport', 2);
add_block('simulink/Signal Routing/From', blk('From Vdc (PWM)'), 'GotoTag', 'Vdc', 'ShowName', 'off', ...
    'Position', [pv(1)-65 pv(2)-10 pv(1)-20 pv(2)+10]);
add_line(m, [pv(1)-20 pv(2); pv]);

%% ---------------- Controller (copied from stage 2b, mirrored) ----------------
vP = portPos(blk('PCC V-I'), 'RConn', 1); iP = portPos(blk('PCC V-I'), 'RConn', 2);
xr = vP(1); yTop = yLine + 85;
ctrl = blk('Controller');
add_block([src2 '/Controller'], ctrl, 'Position', [xr-380 yTop xr-210 yTop+360]);
yIn = arrayfun(@(k) getfield(portPos(ctrl, 'Inport', k), {2}), 1:4); %#ok<GFLD>

% measurements: PS-S -> average over one carrier period -> controller (all mirrored)
add_block(LIB.ps2s, blk('I PS-S'), 'Position', [xr-30 yIn(1)-10 xr yIn(1)+10], 'VectorFormat', '1-D array', 'BlockMirror', 'on');
add_block(LIB.ps2s, blk('V PS-S'), 'Position', [xr-30 yIn(2)-10 xr yIn(2)+10], 'VectorFormat', '1-D array', 'BlockMirror', 'on');
av = {'Coefficients', 'b_avg', 'SampleTime', 'Ts', 'BlockMirror', 'on'};
FIR = 'simulink/Discrete/Discrete FIR Filter';
add_block(FIR, blk('Period avg I'), 'Position', [xr-150 yIn(1)-15 xr-100 yIn(1)+15], av{:});
add_block(FIR, blk('Period avg V'), 'Position', [xr-150 yIn(2)-15 xr-100 yIn(2)+15], av{:});
add_line(m, [iP; iP(1)+12 iP(2); iP(1)+12 yIn(1); xr yIn(1)]);
add_line(m, [vP; vP(1)+25 vP(2); vP(1)+25 yIn(2); xr yIn(2)]);
li = wireSig(m, blk('I PS-S'), 1, blk('Period avg I'), 1); set_param(li, 'Name', 'Iabc_pcc');
lv = wireSig(m, blk('V PS-S'), 1, blk('Period avg V'), 1); set_param(lv, 'Name', 'Vabc_pcc');
wireSig(m, blk('Period avg I'), 1, ctrl, 1);
wireSig(m, blk('Period avg V'), 1, ctrl, 2);

% EMS commands (copied from stage 2b) on row 3, measured Vdc through an ADC on row 4
ems = blk('EMS commands');
add_block([src2 '/EMS commands'], ems, 'Position', [xr-60 yIn(3)-22 xr yIn(3)+22]);
le = wireSig(m, ems, 1, ctrl, 3); set_param(le, 'Name', 'PQ_ref');
add_block('simulink/Signal Routing/From', blk('From Vdc'), 'GotoTag', 'Vdc', 'ShowName', 'off', ...
    'BlockMirror', 'on', 'Position', [xr-45 yIn(4)-10 xr yIn(4)+10]);
add_block('simulink/Signal Attributes/Rate Transition', blk('ADC Vdc'), 'OutPortSampleTime', 'Tc', ...
    'BlockMirror', 'on', 'Position', [xr-145 yIn(4)-15 xr-105 yIn(4)+15]);
wireSig(m, blk('From Vdc'), 1, blk('ADC Vdc'), 1);
wireSig(m, blk('ADC Vdc'), 1, ctrl, 4);

% controller output 1 -> PWM (lane on the far left, around the DC link)
o = arrayfun(@(k) portPos(ctrl, 'Outport', k), 1:6, 'UniformOutput', false);
pr = portPos(pwm, 'Inport', 1);
xl = get_param(dcl, 'Position'); xl = xl(1) - 30;
lr = add_line(m, [o{1}; xl o{1}(2); xl pr(2); pr]); set_param(lr, 'Name', 'vabc_ref');

% scopes on the output rows (mirrored), as in stage 2b
d = o{3}(2) - o{2}(2);
xs = xl + 45;
add_block('simulink/Sinks/Scope', blk('PLL'), 'NumInputPorts', '2', 'BlockMirror', 'on', ...
    'Position', [xs o{2}(2)-d/2 xs+35 o{3}(2)+d/2], 'NamePlacement', 'alternate');
add_block('simulink/Sinks/Scope', blk('Current loop'), 'NumInputPorts', '3', 'BlockMirror', 'on', ...
    'Position', [xs o{4}(2)-d/2 xs+35 o{6}(2)+d/2]);
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
close_system(src1, 0); close_system(src2, 0);

%% ---------------- Model settings ----------------
set_param(m, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', 'FixedStep', 'Ts', ...
    'SolverMode', 'SingleTasking', 'StopTime', 'Tstop', ...
    'SignalLogging', 'on', 'SignalLoggingName', 'logsout', 'ReturnWorkspaceOutputs', 'on', ...
    'SaveTime', 'off', 'SaveOutput', 'off', 'SimscapeLogType', 'none');
cb = sprintf('run(fullfile(fileparts(get_param(''%s'',''FileName'')), ''params_02c_switching.m''));', m);
set_param(m, 'PostLoadFcn', cb, 'InitFcn', cb);

dpos = get_param(dcl, 'Position');
note(m, [dpos(1) yc-130], sprintf(['Stage 2c - Switching IGBT bridge + the stage 2b controller (same blocks, k_delay = 2)\n' ...
    'Copied blocks: bridge and gate interface from step 1, controller and EMS commands from stage 2b.\n' ...
    'PWM: one carrier period per controller period, the controller samples on the carrier peaks.\n' ...
    'Same test as stage 2b; run_02c_switching.m compares the two models.']));
note(m, [gr(1)+100 gn(2)+25], sprintf('Every Simscape network needs\none Solver Configuration (f(x)=0)'));
note(m, [dpos(1) dpos(4)+15], sprintf('Step 3 replaces\nthis block with\nthe battery.'));
note(m, [xr+40 yIn(1)-10], sprintf(['Oversampling ADC: average over one carrier\n' ...
    'period (FIR, b_avg) removes the switching ripple.']));
note(m, [xr+40 yIn(3)-10], sprintf('EMS test profile (lookup tables).'));

set_param(m, 'ZoomFactor', 'FitSystem');
save_system(m, f);
fprintf('Saved %s\n', f);
end

%% ======================= subsystems =======================
function buildDCLink(s, LIB)
% Vdc profile -> controlled voltage source between + and -, voltage sensor -> Vdc
% (stage 2c: an ideal source; step 3 puts the battery here with the same ports)
delete_line(s, 'In1/1', 'Out1/1');
delete_block([s '/In1']);
b = @(n) [s '/' n];
add_block('simulink/Sources/Digital Clock', b('t'), 'SampleTime', 'Tc', 'Position', [20 240 50 260]);
add_block('simulink/Lookup Tables/1-D Lookup Table', b('Vdc profile'), 'Position', [80 230 140 270], ...
    'BreakpointsForDimension1', 't_V', 'Table', 'V_V', 'ExtrapMethod', 'Clip');
add_block(LIB.s2ps, b('S-PS'), 'Position', [170 240 200 260], 'FilteringAndDerivatives', 'zero');
add_block(LIB.cvs, b('Source'), 'Position', [250 100 290 160]);
add_block(LIB.vs, b('Sensor'), 'Position', [350 100 390 160]);
add_block(LIB.ps2s, b('PS-S'), 'Position', [400 200 430 220]);
add_block(LIB.port, b('+'), 'Side', 'Right', 'Position', [540 53 570 67]);
add_block(LIB.port, b('-'), 'Side', 'Right', 'Position', [540 168 570 182]);
set_param(b('Out1'), 'Name', 'Vdc', 'Position', [460 203 490 217]);
add_line(s, 't/1', 'Vdc profile/1');
add_line(s, 'Vdc profile/1', 'S-PS/1');
% Source: LConn1 = + (top), RConn1 = control input, RConn2 = - (both at the bottom)
% Sensor: LConn1 = + (top), RConn1 = V, RConn2 = - (both at the bottom)
P = @(blk, side, k) portPos(b(blk), side, k);
c = P('Source', 'RConn', 1); u = P('S-PS', 'RConn', 1);
add_line(s, [u; c(1) u(2); c]);
sp = P('Source', 'LConn', 1); ep = P('+', 'RConn', 1); np = P('Sensor', 'LConn', 1);
add_line(s, [sp; sp(1) ep(2); ep]);                    % + rail along the top
add_line(s, [np(1) ep(2); np]);
nn = P('Sensor', 'RConn', 2); em = P('-', 'RConn', 1); sn = P('Source', 'RConn', 2);
add_line(s, [nn; nn(1) em(2); em]);                    % - rail to the port ...
add_line(s, [510 em(2); 510 280; sn(1) 280; sn]);      % ... and around the bottom to the source
v = P('Sensor', 'RConn', 1); pin = P('PS-S', 'LConn', 1);
add_line(s, [v; v(1) pin(2); pin]);
add_line(s, 'PS-S/1', 'Vdc/1');
set_param(s, 'ContentPreviewEnabled', 'off', 'MaskDisplay', ...
    sprintf('disp(''DC link\\n(ideal\\nsource)'');port_label(''output'',1,''Vdc'');port_label(''rconn'',1,''+'');port_label(''rconn'',2,''-'');'));
end

function buildPWM(s)
% m = v_ref/(Vdc/2) compared with a triangular carrier (+1 -> -1 -> +1 every Tc).
% upper switch on while m > carrier, lower switch is the complement (no dead time).
% Vdc is sampled at Tc and used one period later, together with the reference it
% scales (the measured Vdc fed straight through would be an algebraic loop:
% Vdc -> gates -> Simscape network -> Vdc).
delete_line(s, 'In1/1', 'Out1/1');
b = @(n) [s '/' n];
set_param(b('In1'), 'Name', 'vabc_ref', 'Position', [20 43 50 57]);
add_block('simulink/Sources/In1', b('Vdc'), 'Position', [20 93 50 107]);
add_block('simulink/Discrete/Unit Delay', b('Vdc sample'), 'SampleTime', 'Tc', 'InitialCondition', 'Vdc', ...
    'Position', [80 85 120 115]);
add_block('simulink/Math Operations/Product', b('v by Vdc'), 'Inputs', '*/', 'Position', [160 40 190 110]);
add_block('simulink/Math Operations/Gain', b('x2'), 'Gain', '2', 'Position', [220 60 260 90]);
add_block('simulink/Sources/Repeating Sequence Stair', b('carrier'), 'OutValues', 'carrier_seq', 'tsamp', 'Ts', ...
    'Position', [220 130 260 160]);
add_block('simulink/Logic and Bit Operations/Relational Operator', b('m > carrier'), 'Operator', '>', ...
    'Position', [300 65 330 145]);
add_block('simulink/Logic and Bit Operations/Logical Operator', b('lower = NOT upper'), 'Operator', 'NOT', ...
    'Position', [370 150 400 180]);
add_block('simulink/Signal Routing/Mux', b('Mux'), 'Inputs', '2', 'Position', [450 80 455 180]);
add_block('simulink/Signal Routing/Selector', b('a+ a- b+ b- c+ c-'), 'IndexParamArray', {'[1 4 2 5 3 6]'}, ...
    'InputPortWidth', '6', 'Position', [490 115 530 145]);
% all six IGBTs stay off until t_en (the diodes block: Vdc > grid line peak)
add_block('simulink/Sources/Step', b('enable'), 'Time', 't_en', 'Before', '0', 'After', '1', 'SampleTime', 'Ts', ...
    'Position', [490 170 530 200]);
add_block('simulink/Logic and Bit Operations/Logical Operator', b('AND'), 'Operator', 'AND', ...
    'Position', [570 117 600 163]);
set_param(b('Out1'), 'Name', 'gates', 'Position', [640 123 670 137]);
add_line(s, 'vabc_ref/1', 'v by Vdc/1');
add_line(s, 'Vdc/1', 'Vdc sample/1');
add_line(s, 'Vdc sample/1', 'v by Vdc/2');
add_line(s, 'v by Vdc/1', 'x2/1');
add_line(s, 'x2/1', 'm > carrier/1', 'autorouting', 'smart');
add_line(s, 'carrier/1', 'm > carrier/2', 'autorouting', 'smart');
l = add_line(s, 'm > carrier/1', 'Mux/1', 'autorouting', 'smart'); set_param(l, 'Name', 'upper');
add_line(s, 'm > carrier/1', 'lower = NOT upper/1', 'autorouting', 'smart');
add_line(s, 'lower = NOT upper/1', 'Mux/2', 'autorouting', 'smart');
add_line(s, 'Mux/1', 'a+ a- b+ b- c+ c-/1');
add_line(s, 'a+ a- b+ b- c+ c-/1', 'AND/1', 'autorouting', 'smart');
add_line(s, 'enable/1', 'AND/2', 'autorouting', 'smart');
add_line(s, 'AND/1', 'gates/1');
set_param(s, 'ContentPreviewEnabled', 'off', 'MaskDisplay', ...
    sprintf('port_label(''input'',1,''vabc_ref'');port_label(''input'',2,''Vdc'');port_label(''output'',1,''gates'');disp(''PWM\\n(carrier)'');'));
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
