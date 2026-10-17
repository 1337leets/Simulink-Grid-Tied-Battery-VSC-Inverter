function build_03b_lcl()
%BUILD_03B_LCL Step 3b: LCL filter instead of the L filter of step 3a.
% Starts from the step 3a model (Save As) and changes only what step 3b needs:
%   - "L Filter" -> "LCL filter" (L1, Cf + Rd in star, L2) with a sensor for
%     the inverter-side current i1
%   - "Current feedback" (Variant Source) chooses which current the controller
%     regulates: grid side (PCC) or inverter side (fb_side = 1 / 2)
%   - "Q comp." corrects Q* for Cf when the inverter-side current is used
% Battery, BMS, PWM and the controller stay as in step 3a.

here = fileparts(mfilename('fullpath'));
m = 'vsi_03b_lcl'; src = 'vsi_03a_battery';
nl = newline;

f3a = fullfile(here, [src '.slx']);
if ~isfile(f3a), build_03a_battery(); end
for s = {m, src}, if bdIsLoaded(s{1}), close_system(s{1}, 0); end, end
f = fullfile(here, [m '.slx']);
if isfile(f), delete(f); end
load_system(f3a);
save_system(src, f);                         % Save As: the loaded model is now vsi_03b_lcl
cb = sprintf('run(fullfile(fileparts(get_param(''%s'',''FileName'')), ''params_03b_lcl.m''));', m);
set_param(m, 'PostLoadFcn', cb, 'InitFcn', cb);
run(fullfile(here, 'params_03b_lcl.m'));
cellfun(@load_system, {'ee_lib', 'nesl_utility'});

LIB.rlc  = 'ee_lib/Passive/RLC Assemblies/RLC (Three-Phase)';
LIB.is   = ['ee_lib/Sensors &' nl 'Transducers/Current Sensor' nl '(Three-Phase)'];
LIB.fn   = ['ee_lib/Connectors &' nl 'References/Grounded Neutral' nl '(Three-Phase)'];
LIB.ps2s = ['nesl_utility/PS-Simulink' nl 'Converter'];
LIB.port = 'nesl_utility/Connection Port';
COMPOSITE = 'ee.enum.threePhasePort.composite';
blk = @(name) [m '/' name];

%% ---------------- L Filter -> LCL filter ----------------
lf = blk('L Filter');
pos = get_param(lf, 'Position');
lh = get_param(lf, 'LineHandles');
delete_line([lh.LConn(lh.LConn > 0), lh.RConn(lh.RConn > 0)]);
delete_block(lf);
lcl = blk('LCL filter');
add_block('simulink/Ports & Subsystems/Subsystem', lcl, 'Position', [pos(1)-15 0 pos(3)+15 70]);
buildLCL(lcl, LIB, COMPOSITE);
alignPort(lcl, 'RConn', 1, portPos(blk('PCC V-I'), 'LConn', 1));
% port handles (the mirrored bridge reports stale coordinates in a loaded model)
pl = get_param(lcl, 'PortHandles');
pb = get_param(blk('Two-Level IGBT Bridge'), 'PortHandles');
pv = get_param(blk('PCC V-I'), 'PortHandles');
add_line(m, pb.LConn(2), pl.LConn(1), 'autorouting', 'smart');
add_line(m, pl.RConn(1), pv.LConn(1));
% With the LCL network the default partition method (ROBUST) fails to take the
% first step; MERGE works and keeps the partitioning solver's speed.
set_param(blk('Solver Configuration'), 'PartitionMethod', 'MERGE');
o = portPos(lcl, 'Outport', 1);
add_block('simulink/Signal Routing/Goto', blk('Goto i_inv'), 'GotoTag', 'i_inv', 'ShowName', 'off', ...
    'Position', [o(1)+15 o(2)-8 o(1)+55 o(2)+8]);
li = add_line(m, [o; o(1)+15 o(2)]);

%% ---------------- Current feedback: grid side or inverter side ----------------
ctrl = blk('Controller');
y1 = getfield(portPos(ctrl, 'Inport', 1), {2}); %#ok<GFLD>
xr = getfield(portPos(blk('PCC V-I'), 'RConn', 1), {1}); %#ok<GFLD>
% the FIR column moves 40 px to the right to make room for the Variant Source
for b = {'Period avg I', 'Period avg V'}
    lh = get_param(blk(b{1}), 'LineHandles'); delete_line([lh.Inport lh.Outport]);
    set_param(blk(b{1}), 'Position', get_param(blk(b{1}), 'Position') + [40 0 40 0]);
end
% (still logged as Iabc_pcc / Vabc_pcc; the lines are now too short for labels)
set_param(wireSig(m, blk('I PS-S'), 1, blk('Period avg I'), 1), 'Name', '');
set_param(wireSig(m, blk('V PS-S'), 1, blk('Period avg V'), 1), 'Name', '');
wireSig(m, blk('Period avg V'), 1, ctrl, 2);
vs = blk('Current feedback');
add_block('simulink/Signal Routing/Variant Source', vs, 'Position', [xr-190 y1-40 xr-155 y1+40], ...
    'VariantControls', {'fb_side == 1', 'fb_side == 2'}, 'ShowConditionOnBlock', 'off', 'BlockMirror', 'on', ...
    'NamePlacement', 'alternate');
alignPort(vs, 'Outport', 1, [0 y1]);
add_block('simulink/Signal Routing/From', blk('From i_inv'), 'GotoTag', 'i_inv', 'ShowName', 'off', ...
    'BlockMirror', 'on', 'Position', [xr-45 y1+35 xr y1+55]);
av = get_param(blk('Period avg I'), 'Position');
add_block('simulink/Discrete/Discrete FIR Filter', blk('Period avg I_inv'), 'Coefficients', 'b_avg', ...
    'SampleTime', 'Ts', 'BlockMirror', 'on', 'Position', [av(1) y1+30 av(3) y1+60]);
wireSig(m, blk('From i_inv'), 1, blk('Period avg I_inv'), 1);
wireSig(m, blk('Period avg I'), 1, vs, 1);
wireSig(m, blk('Period avg I_inv'), 1, vs, 2);
wireSig(m, vs, 1, ctrl, 1);

%% ---------------- Row 3: EMS -> SOC limits -> Q comp. -> controller ----------------
y3 = getfield(portPos(ctrl, 'Inport', 3), {2}); %#ok<GFLD>
lim = blk('SOC limits'); ems = blk('EMS commands');
for b = {lim, ems, blk('From SOC')}
    lh = get_param(b{1}, 'LineHandles');
    h = [lh.Inport lh.Outport]; delete_line(h(h > 0));
end
set_param(lim, 'Position', [xr-95 y3-50 xr-20 y3+50]);
set_param(ems, 'Position', [xr+20 y3-47 xr+80 y3-3]);
set_param(blk('From SOC'), 'Position', [xr+30 y3+15 xr+80 y3+35]);
qc = blk('Q comp.');
add_block('simulink/User-Defined Functions/MATLAB Function', qc, 'Position', [xr-195 y3-20 xr-115 y3+20]);
setScript(qc, strjoin({
    'function PQ = fcn(PQ_in, q_comp, fb_side, Q_C)'
    '% Inverter-side current control regulates i1 = i2 + i_C. The controller'
    '% computes Q from the PCC voltage and i1, so the capacitor current adds its'
    '% reactive power Q_C to the PCC: Q* is lowered by Q_C (feedforward, nominal'
    '% voltage). L2 needs no correction: its Q stays inside the measured quantities.'
    '% Grid-side current control needs no correction: it regulates the PCC current.'
    'k = q_comp*(fb_side == 2);'
    'PQ = [PQ_in(1); PQ_in(2) - k*Q_C];'}, nl), {'q_comp', 'fb_side', 'Q_C'});
set_param(qc, 'BlockMirror', 'on');
le = wireSig(m, ems, 1, lim, 1);
wireSig(m, blk('From SOC'), 1, lim, 2);
lb = wireSig(m, lim, 1, qc, 1);
lr = wireSig(m, qc, 1, ctrl, 3);

for s = {'PQ_ems', le; 'PQ_ref', lr; 'Iabc_inv', li}'
    set_param(get_param(s{2}, 'SrcPortHandle'), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', s{1});
end
% short lines: no labels (new lines inherit the names of the deleted ones)
for h = [le lb lr], set_param(h, 'Name', ''); end

%% ---------------- Notes ----------------
for h = find_system(m, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'annotation')'
    delete(get_param(h, 'Object'));
end
bpos = get_param(blk('Battery + DC link'), 'Position');
ppos = get_param(blk('PWM'), 'Position');
gr =get_param(blk('Ground'), 'Position'); gn = portPos(blk('Programmable Grid'), 'LConn', 1);
note(m, [bpos(1) ppos(2)-120], sprintf(['Step 3b - LCL filter (battery, BMS and controller of step 3a)\n' ...
    'L1 = 3 mH, Cf = 10 uF (star, damping resistor Rd), L2 = 2 mH; resonance ~1.3 kHz with the grid inductance.\n' ...
    'L1 + L2 = Lf and R1 + R2 = Rf: at 50 Hz the plant is unchanged, so is the current controller.\n' ...
    'Current feedback (Variant Source, fb_side): 1 = grid-side current at the PCC, 2 = inverter-side current i1.\n' ...
    'Q comp.: with fb_side = 2, Q* is corrected for the reactive power of Cf.  run_03b_lcl.m compares the cases.']));
note(m, [gr(3)+70 gn(2)+25], sprintf('Every Simscape network needs\none Solver Configuration (f(x)=0)'));
note(m, [xr+40 y1-10], sprintf(['Oversampling ADC: average over one carrier\n' ...
    'period (FIR, b_avg) removes the switching ripple.']));
note(m, [xr+110 y3-35], sprintf(['EMS -> SOC limits (BMS derating) -> Q comp.\n' ...
    '(Q* - Q_C: correction for Cf, only with fb_side = 2)']));

set_param(m, 'ZoomFactor', 'FitSystem');
save_system(m, f);
fprintf('Saved %s\n', f);
end

%% ======================= subsystems =======================
function buildLCL(s, LIB, COMPOSITE)
% ~1 -> i1 sensor -> L1 -> node -> L2 -> ~2;  node -> Rd + Cf -> star point
% The star point is grounded: a floating star (a node with only capacitors)
% stalls the partitioning solver. The DC side has no ground, so no common-mode
% current can flow through this connection.
delete_line(s, 'In1/1', 'Out1/1');
delete_block([s '/In1']);
b = @(n) [s '/' n];
P = @(blk, side, k) portPos(b(blk), side, k);
cp = {'port_option', COMPOSITE};
add_block(LIB.port, b('~1'), 'Side', 'Left', 'Position', [20 93 50 107]);
add_block(LIB.is, b('i1 sensor'), 'Position', [90 80 130 120], cp{:});
add_block(LIB.rlc, b('L1'), 'Position', [180 0 240 40], cp{:}, 'component_structure', '4', 'R', 'R1', 'L', 'L1');
add_block(LIB.rlc, b('L2'), 'Position', [330 0 390 40], cp{:}, 'component_structure', '4', 'R', 'R2', 'L', 'L2');
% series RC (structure 5) turned to vertical: Rd + Cf per phase
add_block(LIB.rlc, b('Rd + Cf'), 'Position', [265 170 305 230], cp{:}, 'Orientation', 'down', ...
    'component_structure', '5', 'R', 'Rd', 'C', 'Cf');
add_block(LIB.fn, b('star point'), 'Position', [270 280 300 310], cp{:});
add_block(LIB.port, b('~2'), 'Side', 'Right', 'Position', [450 0 480 14]);
add_block(LIB.ps2s, b('I PS-S'), 'Position', [330 20 360 40], 'VectorFormat', '1-D array');
set_param(b('Out1'), 'Name', 'i_inv', 'Position', [400 23 430 37]);

alignPort(b('~1'), 'RConn', 1, P('i1 sensor', 'LConn', 1));
yr = getfield(P('i1 sensor', 'RConn', 2), {2}); %#ok<GFLD>
alignPort(b('L1'), 'LConn', 1, [0 yr]);
alignPort(b('L2'), 'LConn', 1, [0 yr]);
alignPort(b('~2'), 'RConn', 1, [0 yr]);
c = P('Rd + Cf', 'LConn', 1); pc = get_param(b('Rd + Cf'), 'Position');
set_param(b('Rd + Cf'), 'Position', pc + [285-c(1) 0 285-c(1) 0]);
c = P('Rd + Cf', 'RConn', 1); n = P('star point', 'LConn', 1); pn = get_param(b('star point'), 'Position');
set_param(b('star point'), 'Position', pn + [c(1)-n(1) 0 c(1)-n(1) 0]);

add_line(s, [P('~1', 'RConn', 1); P('i1 sensor', 'LConn', 1)]);
add_line(s, [P('i1 sensor', 'RConn', 2); P('L1', 'LConn', 1)]);
add_line(s, [P('L1', 'RConn', 1); P('L2', 'LConn', 1)]);
add_line(s, [P('L2', 'RConn', 1); P('~2', 'RConn', 1)]);
c = P('Rd + Cf', 'LConn', 1); add_line(s, [c(1) yr; c]);
add_line(s, [P('Rd + Cf', 'RConn', 1); P('star point', 'LConn', 1)]);
io = P('i1 sensor', 'RConn', 1); ip = P('I PS-S', 'LConn', 1);
add_line(s, [io; io(1)+15 io(2); io(1)+15 ip(2); ip]);
add_line(s, 'I PS-S/1', 'i_inv/1');
set_param(s, 'ContentPreviewEnabled', 'off', 'MaskDisplay', sprintf(['disp(''LCL filter'');' ...
    'port_label(''output'',1,''i1'');port_label(''lconn'',1,''~1'');port_label(''rconn'',1,''~2'');']));
end

%% ======================= helpers =======================
function setScript(b, code, params)
ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', b);
ch.Script = code;
for k = 1:numel(params)
    dobj = ch.find('-isa', 'Stateflow.Data', 'Name', params{k});
    dobj.Scope = 'Parameter';
end
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
