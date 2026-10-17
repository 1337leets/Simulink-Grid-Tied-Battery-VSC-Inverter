function build_03a_battery()
%BUILD_03A_BATTERY Step 3a: battery + DC-link capacitor instead of the ideal DC source.
% Starts from the stage 2c model (Save As) and changes only what step 3a needs:
%   - "DC link" (ideal source) -> "Battery + DC link" (battery, cable, capacitor),
%     same + / - / Vdc ports plus a SOC output
%   - "SOC limits" (BMS derating) between the EMS commands and the controller
%   - EMS day profile: only new values in params_03a (same lookup-table block)
% Bridge, gate interface, PWM, measurements and controller stay as in stage 2c.

here = fileparts(mfilename('fullpath'));
d2 = fullfile(here, '..', '02_current_control');
m = 'vsi_03a_battery'; src = 'vsi_02c_switching';
nl = newline;

f2 = fullfile(d2, [src '.slx']);
if ~isfile(f2), run(fullfile(d2, 'build_02c_switching.m')); end
for s = {m, src}, if bdIsLoaded(s{1}), close_system(s{1}, 0); end, end
f = fullfile(here, [m '.slx']);
if isfile(f), delete(f); end
load_system(f2);
save_system(src, f);                         % Save As: the loaded model is now vsi_03a_battery
cb = sprintf('run(fullfile(fileparts(get_param(''%s'',''FileName'')), ''params_03a_battery.m''));', m);
set_param(m, 'PostLoadFcn', cb, 'InitFcn', cb);
run(fullfile(here, 'params_03a_battery.m'));
cellfun(@load_system, {'ee_lib', 'fl_lib', 'nesl_utility'});

LIB.bat  = ['ee_lib/Sources/Battery' nl '(Table-Based)'];
LIB.C    = 'fl_lib/Electrical/Electrical Elements/Capacitor';
LIB.L    = 'fl_lib/Electrical/Electrical Elements/Inductor';
LIB.is   = 'fl_lib/Electrical/Electrical Sensors/Current Sensor';
LIB.vs   = 'fl_lib/Electrical/Electrical Sensors/Voltage Sensor';
LIB.ps2s = ['nesl_utility/PS-Simulink' nl 'Converter'];
LIB.port = 'nesl_utility/Connection Port';
blk = @(name) [m '/' name];

%% ---------------- DC link -> Battery + DC link ----------------
br = blk('Two-Level IGBT Bridge');
bp = portPos(br, 'RConn', 1);
dcl = blk('DC link');
pos = get_param(dcl, 'Position');
lh = get_param(dcl, 'LineHandles');
delete_line([lh.Outport(lh.Outport > 0), lh.RConn(lh.RConn > 0)]);
delete_block({dcl, blk('Goto Vdc')});
bat = blk('Battery + DC link');
add_block('simulink/Ports & Subsystems/Subsystem', bat, 'Position', [pos(1)-20 0 pos(3) 150]);
buildBattery(bat, LIB);
alignPort(bat, 'RConn', 1, bp);
% connect by port handles: the bridge icon of a loaded (not freshly added) model
% reports its port x before rendering, so coordinates would miss the port
pb = get_param(bat, 'PortHandles'); pr = get_param(br, 'PortHandles');
add_line(m, pb.RConn(1), pr.RConn(1), 'autorouting', 'smart');
add_line(m, pb.RConn(2), pr.RConn(2), 'autorouting', 'smart');

% Vdc and SOC go out through Goto tags
tags = {'Vdc', 'SOC'}; lo = zeros(1, 2);
for k = 1:2
    o = portPos(bat, 'Outport', k);
    add_block('simulink/Signal Routing/Goto', blk(['Goto ' tags{k}]), 'GotoTag', tags{k}, 'ShowName', 'off', ...
        'Position', [o(1)+15 o(2)-8 o(1)+50 o(2)+8]);
    lo(k) = add_line(m, [o; o(1)+15 o(2)]);
end

%% ---------------- SOC limits between the EMS and the controller ----------------
ems = blk('EMS commands'); ctrl = blk('Controller');
lh = get_param(ems, 'LineHandles'); delete_line(lh.Outport(1));
y3 = getfield(portPos(ctrl, 'Inport', 3), {2}); %#ok<GFLD>
xr = getfield(portPos(blk('PCC V-I'), 'RConn', 1), {1}); %#ok<GFLD>
lim = blk('SOC limits');
add_block('simulink/Ports & Subsystems/Subsystem', lim, 'Position', [xr-175 y3-50 xr-100 y3+50]);
buildLimits(lim);
set_param(lim, 'BlockMirror', 'on');
pe = get_param(ems, 'Position');
set_param(ems, 'Position', [pe(1) y3-25-22 pe(3) y3-25+22]);
le = wireSig(m, ems, 1, lim, 1);            % logged as PQ_ems (no label: the line is short)
set_param(le, 'Name', '');                  % (the new line inherits the old name PQ_ref)
lr = wireSig(m, lim, 1, ctrl, 3); set_param(lr, 'Name', 'PQ_ref');
ps = portPos(lim, 'Inport', 2);
add_block('simulink/Signal Routing/From', blk('From SOC'), 'GotoTag', 'SOC', 'ShowName', 'off', ...
    'BlockMirror', 'on', 'Position', [xr-45 ps(2)-10 xr ps(2)+10]);
wireSig(m, blk('From SOC'), 1, lim, 2);

for s = {'Vdc', lo(1); 'SOC', lo(2); 'PQ_ems', le; 'PQ_ref', lr}'
    set_param(get_param(s{2}, 'SrcPortHandle'), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', s{1});
end

%% ---------------- Notes ----------------
for h = find_system(m, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'annotation')'
    delete(get_param(h, 'Object'));
end
bpos = get_param(bat, 'Position');
ppos = get_param(blk('PWM'), 'Position');
gr = get_param(blk('Ground'), 'Position'); gn = portPos(blk('Programmable Grid'), 'LConn', 1);
vI = portPos(blk('Period avg I'), 'Inport', 1);
note(m, [bpos(1) ppos(2)-120], sprintf(['Step 3a - Battery on the DC link (switching model and controller of stage 2c)\n' ...
    'Battery (Table-Based): 224 LFP cells in series, 28 Ah (20 kWh), with a cable inductance and a DC-link capacitor.\n' ...
    'SOC limits (BMS): P* derated to zero between 85-90 %% SOC (charge) and 15-10 %% (discharge); Q* is not limited.\n' ...
    'EMS day profile 06:00-24:00, time-compressed: 1 h = T_h = 50 ms, battery capacity scaled by the same factor.']));
note(m, [gr(3)+70 gn(2)+25], sprintf('Every Simscape network needs\none Solver Configuration (f(x)=0)'));
note(m, [bpos(1) bpos(4)+15], sprintf('Same + / - / Vdc ports as\nthe ideal source of stage 2c,\nplus the SOC from the BMS.'));
note(m, [xr+40 vI(2)-10], sprintf(['Oversampling ADC: average over one carrier\n' ...
    'period (FIR, b_avg) removes the switching ripple.']));
note(m, [xr+40 y3-35], sprintf(['EMS day profile (lookup tables, params_03a).\n' ...
    'SOC limits: BMS derating curves (lookup tables)\nclamp P* with a Saturation Dynamic block.']));

set_param(m, 'ZoomFactor', 'FitSystem');
save_system(m, f);
fprintf('Saved %s\n', f);
end

%% ======================= subsystems =======================
function buildBattery(s, LIB)
% Battery -> current sensor -> cable L -> DC-link capacitor || voltage sensor -> + / -
% BMS: SOC by coulomb counting of the measured battery current.
% Signal lane along the top (I_bat -> SOC), - rail along the bottom.
delete_line(s, 'In1/1', 'Out1/1');
delete_block([s '/In1']);
b = @(n) [s '/' n];
P = @(blk, side, k) portPos(b(blk), side, k);

add_block(LIB.bat, b('Battery'), 'Position', [40 140 80 200], ...
    'T_dependence', 'simscape.enum.tablebattery.temperature_dependence.no', ...
    'SOC_vec', 'SOC_bp', 'V0_vec', 'V0_bat', 'R0_vec', 'R0_bat*ones(size(SOC_bp))', 'AH', 'AH_sim', ...
    'stateOfCharge_specify', 'on', 'stateOfCharge_priority', 'High', 'stateOfCharge', 'SOC0');
% Current sensor: + left, I upper right, - lower right
add_block(LIB.is, b('I_bat sensor'), 'Position', [120 70 160 110]);
add_block(LIB.L, b('Cable L'), 'Position', [200 85 240 115], 'l', 'L_cable', 'r', '0', ...
    'i_L_specify', 'on', 'i_L', '0');
add_block(LIB.C, b('C_dc'), 'Position', [285 150 315 190], 'Orientation', 'down', 'c', 'C_dc', ...
    'vc_specify', 'on', 'vc_priority', 'High', 'vc', 'Vdc');
% Voltage sensor: + top, V bottom left, - bottom right
add_block(LIB.vs, b('Vdc sensor'), 'Position', [380 150 420 210]);
add_block(LIB.ps2s, b('I PS-S'), 'Position', [270 30 300 50]);
% BMS: SOC(t) = SOC0 - integral(i_bat dt)/(3600*Ah)   (i_bat > 0: discharge)
add_block('simulink/Math Operations/Gain', b('Ah to SOC'), 'Position', [340 25 410 55], 'Gain', '-1/(3600*AH_sim)');
add_block('simulink/Discrete/Discrete-Time Integrator', b('BMS: coulomb counting'), 'Position', [450 20 490 60], ...
    'InitialCondition', 'SOC0', 'SampleTime', 'Ts');
add_block(LIB.ps2s, b('Vdc PS-S'), 'Position', [460 260 490 280]);
set_param(b('Out1'), 'Name', 'Vdc', 'Position', [525 263 555 277]);
add_block('simulink/Sinks/Out1', b('SOC'), 'Position', [540 33 570 47]);
add_block(LIB.port, b('+'), 'Side', 'Right', 'Position', [620 0 650 14]);
add_block(LIB.port, b('-'), 'Side', 'Right', 'Position', [620 0 650 14]);

% snap the inline elements onto the rails
alignPort(b('I_bat sensor'), 'LConn', 1, [0 90]);
alignPort(b('Cable L'), 'LConn', 1, P('I_bat sensor', 'RConn', 2));
yr = getfield(P('Cable L', 'RConn', 1), {2}); %#ok<GFLD>
alignPort(b('+'), 'RConn', 1, [0 yr]);
alignPort(b('-'), 'RConn', 1, [0 240]);
c = P('C_dc', 'LConn', 1); pc = get_param(b('C_dc'), 'Position');
set_param(b('C_dc'), 'Position', pc + [300-c(1) 0 300-c(1) 0]);
alignPort(b('Vdc PS-S'), 'LConn', 1, [0 270]);
alignPort(b('Vdc'), 'Inport', 1, P('Vdc PS-S', 'Outport', 1));

% + rail: battery -> sensor -> cable -> capacitor / voltage sensor -> port
bt = P('Battery', 'LConn', 1); si = P('I_bat sensor', 'LConn', 1);
add_line(s, [bt; bt(1) si(2); si]);
add_line(s, [P('I_bat sensor', 'RConn', 2); P('Cable L', 'LConn', 1)]);
lr = P('Cable L', 'RConn', 1); ep = P('+', 'RConn', 1);
add_line(s, [lr; ep]);
c = P('C_dc', 'LConn', 1); add_line(s, [c(1) yr; c]);
v = P('Vdc sensor', 'LConn', 1); add_line(s, [v(1) yr; v]);
% - rail: battery along the bottom to the junction, voltage sensor straight to the port
yb = 300; xj = 590;
bm = P('Battery', 'RConn', 1); em = P('-', 'RConn', 1);
nn = P('Vdc sensor', 'RConn', 2);
add_line(s, [nn; nn(1) em(2); em]);
add_line(s, [bm; bm(1) yb; xj yb; xj em(2)]);
cm = P('C_dc', 'RConn', 1); add_line(s, [cm; cm(1) yb]);
% signals: I_bat -> SOC along the top, Vdc below the - port
io = P('I_bat sensor', 'RConn', 1); ip = P('I PS-S', 'LConn', 1);
add_line(s, [io; io(1)+15 io(2); io(1)+15 ip(2); ip]);
alignPort(b('Ah to SOC'), 'Inport', 1, P('I PS-S', 'Outport', 1));
alignPort(b('BMS: coulomb counting'), 'Inport', 1, P('I PS-S', 'Outport', 1));
alignPort(b('SOC'), 'Inport', 1, P('BMS: coulomb counting', 'Outport', 1));
li = add_line(s, 'I PS-S/1', 'Ah to SOC/1');
add_line(s, 'Ah to SOC/1', 'BMS: coulomb counting/1');
add_line(s, 'BMS: coulomb counting/1', 'SOC/1');
vv = P('Vdc sensor', 'RConn', 1); vp = P('Vdc PS-S', 'LConn', 1);
add_line(s, [vv; vv(1) vp(2); vp]);
add_line(s, 'Vdc PS-S/1', 'Vdc/1');
set_param(get_param(li, 'SrcPortHandle'), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'I_bat');
set_param(li, 'Name', 'I_bat');

set_param(s, 'ContentPreviewEnabled', 'off', 'MaskDisplay', sprintf(['disp(''Battery\\n+ DC link'');' ...
    'port_label(''output'',1,''Vdc'');port_label(''output'',2,''SOC'');port_label(''rconn'',1,''+'');port_label(''rconn'',2,''-'');']));
end

function buildLimits(s)
% BMS derating: allowed discharge power P_dis(SOC) >= P* >= allowed charge power P_ch(SOC).
% Q* passes unchanged. The SOC inport is duplicated (same port, second block)
% so that each derating curve gets its own short wire.
delete_line(s, 'In1/1', 'Out1/1');
b = @(n) [s '/' n];
set_param(b('In1'), 'Name', 'PQ_ems', 'Position', [70 103 100 117]);
set_param(b('Out1'), 'Name', 'PQ_ref', 'Position', [500 113 530 127]);
add_block('simulink/Sources/In1', b('SOC'), 'Port', '2', 'Position', [130 23 160 37]);
add_block(b('SOC'), b('SOC '), 'CopyOption', 'duplicate', 'Position', [130 183 160 197]);
add_block('simulink/Signal Routing/Demux', b('P*, Q*'), 'Outputs', '2', 'Position', [130 90 135 150]);
add_block('simulink/Lookup Tables/1-D Lookup Table', b('discharge limit'), 'Position', [200 10 260 50], ...
    'BreakpointsForDimension1', 'SOC_dis', 'Table', 'P_dis', 'ExtrapMethod', 'Clip');
add_block('simulink/Lookup Tables/1-D Lookup Table', b('charge limit'), 'Position', [200 170 260 210], ...
    'BreakpointsForDimension1', 'SOC_ch', 'Table', 'P_ch', 'ExtrapMethod', 'Clip');
add_block('simulink/Discontinuities/Saturation Dynamic', b('clamp P*'), 'Position', [320 55 370 145]);
add_block('simulink/Signal Routing/Mux', b('Mux'), 'Inputs', '2', 'Position', [430 80 435 160]);
alignPort(b('P*, Q*'), 'Outport', 1, portPos(b('clamp P*'), 'Inport', 2));
alignPort(b('PQ_ems'), 'Outport', 1, portPos(b('P*, Q*'), 'Inport', 1));
alignPort(b('Mux'), 'Inport', 1, portPos(b('clamp P*'), 'Outport', 1));
alignPort(b('PQ_ref'), 'Inport', 1, portPos(b('Mux'), 'Outport', 1));
add_line(s, 'PQ_ems/1', 'P*, Q*/1');
add_line(s, 'SOC/1', 'discharge limit/1');
add_line(s, 'SOC /1', 'charge limit/1');
add_line(s, 'P*, Q*/1', 'clamp P*/2');
l = add_line(s, 'discharge limit/1', 'clamp P*/1', 'autorouting', 'smart'); set_param(l, 'Name', 'max');
l = add_line(s, 'charge limit/1', 'clamp P*/3', 'autorouting', 'smart'); set_param(l, 'Name', 'min');
add_line(s, 'clamp P*/1', 'Mux/1');
q = portPos(b('P*, Q*'), 'Outport', 2); mq = portPos(b('Mux'), 'Inport', 2);
yq = 240;
add_line(s, [q; q(1)+15 q(2); q(1)+15 yq; mq(1)-20 yq; mq(1)-20 mq(2); mq]);
add_line(s, 'Mux/1', 'PQ_ref/1');
set_param(s, 'ContentPreviewEnabled', 'off', 'MaskDisplay', sprintf(['disp(''SOC\\nlimits'');' ...
    'port_label(''input'',1,''P*,Q*'');port_label(''input'',2,''SOC'');']));
end

%% ======================= helpers =======================
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
