%% build_chap3_simulink.m
% =========================================================
% Builds chap3_sim.slx — 6-DOF EOM with animation
% AE700 — IIT Bombay | F-4 Phantom
%
% Simulink layout:
%   6 Constant force/moment inputs
%   -> Mux -> chap3_eom (S-Function) -> Demux -> animation
%
% Initial conditions: all zero (stationary aircraft at 300 ft)
% -> With zero forces, aircraft does NOT move (correct baseline).
%
% Run once: >> build_chap3_simulink
% Then Ctrl+T to run.
% =========================================================

clear; clc;
F4_params;    % loads P and workspace scalars (mass, I_xx, etc.)

model='chap3_sim';
if bdIsLoaded(model), close_system(model,0); end
if exist([model '.slx'],'file'), delete([model '.slx']); end
new_system(model); open_system(model);

BW=130; BH=40; GAP=12;
row=@(i) 40+(i-1)*(BH+GAP);

%% Force/moment Constant blocks
fm_labels={'fx_lb','fy_lb','fz_lb','l_lbft','m_lbft','n_lbft'};
for i=1:6
    add_block('simulink/Sources/Constant',[model '/' fm_labels{i}],...
        'Value','0','Position',[30,row(i),30+BW,row(i)+BH]);
    set_param([model '/' fm_labels{i}],'OutDataTypeStr','double');
end

%% Mux 6->1
xM=30+BW+20; yMt=row(1); yMb=row(6)+BH;
add_block('simulink/Signal Routing/Mux',[model '/Mux_FM'],...
    'Inputs','6','Position',[xM,yMt,xM+25,yMb]);
for i=1:6
    add_line(model,[fm_labels{i} '/1'],['Mux_FM/' num2str(i)],'autorouting','on');
end

%% EOM S-Function
% Parameters: mass Jx Jy Jz Jxz | pn0 pe0 pd0 | u0 v0 w0 | phi0 theta0 psi0 | p0 q0 r0
% ALL initial velocities/angles ZERO -> clean step response tests
xE=xM+65; yE=row(2);
pstr=sprintf('%g,%g,%g,%g,%g, 0,0,-300, 0,0,0, 0,0,0, 0,0,0',...
             mass, I_xx, I_yy, I_zz, I_xz);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',...
    [model '/EOM'],'FunctionName','chap3_eom','Parameters',pstr,...
    'Position',[xE,yE,xE+170,yE+100]);
add_line(model,'Mux_FM/1','EOM/1','autorouting','on');

%% Demux 12 states -> animation mux
xS=xE+210;
add_block('simulink/Signal Routing/Demux',[model '/Demux12'],...
    'Outputs','12','Position',[xS,yE,xS+20,yE+100]);
add_line(model,'EOM/1','Demux12/1','autorouting','on');

% Select animation states: pn(1) pe(2) pd(3) phi(7) theta(8) psi(9)
xM2=xS+55;
add_block('simulink/Signal Routing/Mux',[model '/Mux_anim'],...
    'Inputs','6','Position',[xM2,yE+5,xM2+20,yE+95]);
anim_idx=[1,2,3,7,8,9];
for k=1:6
    add_line(model,['Demux12/' num2str(anim_idx(k))],['Mux_anim/' num2str(k)],'autorouting','on');
end

%% Animation
xA=xM2+60;
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',...
    [model '/Animation'],'FunctionName','chap2_animation','Parameters','1',...
    'Position',[xA,yE+5,xA+160,yE+95]);
add_line(model,'Mux_anim/1','Animation/1','autorouting','on');

%% Scope
add_block('simulink/Sinks/Scope',[model '/Scope'],...
    'Position',[xS,yE+130,xS+50,yE+170]);
add_line(model,'EOM/1','Scope/1','autorouting','on');

%% Solver
set_param(model,'SolverType','Variable-step','Solver','ode45',...
          'StopTime','20','MaxStep','0.05');
save_system(model,[model '.slx']);

g=P.g;
fx_trim =  mass * g * sin(theta_trim);
fz_trim = -mass * g * cos(theta_trim);

fprintf('chap3_sim.slx built successfully.\n\n');
fprintf('INDIVIDUAL AXIS TESTS (enter ONE value, others = 0, run, observe):\n');
fprintf('  Block     Value        Expected animation\n');
fprintf('  %-10s %+8.0f lb   Forward acceleration\n','fx_lb',5000);
fprintf('  %-10s %+8.0f lb   Sideslip right\n','fy_lb',800);
fprintf('  %-10s %+8.0f lb   Altitude drops\n','fz_lb',800);
fprintf('  %-10s %+8.0f lb.ft  Right roll\n','l_lbft',8000);
fprintf('  %-10s %+8.0f lb.ft  Nose pitches up\n','m_lbft',8000);
fprintf('  %-10s %+8.0f lb.ft  Yaw right\n','n_lbft',8000);
fprintf('\nTRIM HOVER (aircraft stays still):\n');
fprintf('  fx_lb=%.1f lb  fz_lb=%.1f lb  (and u0=%.0f ft/s in EOM params)\n',...
        fx_trim, fz_trim, Va_trim);
fprintf('\nJxz COUPLING TEST:\n');
fprintf('  1. Edit EOM params: change Jxz=%.0f -> 0, set l=3000, n=3000 -> p,r independent\n',I_xz);
fprintf('  2. Restore Jxz=%.0f, same inputs -> p,r are different (coupled)\n',I_xz);
