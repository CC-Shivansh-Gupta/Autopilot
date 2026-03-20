%% build_chap3_simulink.m  (v3 - correct initial conditions)
% Builds chap3_sim.slx: Constants -> Mux -> EOM -> Demux -> Animation
%
% KEY FIX: Initial conditions set to ALL ZEROS so that when
%          all forces/moments = 0, aircraft stays perfectly still.
%
% Run: >> build_chap3_simulink
% Then Ctrl+T in the opened model.

clear; clc;
F4_Phantom_params;   % loads mass, I_xx, I_yy, I_zz, I_xz, Va_trim, theta_trim

model = 'chap3_sim';
if bdIsLoaded(model), close_system(model,0); end
if exist([model '.slx'],'file'), delete([model '.slx']); end
new_system(model); open_system(model);

%% ---- Layout ----
BW=130; BH=40; GAP=12;
rowY = @(i) 40 + (i-1)*(BH+GAP);

%% ============================================================
%  1. FORCE/MOMENT CONSTANT BLOCKS
% ============================================================
labels = {'fx_lb','fy_lb','fz_lb','l_lbft','m_lbft','n_lbft'};
for i = 1:6
    add_block('simulink/Sources/Constant', [model '/' labels{i}], ...
        'Value','0', ...
        'Position',[30, rowY(i), 30+BW, rowY(i)+BH]);
    set_param([model '/' labels{i}],'OutDataTypeStr','double');
end

%% ============================================================
%  2. MUX 6->1
% ============================================================
xM=30+BW+25; yMt=rowY(1); yMb=rowY(6)+BH;
add_block('simulink/Signal Routing/Mux',[model '/Mux_FM'], ...
    'Inputs','6','Position',[xM,yMt,xM+25,yMb]);
for i=1:6
    add_line(model,[labels{i} '/1'],['Mux_FM/' num2str(i)],'autorouting','on');
end

%% ============================================================
%  3. EOM S-FUNCTION
%  IMPORTANT: Initial conditions ALL ZERO
%  -> With zero forces, zero velocity, aircraft stays still
%  -> Each force test then shows clean isolated response
% ============================================================
xE=xM+60; yE=rowY(2);

% Parameters: mass Jx Jy Jz Jxz | pn0 pe0 pd0 | u0 v0 w0 | phi0 theta0 psi0 | p0 q0 r0
% pd0=-300 so aircraft starts 300 ft above ground (visible in animation)
% ALL VELOCITIES and ANGLES = 0 -> stationary aircraft
pstr = sprintf('%g,%g,%g,%g,%g, 0,0,-300, 0,0,0, 0,0,0, 0,0,0', ...
               mass, I_xx, I_yy, I_zz, I_xz);

add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    [model '/EOM'], ...
    'FunctionName','chap3_eom', ...
    'Parameters',pstr, ...
    'Position',[xE,yE,xE+170,yE+100]);
add_line(model,'Mux_FM/1','EOM/1','autorouting','on');

%% ============================================================
%  4. DEMUX 12 states, re-MUX 6 animation states
% ============================================================
xS=xE+210;
add_block('simulink/Signal Routing/Demux',[model '/Demux12'], ...
    'Outputs','12','Position',[xS,yE,xS+20,yE+100]);
add_line(model,'EOM/1','Demux12/1','autorouting','on');

xM2=xS+60;
add_block('simulink/Signal Routing/Mux',[model '/Mux_anim'], ...
    'Inputs','6','Position',[xM2,yE+5,xM2+20,yE+95]);
anim_idx = [1,2,3,7,8,9];   % pn pe pd phi theta psi
for k=1:6
    add_line(model,['Demux12/' num2str(anim_idx(k))], ...
                   ['Mux_anim/' num2str(k)],'autorouting','on');
end

%% ============================================================
%  5. ANIMATION
% ============================================================
xA=xM2+60;
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    [model '/Animation'], ...
    'FunctionName','chap2_animation','Parameters','1', ...
    'Position',[xA,yE+5,xA+160,yE+95]);
add_line(model,'Mux_anim/1','Animation/1','autorouting','on');

%% ============================================================
%  6. SCOPE
% ============================================================
add_block('simulink/Sinks/Scope',[model '/Scope'], ...
    'Position',[xS,yE+130,xS+50,yE+170]);
add_line(model,'EOM/1','Scope/1','autorouting','on');

%% ============================================================
%  7. SOLVER
% ============================================================
set_param(model,'SolverType','Variable-step','Solver','ode45', ...
          'StopTime','20','MaxStep','0.05');

save_system(model,[model '.slx']);

%% ============================================================
%  Print exact test values
% ============================================================
g = 32.174;
fx_trim =  mass * g * sin(theta_trim);
fz_trim = -mass * g * cos(theta_trim);

fprintf('\n=== chap3_sim.slx ready — Press Ctrl+T to run ===\n\n');
fprintf('INITIAL CONDITIONS (all zeros = aircraft stationary at 300 ft):\n');
fprintf('  u=v=w=0, phi=theta=psi=0, p=q=r=0\n');
fprintf('  With forces=0: aircraft does NOT move. This is correct.\n\n');

fprintf('SECTION 2(ii) — AXIS VERIFICATION TESTS:\n');
fprintf('Double-click ONE block, enter the value, run, watch animation.\n');
fprintf('Reset to 0 between tests.\n\n');
fprintf('  Block      Value [units]    Expected animation\n');
fprintf('  %-12s %+10.0f lb      Forward acceleration (nose stays level)\n','fx_lb', 5000);
fprintf('  %-12s %+10.0f lb      Sideslip to right\n',                      'fy_lb', 800);
fprintf('  %-12s %+10.0f lb      Pushed down, altitude drops\n',            'fz_lb', 800);
fprintf('  %-12s %+10.0f lb.ft   Right roll, right wing dips\n',            'l_lbft',8000);
fprintf('  %-12s %+10.0f lb.ft   Pitch up, nose rises\n',                   'm_lbft',8000);
fprintf('  %-12s %+10.0f lb.ft   Yaw right, nose swings right\n',           'n_lbft',8000);

fprintf('\nTRIM HOVER TEST (aircraft stays perfectly still):\n');
fprintf('  fx_lb  = %+.1f lb    (thrust balances gravity component)\n', fx_trim);
fprintf('  fz_lb  = %+.1f lb    (lift balances weight)\n', fz_trim);
fprintf('  u0 must also be changed to Va_trim=%g in EOM params\n', Va_trim);
fprintf('  -> aircraft holds position, no motion\n\n');

fprintf('SECTION 2(iii) — Jxz COUPLING TEST:\n');
fprintf('  1. Double-click EOM block -> edit Parameters\n');
fprintf('     Change param 5 (Jxz=%g) to 0\n', I_xz);
fprintf('     Set l_lbft=5000, n_lbft=5000, run -> p and r grow independently\n');
fprintf('  2. Restore Jxz=%g, same inputs -> p and r are different (coupled)\n\n', I_xz);
