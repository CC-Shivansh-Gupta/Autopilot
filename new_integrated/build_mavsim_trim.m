%% build_mavsim_trim.m
% Builds mavsim_trim.slx: the trim Simulink model required by Section 4(ii).
% Structure follows Figure F.1 from Appendix F of Beard & McLain.
%
% ARCHITECTURE:
%   4 Inports  : [de, da, dr, dt]          <- control inputs
%   EOM block  : chap3_eom (12 states)
%   FM  block  : forces_moments_sfunc
%   3 Outports : [Va, alpha, beta]          <- trim outputs
%
% The Simulink trim command then searches for [de,da,dr,dt] such that
%   xdot = 0  (or xdot = xdot_star for climbing/turning trim)
%   Va   = Va_star  (constrained output)
%   beta = 0        (coordinated flight)
%
% Run: >> build_mavsim_trim
% Then use: >> run_linmod_trim  (separate script)

clear; clc;
F4_chap4_params;   % loads struct P

model = 'mavsim_trim';
if bdIsLoaded(model), close_system(model,0); end
if exist([model '.slx'],'file'), delete([model '.slx']); end
new_system(model);
open_system(model);

%% ---- Parameters string for EOM block ----
% EOM dialog: mass Jx Jy Jz Jxz | pn0 pe0 pd0 | u0 v0 w0 | phi0 theta0 psi0 | p0 q0 r0
% Initial condition: trimmed flight at Va_trim, alpha_trim, 1000 ft altitude
u0 = P.Va_trim * cos(P.alpha_trim);
w0 = P.Va_trim * sin(P.alpha_trim);
eom_pstr = sprintf('%g,%g,%g,%g,%g, 0,0,-1000, %g,0,%g, 0,%g,0, 0,0,0', ...
    P.mass, P.Jx, P.Jy, P.Jz, P.Jxz, u0, w0, P.alpha_trim);

%% ---- Block positions ----
x0=30; BW=140; BH=50; GAP=20;

% Column x-positions
xIn   = x0;
xFM   = xIn  + BW + 60;
xEOM  = xFM  + BW + 60;
xDmx  = xEOM + BW + 60;
xOut  = xDmx + BW + 60;

% Row y-positions
yMid = 200;

%% ---- 4 Input ports (de, da, dr, dt) ----
in_labels = {'de','da','dr','dt'};
for k=1:4
    yy = yMid + (k-2.5)*70;
    add_block('simulink/Sources/In1', [model '/in_' in_labels{k}], ...
        'Port',num2str(k), ...
        'Position',[xIn, yy, xIn+BW, yy+BH]);
    set_param([model '/in_' in_labels{k}],'Name',in_labels{k});
end

%% ---- Mux 4->1 for controls ----
xMC = xIn+BW+20; yMC = yMid-40;
add_block('simulink/Signal Routing/Mux',[model '/Mux_ctrl'], ...
    'Inputs','4','Position',[xMC,yMC,xMC+20,yMC+5*70]);
for k=1:4
    add_line(model,[in_labels{k} '/1'],['Mux_ctrl/' num2str(k)],'autorouting','on');
end

%% ---- Forces & Moments S-function ----
% We embed forces_moments inline as a Matlab Function block to avoid
% needing a separate S-function wrapper. The block takes [x(12); ctrl(4); wind(6)]
% and outputs [fx;fy;fz;l;m;n;Va;alpha;beta;wn;we;wd].
%
% For linmod compatibility we use a MATLAB Function block.

xFM2 = xMC+40;
add_block('simulink/User-Defined Functions/MATLAB Function',[model '/ForceMoments'], ...
    'Position',[xFM2,yMid-25,xFM2+BW,yMid+25]);

% Set the MATLAB function code via script (MATLABFunction block)
% We'll create a helper function file instead and use S-function
% Actually for simplicity let's use a Level-2 S-function wrapper
% Remove the MATLAB Function block and use the fm_sfunc approach
delete_block([model '/ForceMoments']);

% Use a Fcn approach: create fm_trim_sfunc.m wrapper
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    [model '/ForceMoments'], ...
    'FunctionName','fm_trim_sfunc', ...
    'Parameters','P', ...
    'Position',[xFM2,yMid-30,xFM2+BW,yMid+30]);

%% ---- EOM S-function ----
xE2 = xFM2+BW+40;
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    [model '/EOM'], ...
    'FunctionName','chap3_eom', ...
    'Parameters',eom_pstr, ...
    'Position',[xE2,yMid-40,xE2+BW,yMid+40]);

%% ---- Connect FM -> EOM ----
% FM outputs [fx;fy;fz;l;m;n;Va;alpha;beta;wn;we;wd] (12x1)
% EOM inputs [fx;fy;fz;l;m;n] (6x1) -> need first 6 elements
% EOM outputs state x (12x1)
% FM needs state x (12x1) + controls (4x1) + wind (6x1)

% Mux for FM input: [x(12); ctrl(4); wind(6)] = 22x1
% Wind = constant zeros
xWind = xFM2-80; yWind = yMid+80;
add_block('simulink/Sources/Constant',[model '/Wind'], ...
    'Value','zeros(6,1)','OutDataTypeStr','double', ...
    'Position',[xWind,yWind,xWind+BW,yWind+BH]);

% Mux: state(12) + ctrl(4) + wind(6) -> 22
xMux2 = xFM2-40; yMux2 = yMid-60;
add_block('simulink/Signal Routing/Mux',[model '/Mux_fm_in'], ...
    'Inputs','3','Position',[xMux2,yMux2,xMux2+20,yMux2+140]);

add_line(model,'EOM/1','Mux_fm_in/1','autorouting','on');
add_line(model,'Mux_ctrl/1','Mux_fm_in/2','autorouting','on');
add_line(model,'Wind/1','Mux_fm_in/3','autorouting','on');
add_line(model,'Mux_fm_in/1','ForceMoments/1','autorouting','on');

% FM output port 1 = [fx;fy;fz;l;m;n;Va;alpha;beta;wn;we;wd]
% Demux FM output
xDmxFM = xFM2+BW+20; yDmxFM = yMid-60;
add_block('simulink/Signal Routing/Demux',[model '/Demux_fm'], ...
    'Outputs','12','Position',[xDmxFM,yDmxFM,xDmxFM+15,yDmxFM+130]);
add_line(model,'ForceMoments/1','Demux_fm/1','autorouting','on');

% Mux first 6 elements -> EOM input [fx,fy,fz,l,m,n]
xMux3 = xDmxFM+40; yMux3 = yDmxFM;
add_block('simulink/Signal Routing/Mux',[model '/Mux_eom_in'], ...
    'Inputs','6','Position',[xMux3,yMux3,xMux3+20,yMux3+90]);
for k=1:6
    add_line(model,['Demux_fm/' num2str(k)],['Mux_eom_in/' num2str(k)],'autorouting','on');
end
add_line(model,'Mux_eom_in/1','EOM/1','autorouting','on');

%% ---- 3 Output ports: Va (idx 7), alpha (idx 8), beta (idx 9) ----
xOp = xDmxFM+80;
out_fm_idx = [7, 8, 9];
out_names  = {'Va','alpha','beta'};
for k=1:3
    yy = yMid + (k-2)*80;
    add_block('simulink/Sinks/Out1',[model '/out_' out_names{k}], ...
        'Port',num2str(k), ...
        'Position',[xOp,yy,xOp+BW,yy+BH]);
    set_param([model '/out_' out_names{k}],'Name',out_names{k});
    add_line(model,['Demux_fm/' num2str(out_fm_idx(k))], ...
             ['out_' out_names{k} '/1'],'autorouting','on');
end

%% ---- Also add a scope for full state monitoring ----
xSc = xE2+BW+30;
add_block('simulink/Sinks/Scope',[model '/StateScopeV'], ...
    'Position',[xSc,yMid-20,xSc+50,yMid+20]);
add_line(model,'EOM/1','StateScopeV/1','autorouting','on');

%% ---- Solver ----
set_param(model,'SolverType','Variable-step','Solver','ode45', ...
    'StopTime','60','MaxStep','0.05','RelTol','1e-6','AbsTol','1e-8');

save_system(model,[model '.slx']);
fprintf('\n=== mavsim_trim.slx built ===\n');
fprintf('Use run_linmod_trim.m to compute trim and state-space models.\n\n');
