%% build_mavsim_trim.m  (v2 — fixed input wiring)
% =========================================================
% Builds mavsim_trim.slx matching Appendix F Figure F.1.
%
% Structure:
%   INPUTS  (4 Inports)  : delta_e, delta_a, delta_r, delta_t
%   STATES  (12)         : pn..r  (inside EOM S-function)
%   OUTPUTS (3 Outports) : Va, alpha, beta
%
% Forces_moments S-function receives a single 22-element bus:
%   [state(12); controls(4); wind(6)]
%
% Run order:
%   1. >> F4_params          (loads P into workspace)
%   2. >> build_mavsim_trim  (creates mavsim_trim.slx)
%   3. >> Section4_Linear    (uses trim + linmod on the model)
% =========================================================

clear; clc;
F4_params;   % loads P

model = 'mavsim_trim';
if bdIsLoaded(model), close_system(model,0); end
if exist([model '.slx'],'file'), delete([model '.slx']); end
new_system(model);
open_system(model);

%% ---- Layout helpers ----
BH = 40; BW = 130; GAP = 15;
col = @(n) 50  + (n-1)*220;
row = @(n) 50  + (n-1)*(BH+GAP);

%% ============================================================
%  1. INPORTS  [de, da, dr, dt]
% ============================================================
inport_names = {'delta_e','delta_a','delta_r','delta_t'};
for i = 1:4
    add_block('simulink/Sources/In1',[model '/' inport_names{i}], ...
        'Port',num2str(i), ...
        'Position',[col(1),row(i),col(1)+BW,row(i)+BH]);
end

%% ============================================================
%  2. MUX controls  4->1
% ============================================================
yMt = row(1); yMb = row(4)+BH;
add_block('simulink/Signal Routing/Mux',[model '/Mux_ctrl'], ...
    'Inputs','4', ...
    'Position',[col(2),yMt,col(2)+20,yMb]);
for i = 1:4
    add_line(model,[inport_names{i} '/1'],['Mux_ctrl/' num2str(i)],'autorouting','on');
end

%% ============================================================
%  3. ZERO WIND  (6x1 constant)
% ============================================================
add_block('simulink/Sources/Constant',[model '/Wind'], ...
    'Value','zeros(6,1)', ...
    'OutDataTypeStr','double', ...
    'Position',[col(2),row(5),col(2)+BW,row(5)+BH]);

%% ============================================================
%  4. EOM S-Function  (chap3_eom)
%     Input  : forces/moments (6x1)
%     Output : state (12x1)
% ============================================================
pstr = sprintf('%g,%g,%g,%g,%g, 0,0,-1000, %g,0,0, 0,%g,0, 0,0,0', ...
    P.mass, P.Jx, P.Jy, P.Jz, P.Jxz, P.Va_trim, P.alpha_trim);

xEOM = col(4); yEOM = row(2);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    [model '/EOM'], ...
    'FunctionName','chap3_eom', ...
    'Parameters',pstr, ...
    'Position',[xEOM,yEOM,xEOM+160,yEOM+80]);

%% ============================================================
%  5. MEMORY  (breaks algebraic loop on state feedback)
% ============================================================
ic_str = sprintf('[0,0,-1000,%g,0,0,0,%g,0,0,0,0]', P.Va_trim, P.alpha_trim);
xMem = col(3); yMem = yEOM+110;
add_block('simulink/Discrete/Memory',[model '/StateMem'], ...
    'X0',ic_str, ...
    'Position',[xMem,yMem,xMem+BW,yMem+BH]);

%% ============================================================
%  6. MUX for Forces_Moments: [state(12); ctrl(4); wind(6)] = 22
% ============================================================
xMFM = col(3); yMFM = yEOM;
add_block('simulink/Signal Routing/Mux',[model '/Mux_FM'], ...
    'Inputs','3', ...
    'Position',[xMFM,yMFM,xMFM+20,yMFM+80]);

add_line(model,'StateMem/1','Mux_FM/1','autorouting','on');
add_line(model,'Mux_ctrl/1','Mux_FM/2','autorouting','on');
add_line(model,'Wind/1',    'Mux_FM/3','autorouting','on');

%% ============================================================
%  7. FORCES_MOMENTS S-Function
%     Input  : 22x1 [state; ctrl; wind]
%     Output : 12x1 [fx,fy,fz,l,m,n,Va,alpha,beta,wn,we,wd]
% ============================================================
xFM = col(3)+100; yFM = yEOM-80;
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    [model '/Forces_Moments'], ...
    'FunctionName','chap4_forces_moments_sfcn', ...
    'Position',[xFM,yFM,xFM+180,yFM+60]);

add_line(model,'Mux_FM/1','Forces_Moments/1','autorouting','on');

%% ---- Extract elements 1:6 [fx..n] and feed to EOM ----
add_block('simulink/Signal Routing/Selector',[model '/FM_to_EOM'], ...
    'Indices','1:6', ...
    'InputPortWidth','12', ...
    'Position',[xFM+200,yFM,xFM+250,yFM+40]);
add_line(model,'Forces_Moments/1','FM_to_EOM/1','autorouting','on');
add_line(model,'FM_to_EOM/1','EOM/1','autorouting','on');

%% ---- Close state feedback loop ----
add_line(model,'EOM/1','StateMem/1','autorouting','on');

%% ============================================================
%  8. DEMUX FM output 12->individual
% ============================================================
xDmx = xFM+270; yDmx = yFM;
add_block('simulink/Signal Routing/Demux',[model '/Demux_FM'], ...
    'Outputs','12', ...
    'Position',[xDmx,yDmx,xDmx+20,yDmx+80]);
add_line(model,'Forces_Moments/1','Demux_FM/1','autorouting','on');

%% ============================================================
%  9. OUTPORTS  Va(7), alpha(8), beta(9)
% ============================================================
outport_names = {'Va','alpha','beta'};
fm_indices    = [7, 8, 9];
xOut = xDmx+80;
for i = 1:3
    add_block('simulink/Sinks/Out1',[model '/' outport_names{i}], ...
        'Port',num2str(i), ...
        'Position',[xOut,yDmx+(i-1)*55,xOut+BW,yDmx+(i-1)*55+BH]);
    add_line(model, ...
        ['Demux_FM/' num2str(fm_indices(i))], ...
        [outport_names{i} '/1'],'autorouting','on');
end

%% ============================================================
%  10. SOLVER
% ============================================================
set_param(model, ...
    'SolverType','Variable-step','Solver','ode45', ...
    'StopTime','30','MaxStep','0.05', ...
    'AbsTol','1e-8','RelTol','1e-6');

save_system(model,[model '.slx']);

fprintf('\n=== mavsim_trim.slx built successfully ===\n');
fprintf('Input-Output structure (Fig F.1):\n');
fprintf('  Inports  (4): delta_e, delta_a, delta_r, delta_t\n');
fprintf('  States  (12): pn,pe,pd,u,v,w,phi,theta,psi,p,q,r\n');
fprintf('  Outports (3): Va, alpha, beta\n\n');
fprintf('Next steps:\n');
fprintf('  >> [x,u,y,dx] = trim(''mavsim_trim'', x0,u0,y0,ix,iu,iy,dx0,idx)\n');
fprintf('  >> [A,B,C,D]  = linmod(''mavsim_trim'', x_trim, u_trim)\n\n');
