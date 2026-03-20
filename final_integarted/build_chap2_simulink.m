%% build_chap2_simulink.m
% =========================================================
% Builds chap2_sim.slx — static pose animation model
% AE700 — IIT Bombay | F-4 Phantom
%
% Simulink layout:
%   6 Constant blocks (pn,pe,pd,phi,theta,psi)
%   -> Mux -> chap2_animation S-Function -> display
%
% Run once: >> build_chap2_simulink
% Then Ctrl+T to run.
% =========================================================

clear; clc;
F4_params;

model='chap2_sim';
if bdIsLoaded(model), close_system(model,0); end
if exist([model '.slx'],'file'), delete([model '.slx']); end
new_system(model); open_system(model);

%% Layout
BW=120; BH=40; GAP=55;
row=@(i) 60+(i-1)*(BH+GAP);
labels={'pn','pe','pd','phi','theta','psi'};
values={'0','0','-300','0','0.0873','0'};   % pd=-300 ft, theta=5 deg trim

%% Constant blocks
for i=1:6
    blk=[model '/const_' labels{i}];
    add_block('simulink/Sources/Constant',blk,'Value',values{i},...
              'Position',[50,row(i),50+BW,row(i)+BH]);
    set_param(blk,'OutDataTypeStr','double');
end

%% Mux
ymt=row(1); ymb=row(6)+BH;
add_block('simulink/Signal Routing/Mux',[model '/Mux'],...
    'Inputs','6','Position',[220,ymt,250,ymb]);
for i=1:6
    add_line(model,['const_' labels{i} '/1'],['Mux/' num2str(i)],'autorouting','on');
end

%% Animation S-Function
yef=ymt+2*(BH+GAP);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',...
    [model '/chap2_animation'],'FunctionName','chap2_animation','Parameters','1',...
    'Position',[300,yef,470,yef+70]);
add_line(model,'Mux/1','chap2_animation/1','autorouting','on');

%% Solver
set_param(model,'SolverType','Fixed-step','Solver','ode4',...
          'FixedStep','0.02','StopTime','1');
save_system(model,[model '.slx']);

fprintf('chap2_sim.slx built. Press Ctrl+T to view F-4 at default pose.\n');
fprintf('\nSuggested test poses (edit Constant blocks):\n');
fprintf('  Level flight  : pd=-300  phi=0      theta=0      psi=0\n');
fprintf('  Roll 45°      : pd=-300  phi=0.785  theta=0      psi=0\n');
fprintf('  Pitch up 20°  : pd=-300  phi=0      theta=0.349  psi=0\n');
fprintf('  Yaw 90°       : pd=-300  phi=0      theta=0      psi=1.571\n');
fprintf('Part (iv) demo  : change chap2_animation parameter from 1 to 2.\n');
