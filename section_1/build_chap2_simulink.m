%% build_chap2_simulink.m
% Auto-builds the Simulink animation model for Section 1 (AE700).
% Run this script once — it creates and saves chap2_sim.slx automatically.
%
% Requirements: drawAircraft.m and chap2_animation.m must be on MATLAB path.
%
% After running:
%   1. Open chap2_sim.slx (it opens automatically)
%   2. Press Run to see the F-4 at the pose set by the Constant blocks
%   3. Change Constant block values and re-run to verify different poses
%   4. Change S-Function parameter to 2 for Part (iv) demo

clear; clc;

model = 'chap2_sim';

%% Close and delete old version if it exists
if bdIsLoaded(model), close_system(model, 0); end
if exist([model '.slx'], 'file'), delete([model '.slx']); end

%% Create new blank model
new_system(model);
open_system(model);

%% ---------------------------------------------------------------
%  Block positions  [left, top, right, bottom]  (pixels)
%% ---------------------------------------------------------------
x_const  = 50;    % x start of constant blocks
x_mux    = 280;   % x of mux
x_sfunc  = 430;   % x of s-function

row_start = 80;   % y of first constant block
row_gap   = 60;   % vertical gap between blocks
bw = 120; bh = 40; % block width, height

labels = {'pn', 'pe', 'pd', 'phi', 'theta', 'psi'};
values = {'0',  '0',  '-300', '0', '0.0873', '0'};
% pd=-300 -> 300 ft altitude; theta=0.0873 rad = 5 deg (trim pitch)

%% ---------------------------------------------------------------
%  Add Constant blocks
%% ---------------------------------------------------------------
const_handles = zeros(1,6);
for i = 1:6
    blk = [model '/const_' labels{i}];
    top = row_start + (i-1)*row_gap;
    add_block('simulink/Sources/Constant', blk, ...
        'Value',    values{i}, ...
        'Position', [x_const, top, x_const+bw, top+bh], ...
        'Name',     ['const_' labels{i}]);
    set_param(blk, 'OutDataTypeStr', 'double');
    const_handles(i) = get_param(blk, 'Handle');
end

%% ---------------------------------------------------------------
%  Add Mux block (6 inputs -> 1 bus)
%% ---------------------------------------------------------------
mux_top = row_start + 1.5*row_gap;
add_block('simulink/Signal Routing/Mux', [model '/Mux'], ...
    'Inputs',   '6', ...
    'Position', [x_mux, mux_top, x_mux+40, mux_top + 5*row_gap + bh]);

%% ---------------------------------------------------------------
%  Add S-Function block (chap2_animation)
%  Parameter 1 = correct order (rotate then translate)
%  Parameter 2 = wrong order   (translate then rotate) for Part iv
%% ---------------------------------------------------------------
sf_top = mux_top + 2*row_gap - 10;
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', [model '/chap2_animation'], ...
    'FunctionName', 'chap2_animation', ...
    'Parameters',   '1', ...
    'Position',     [x_sfunc, sf_top, x_sfunc+160, sf_top+60]);

%% ---------------------------------------------------------------
%  Connect: each Constant -> Mux input port
%% ---------------------------------------------------------------
for i = 1:6
    add_line(model, ...
        ['const_' labels{i} '/1'], ...
        ['Mux/' num2str(i)], ...
        'autorouting', 'on');
end

%% ---------------------------------------------------------------
%  Connect: Mux output -> S-Function input
%% ---------------------------------------------------------------
add_line(model, 'Mux/1', 'chap2_animation/1', 'autorouting', 'on');

%% ---------------------------------------------------------------
%  Solver settings
%% ---------------------------------------------------------------
set_param(model, ...
    'SolverType',       'Fixed-step', ...
    'Solver',           'ode4', ...
    'FixedStep',        '0.02', ...
    'StopTime',         '1', ...
    'SimulationMode',   'normal');

%% ---------------------------------------------------------------
%  Save
%% ---------------------------------------------------------------
save_system(model, [model '.slx']);
fprintf('\nModel "%s.slx" built and saved successfully.\n', model);
fprintf('\n--- HOW TO USE ---\n');
fprintf('1. Press RUN (Ctrl+T) to see the F-4 at current pose.\n');
fprintf('2. Double-click any Constant block to change its value.\n');
fprintf('   Suggested test poses:\n');
fprintf('   Level flight : pd=-300, phi=0,      theta=0,      psi=0\n');
fprintf('   Roll 45 deg  : pd=-300, phi=0.7854, theta=0,      psi=0\n');
fprintf('   Pitch up 20  : pd=-300, phi=0,      theta=0.3491, psi=0\n');
fprintf('   Yaw 90 deg   : pd=-300, phi=0,      theta=0,      psi=1.5708\n');
fprintf('3. For Part (iv): double-click chap2_animation block,\n');
fprintf('   change parameter from 1 to 2, then re-run.\n');
fprintf('   Observe aircraft rotates about world origin instead of CG.\n\n');
