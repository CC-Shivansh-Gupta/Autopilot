%% chap2_test.m
% Test script for F-4 Phantom coordinate frame animation
% AE700 - IIT Bombay | Aircraft: F-4 Phantom
%
% Run this script to verify:
%   (iii) Correct rotation then translation
%   (iv)  Wrong order: translate then rotate
%
% States: uu = [pn; pe; pd; phi; theta; psi]
%   pn    = North position [ft]
%   pe    = East position  [ft]
%   pd    = Down  position [ft]  (positive = downward)
%   phi   = Roll  [rad]
%   theta = Pitch [rad]
%   psi   = Yaw   [rad]

clear; clc; close all;

fprintf('==============================================\n');
fprintf('  F-4 Phantom - Coordinate Frame Animation\n');
fprintf('  AE700 Project - IIT Bombay\n');
fprintf('==============================================\n\n');

%% ---------------------------------------------------------------
%  TEST 1: Static pose checks (Part iii)
%  Verify each rotation axis individually
%% ---------------------------------------------------------------
fprintf('TEST 1: Static pose verification...\n');

test_cases = {
    [0; 0; -100; 0;    0;    0   ], 'Level flight at 100 ft altitude';
    [0; 0; -100; pi/4; 0;    0   ], 'Roll 45 deg right';
    [0; 0; -100; 0;    pi/8; 0   ], 'Pitch up 22.5 deg';
    [0; 0; -100; 0;    0;    pi/4], 'Yaw 45 deg (heading change)';
    [0; 0; -100; pi/6; pi/8; pi/3], 'Combined: roll+pitch+yaw';
    [200; 100; -300; pi/8; pi/12; pi/3], 'Translated + rotated';
};

for k = 1:length(test_cases)
    uu  = test_cases{k, 1};
    lbl = test_cases{k, 2};
    
    figure(k); clf;
    h = [];
    h = drawAircraft(uu, h, 1, k);
    title(sprintf('Test %d: %s', k, lbl), 'FontSize', 12, 'FontWeight', 'bold');
    
    % Mark the CG
    hold on;
    hcg = plot3(uu(1), uu(2), -uu(3), 'r*', 'MarkerSize', 12, 'LineWidth', 2);
    legend([h, hcg], {'Aircraft', 'CG position'}, 'Location', 'best');
    hold off;
    
    fprintf('  %d. %s\n', k, lbl);
    pause(0.5);
end

%% ---------------------------------------------------------------
%  TEST 2: Animation - aircraft flying a circle (Part iii dynamic)
%% ---------------------------------------------------------------
fprintf('\nTEST 2: Animated circular flight path (CORRECT: rotate then translate)...\n');

figure(10); clf;
h = [];

axis equal
axis([-3000 3000 0 6000 440 580]);  % adjust based on your motion
view(3)
grid on
hold on


% Create video object
video = VideoWriter('circular_flight.mp4', 'MPEG-4');
video.FrameRate = 20;   % adjust smoothness
open(video);

t = 0:0.05:15;
Va = 800;
R  = 3000;
omega = Va / R;

% Trajectory storage
traj_x = [];
traj_y = [];
traj_z = [];

for i = 1:length(t)
    psi   = omega * t(i);
    pn    = R * sin(psi);
    pe    = R * (1 - cos(psi));
    pd    = -500;
    phi   = atan(Va^2 / (32.2 * R));
    theta = 0;
    
    uu = [pn; pe; pd; phi; theta; psi];
    h  = drawAircraft(uu, h, 1, 10);

    title(sprintf('F-4 Phantom Circular Flight  |  t = %.1f s  |  Alt = %d ft', ...
                   t(i), round(-pd)), 'FontSize', 11);
    drawnow;

    % Capture frame
    frame = getframe(gcf);
    writeVideo(video, frame);
end

% Close video file
close(video);

fprintf('  Circular flight animation complete.\n');

%% ---------------------------------------------------------------
%  TEST 3: Part (iv) - Wrong order: Translate THEN Rotate
%  Compare side by side with correct order
%% ---------------------------------------------------------------
fprintf('\nTEST 3: Demonstrating WRONG order (translate then rotate)...\n');
fprintf('  -> Watch how aircraft orbits origin instead of rotating in place\n\n');

pn_0 = 200; pe_0 = 150; pd_0 = -300;

% Pre-create both figures so they stay open independently
fig11 = figure(11); clf;
set(fig11, 'Position', [50  200 700 560]);
%title('CORRECT: Rotate then Translate', 'FontSize', 13, 'Color', [0 0.5 0]);

fig12 = figure(12); clf;
set(fig12, 'Position', [770 200 700 560]);
%title('WRONG (Part iv): Translate then Rotate', 'FontSize', 13, 'Color', [0.8 0 0]);

h_correct = [];
h_wrong   = [];

% VideoWriter setup -- one file, composite side-by-side frames
v3           = VideoWriter('rotation_order.mp4', 'MPEG-4');
v3.FrameRate = 15;
v3.Quality   = 90;
open(v3);

for psi = 0 : pi/20 : 2*pi
    uu = [pn_0; pe_0; pd_0; 0; 0; psi];
    
    % Correct order (mode=1)
    h_correct = drawAircraft(uu, h_correct, 1, 11);
    figure(fig11);
    title(sprintf('CORRECT: Rotate \\rightarrow Translate  |  psi=%.0f deg', ...
          rad2deg(psi)), 'FontSize', 12, 'Color', [0 0.5 0]);
    drawnow;
    
    % Wrong order (mode=2)
    h_wrong = drawAircraft(uu, h_wrong, 2, 12);
    figure(fig12);
    title(sprintf('WRONG (Part iv): Translate \\rightarrow Rotate  |  psi=%.0f deg', ...
          rad2deg(psi)), 'FontSize', 12, 'Color', [0.8 0 0]);
    drawnow;

    % Capture both figures as image arrays
    f11 = getframe(fig11);
    f12 = getframe(fig12);
 
    % Make both the same height before tiling (they should match, but just in case)
    h11 = size(f11.cdata, 1);
    h12 = size(f12.cdata, 1);
    target_h = min(h11, h12);
 
    img11 = imresize(f11.cdata, [target_h, NaN]);
    img12 = imresize(f12.cdata, [target_h, NaN]);
 
    % Thin white divider line between the two panels
    divider   = 255 * ones(target_h, 6, 3, 'uint8');
    composite = [img11, divider, img12];
 
    writeVideo(v3, composite);
    
    pause(0.04);
end
close(v3);
fprintf('\nObservation for Part (iv):\n');
fprintf('  CORRECT order: Aircraft rotates about its own CG.\n');
fprintf('  WRONG order:   Aircraft body rotates about the ORIGIN (0,0,0),\n');
fprintf('                 not about its own center. This is physically wrong.\n');

%% ---------------------------------------------------------------
%  ROTATION MATRICES (for reference / report)
%% ---------------------------------------------------------------
fprintf('\n--- Rotation Matrix reference (phi=10, theta=5, psi=30 deg) ---\n');
phi   = deg2rad(10);
theta = deg2rad(5);
psi   = deg2rad(30);

Rx = [1,      0,       0;
      0, cos(phi), sin(phi);   % body to vehicle: transpose of standard
      0,-sin(phi), cos(phi)];

Ry = [ cos(theta), 0, -sin(theta);
       0,          1,  0;
       sin(theta), 0,  cos(theta)];

Rz = [cos(psi), sin(psi), 0;
     -sin(psi), cos(psi), 0;
      0,        0,        1];

% Complete rotation body -> inertial (ZYX order: Rz * Ry * Rx)
R_bv = (Rz * Ry * Rx)';   % transpose gives body->vehicle

fprintf('R_body_to_vehicle =\n');
disp(R_bv);
fprintf('  (Use this to rotate body-frame vectors to inertial NED frame)\n');

fprintf('\nAll tests complete!\n');
