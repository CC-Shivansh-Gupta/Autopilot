%% Section12_PathPlanning.m
% =========================================================
% AE700 — Section 12: Path Planning (RRT)
% IIT Bombay | F-4 Phantom
%
% Creates a random 5-building obstacle map, runs both
% straight-line and Dubins RRT, and plots both planned
% paths on a 3-D map of the buildings.
%
% Requires: F4_params.m, rrt.m, dubins_parameters.m
%
% Run: >> Section12_PathPlanning
% =========================================================

clear; clc; close all;
rng(42);   % reproducible random map

F4_params;

%% ---- RRT parameters (add to F4_params.m) ----
P.R_min    = 300;    % minimum Dubins radius [ft]
P.rrt_step = 150;    % RRT step size [ft]

%% ---- Random 5-building map ----
%  buildings: Nx5 [pn, pe, h, width, height]  (h=ground level=0, height>0)
world_W = 2000;   % north extent [ft]
world_H = 2000;   % east extent  [ft]

map.world_width  = world_W;
map.world_height = world_H;

n_bldg = 5;
map.buildings = zeros(n_bldg, 5);
for b = 1:n_bldg
    map.buildings(b,:) = [200 + rand*(world_W-400), ...  % pn centre
                           200 + rand*(world_H-400), ...  % pe centre
                           0, ...                          % ground z
                           100 + rand*150, ...             % width [ft]
                           150 + rand*200];                % height [ft]
end

%% ---- Start and goal configurations ----
start = [50,   50,  200, pi/4];        % [pn, pe, h, chi]
goal  = [1900, 1900, 200, pi/4];

%% ---- Run RRT ----
fprintf('Running straight-line RRT...\n');
wp_line   = rrt(map, start, goal, P, false);
fprintf('  Found path with %d waypoints.\n', size(wp_line,1));

fprintf('Running Dubins RRT...\n');
wp_dubins = rrt(map, start, goal, P, true);
fprintf('  Found path with %d waypoints.\n', size(wp_dubins,1));

%% ---- 3D Plot ----
FS = 12;
figure('Name','RRT Path Planning','Position',[0 0 900 700]);

hold on;

% Draw buildings as 3D boxes
for b = 1:n_bldg
    bldg = map.buildings(b,:);
    pn0 = bldg(1) - bldg(4)/2;
    pe0 = bldg(2) - bldg(4)/2;
    h_b = bldg(5);

    % Draw box faces (simplified: just a vertical patch)
    pe_box = [pe0, pe0+bldg(4), pe0+bldg(4), pe0, pe0];
    pn_box = [pn0, pn0,         pn0+bldg(4), pn0+bldg(4), pn0];
    fill3(pe_box, pn_box, repmat(h_b,1,5), [0.7 0.7 0.7], ...
          'FaceAlpha',0.5, 'EdgeColor','k', 'HandleVisibility','off');
    % Vertical edges (4 corners)
    for cx = [pn0, pn0+bldg(4)]
        for cy = [pe0, pe0+bldg(4)]
            plot3([cy cy],[cx cx],[0 h_b],'k-','HandleVisibility','off');
        end
    end
end

% Paths (altitude = constant here; Dubins varies)
if size(wp_line,1) > 1
    plot3(wp_line(:,2),   wp_line(:,1),   repmat(start(3), size(wp_line,1),1), ...
          'b-o','LineWidth',2,'MarkerSize',5,'DisplayName','Straight-line RRT');
end
if size(wp_dubins,1) > 1
    plot3(wp_dubins(:,2), wp_dubins(:,1), repmat(start(3), size(wp_dubins,1),1), ...
          'r-s','LineWidth',2,'MarkerSize',5,'DisplayName','Dubins RRT');
end

% Start and goal
plot3(start(2), start(1), start(3), 'gp','MarkerSize',15,'MarkerFaceColor','g','DisplayName','Start');
plot3(goal(2),  goal(1),  goal(3),  'rp','MarkerSize',15,'MarkerFaceColor','r','DisplayName','Goal');

xlabel('East [ft]','FontSize',FS);
ylabel('North [ft]','FontSize',FS);
zlabel('Altitude [ft]','FontSize',FS);
title('RRT Path Planning — Straight-Line (Alg 11) vs Dubins (Alg 12)','FontSize',FS);
legend('FontSize',11,'Location','northwest');
grid on; view(30,30);
xlim([0 world_H]); ylim([0 world_W]); zlim([0 400]);

fprintf('Section 12 complete.\n');
