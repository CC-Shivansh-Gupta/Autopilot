%% Section11_PathManager.m
% =========================================================
% AE700 — Section 11: Path Manager
% IIT Bombay | F-4 Phantom
%
% Flies a 4-waypoint rectangular circuit using all three
% manager types (line, fillet, dubins) and overlays the
% ground tracks on one figure.
%
% Requires: F4_params.m, eom.m, forces_moments.m,
%           compute_trim.m, path_manager.m,
%           dubins_parameters.m, path_follower.m
%
% Run: >> Section11_PathManager
% =========================================================

clear; clc; close all;

F4_params;
P.R_min       = 300;     % minimum turn radius [ft]  (add to F4_params.m)
P.chi_inf     = 70*pi/180;
P.k_path      = 0.02;
P.k_orbit     = 0.7;

%% ---- 4 rectangular waypoints (pn, pe, h, chi) ----
% chi at each corner is tangent to the rectangle edge leaving that corner.
waypoints = [   0,    0,  100,  0;          % SW corner, heading east
              500,    0,  100,  pi/2;       % SE corner, heading north
              500,  500,  100,  pi;         % NE corner, heading west
                0,  500,  100, -pi/2];      % NW corner, heading south

%% ---- Trim ----
[x_trim, u_trim] = compute_trim(P.Va_trim, 0, Inf, P);
x_trim(3) = -100;

T  = 60;  dt = 0.05;
t  = (0:dt:T)';  N = length(t);
wind = zeros(6,1);
oo   = odeset('RelTol',1e-6,'AbsTol',1e-8);

track = struct();
managers = {'line', 'fillet', 'dubins'};
colors   = {'b', 'r', 'g'};

for m = 1:3
    P.manager_type = managers{m};
    X = zeros(N, 12);
    X(1,:) = x_trim';
    % Reset persistent state in path_manager
    clear path_manager;

    for k = 1:N-1
        x = X(k,:)';
        fm = forces_moments(x, u_trim, wind, P);

        state.pn  = x(1);  state.pe = x(2);  state.h = -x(3);
        state.Va  = fm(7);  state.chi = x(9);

        path  = path_manager(waypoints, state, P);
        cmd   = path_follower(path, state, P);

        % TODO: connect cmd to inner-loop autopilot -> delta
        delta = u_trim;    % placeholder

        [~, Xk]  = ode45(@(tt,xx) eom(xx, delta, wind, P), [0 dt], x, oo);
        X(k+1,:) = Xk(end,:);
    end

    track.(managers{m}) = X;
end

%% ---- Plot all three ground tracks ----
FS = 12;
figure('Name','Path Manager Comparison','Position',[0 0 750 700]);
hold on;

% Draw waypoint rectangle
wp_pn = [waypoints(:,1); waypoints(1,1)];
wp_pe = [waypoints(:,2); waypoints(1,2)];
plot(wp_pe, wp_pn, 'k--', 'LineWidth',1.5, 'DisplayName','Waypoints');
plot(waypoints(:,2), waypoints(:,1), 'ks', 'MarkerSize',10, ...
     'MarkerFaceColor','k', 'HandleVisibility','off');

for m = 1:3
    X = track.(managers{m});
    plot(X(:,2), X(:,1), colors{m}, 'LineWidth',1.5, ...
         'DisplayName', [upper(managers{m}(1)) managers{m}(2:end) ' manager']);
end

xlabel('East [ft]','FontSize',FS);  ylabel('North [ft]','FontSize',FS);
title('Path Manager: Line / Fillet / Dubins (4-waypoint rectangle)','FontSize',FS);
legend('FontSize',11,'Location','best'); grid on; axis equal;

fprintf('Section 11 complete.\n');
