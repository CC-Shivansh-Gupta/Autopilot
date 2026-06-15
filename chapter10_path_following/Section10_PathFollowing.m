%% Section10_PathFollowing.m
% =========================================================
% AE700 — Section 10: Path Following
% IIT Bombay | F-4 Phantom
%
% Test 1: straight-line segment from [0,0,-100] ft toward
%         [1000,1000,-100] ft with wind [5,-3,0] ft/s.
% Test 2: orbit at centre [500,500,-100] ft, radius 200 ft.
% Plots ground track for both tests.
%
% Requires: F4_params.m, eom.m, forces_moments.m,
%           compute_trim.m, path_follower.m, Section5 autopilot.
%
% Run: >> Section10_PathFollowing
% =========================================================

clear; clc; close all;

F4_params;

%% Path-following gains (add these to F4_params.m)
P.chi_inf = 70 * pi/180;   % straight-line approach half-angle [rad] (Eq 10.4)
P.k_path  = 0.02;          % straight-line cross-track gain
P.k_orbit = 0.7;           % orbit radial-error gain             (Eq 10.12)

%% ---- Trim ----
[x_trim, u_trim] = compute_trim(P.Va_trim, 0, Inf, P);
x_trim(3) = -100;   % start at 100 ft altitude (positive up -> pd negative)

%% ---- Wind ----
wind_ss  = [5; -3; 0];    % steady inertial wind [ft/s]
wind_6   = [wind_ss; 0; 0; 0];

%% ============================================================
%  TEST 1 — Straight-line path
% ============================================================
path_line.flag = 1;
path_line.Va_d = P.Va_trim;
path_line.r    = [0; 0; -100];          % point on line (NED)
q_raw          = [1000; 1000; 0] - path_line.r(1:2);
path_line.q    = [q_raw/norm(q_raw); 0];  % unit direction

T  = 40;  dt = 0.05;
t  = (0:dt:T)';  N = length(t);
X1 = zeros(N,12);  X1(1,:) = x_trim';

oo = odeset('RelTol',1e-6,'AbsTol',1e-8);

for k = 1:N-1
    x = X1(k,:)';

    % Build state struct for path_follower
    fm  = forces_moments(x, u_trim, wind_6, P);
    state.pn  = x(1);  state.pe = x(2);  state.h = -x(3);
    state.Va  = fm(7);
    state.chi = x(9);   % use psi as chi proxy (simplified)

    cmd = path_follower(path_line, state, P);

    % TODO: feed cmd into autopilot to get delta
    % For skeleton: fly trim controls
    delta = u_trim;

    [~, Xk]   = ode45(@(tt,xx) eom(xx, delta, wind_6, P), [0 dt], x, oo);
    X1(k+1,:) = Xk(end,:);
end

%% ============================================================
%  TEST 2 — Orbit path
% ============================================================
path_orb.flag   = 2;
path_orb.Va_d   = P.Va_trim;
path_orb.c      = [500; 500; -100];   % orbit centre (NED)
path_orb.rho    = 200;                % radius [ft]
path_orb.lambda = +1;                 % clockwise

X2 = zeros(N,12);  X2(1,:) = x_trim';
X2(1,1) = 500;  X2(1,2) = 300;       % start near orbit entry

for k = 1:N-1
    x = X2(k,:)';

    fm = forces_moments(x, u_trim, wind_6, P);
    state.pn  = x(1);  state.pe = x(2);  state.h = -x(3);
    state.Va  = fm(7);  state.chi = x(9);

    cmd = path_follower(path_orb, state, P);

    delta = u_trim;   % TODO: connect autopilot

    [~, Xk]   = ode45(@(tt,xx) eom(xx, delta, wind_6, P), [0 dt], x, oo);
    X2(k+1,:) = Xk(end,:);
end

%% ---- Plots ----
FS = 12;
figure('Name','Path Following Ground Track','Position',[0 0 800 700]);

subplot(2,1,1);
plot(X1(:,2), X1(:,1), 'b-', 'LineWidth',1.5, 'DisplayName','MAV track'); hold on;
plot([path_line.r(2), path_line.r(2)+200*path_line.q(2)], ...
     [path_line.r(1), path_line.r(1)+200*path_line.q(1)], ...
     'r--', 'LineWidth',2, 'DisplayName','Desired line');
xlabel('East [ft]','FontSize',FS); ylabel('North [ft]','FontSize',FS);
title('Straight-Line Path Following (Alg 5, wind=[5,-3,0] ft/s)','FontSize',FS);
legend; grid on; axis equal;

subplot(2,1,2);
theta_orbit = linspace(0, 2*pi, 200);
plot(X2(:,2), X2(:,1), 'b-', 'LineWidth',1.5, 'DisplayName','MAV track'); hold on;
plot(path_orb.c(2) + path_orb.rho*sin(theta_orbit), ...
     path_orb.c(1) + path_orb.rho*cos(theta_orbit), ...
     'r--', 'LineWidth',2, 'DisplayName','Desired orbit');
plot(path_orb.c(2), path_orb.c(1), 'kx', 'MarkerSize',10, 'DisplayName','Centre');
xlabel('East [ft]','FontSize',FS); ylabel('North [ft]','FontSize',FS);
title('Orbit Path Following (Alg 6, \rho=200 ft, CW)','FontSize',FS);
legend; grid on; axis equal;

fprintf('Section 10 complete.\n');
