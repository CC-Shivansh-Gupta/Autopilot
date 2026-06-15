%% Section13_Camera.m
% =========================================================
% AE700 — Section 13: Gimbal Model and Camera
% IIT Bombay | F-4 Phantom
%
% MAV orbits at 200 ft altitude over a stationary ground
% target at [500, 500, 0] ft.  Calls gimbal_model() at
% each timestep and plots the image-plane track of the
% target over one full orbit.
%
% Requires: F4_params.m, eom.m, forces_moments.m,
%           compute_trim.m, gimbal_model.m,
%           path_follower.m (orbit mode, Ch 10)
%
% Run: >> Section13_Camera
% =========================================================

clear; clc; close all;

F4_params;

%% ---- Camera parameters (add to F4_params.m) ----
P.f_cam      = 500;    % focal length [pixels]
P.img_width  = 640;    % image width  [pixels]
P.img_height = 480;    % image height [pixels]

%% ---- Path following params (from Ch 10) ----
P.k_orbit = 0.7;
P.k_path  = 0.02;
P.chi_inf = 70*pi/180;

%% ---- Target and orbit definition ----
target_pos = [500; 500; 0];    % ground target NED [ft]
orbit_c    = [500; 500; -200]; % orbit centre NED (alt=200 ft)
orbit_r    = 400;              % orbit radius [ft]
orbit_dir  = +1;               % +1 = clockwise (lambda)

%% ---- Trim: set up MAV on orbit entry point ----
[x_trim, u_trim] = compute_trim(P.Va_trim, 0, Inf, P);

% Place MAV at south point of orbit, heading east (start of CW orbit)
x0      = x_trim;
x0(1)   = orbit_c(1) + orbit_r;   % pn = centre_n + r (south of centre)
x0(2)   = orbit_c(2);              % pe = centre_e
x0(3)   = orbit_c(3);              % pd = -200 ft

%% ---- Simulation: one orbit (~2*pi*r / Va_trim seconds) ----
T_orbit = 2*pi*orbit_r / P.Va_trim * 1.1;   % add 10% margin
dt  = 0.05;
t   = (0:dt:T_orbit)';
N   = length(t);
X   = zeros(N,12);  X(1,:) = x0';

AZ      = zeros(N,1);
EL      = zeros(N,1);
IMG     = zeros(N,2);    % [x_img, y_img]

wind = zeros(6,1);
oo   = odeset('RelTol',1e-6,'AbsTol',1e-8);

% Orbit path struct for path_follower
path_orb.flag   = 2;
path_orb.Va_d   = P.Va_trim;
path_orb.c      = orbit_c;
path_orb.rho    = orbit_r;
path_orb.lambda = orbit_dir;

for k = 1:N-1
    x = X(k,:)';

    %% Gimbal model
    [az, el, img_pt] = gimbal_model(x, target_pos, P);
    AZ(k)     = az;
    EL(k)     = el;
    IMG(k,:)  = img_pt';

    %% Path follower -> autopilot (placeholder: trim controls)
    fm = forces_moments(x, u_trim, wind, P);
    state.pn  = x(1);  state.pe = x(2);  state.h = -x(3);
    state.Va  = fm(7);  state.chi = x(9);
    cmd   = path_follower(path_orb, state, P);   % Ch 10
    delta = u_trim;   % TODO: connect autopilot with cmd

    [~, Xk]  = ode45(@(tt,xx) eom(xx, delta, wind, P), [0 dt], x, oo);
    X(k+1,:) = Xk(end,:);
end
% Last timestep
[AZ(N), EL(N), img_tmp] = gimbal_model(X(N,:)', target_pos, P);
IMG(N,:) = img_tmp';

%% ---- Plots ----
FS = 12;

figure('Name','Ground Track & Orbit','Position',[0 0 700 600]);
theta_c = linspace(0, 2*pi, 200);
plot(orbit_c(2) + orbit_r*sin(theta_c), orbit_c(1) + orbit_r*cos(theta_c), ...
     'k--', 'LineWidth',1.5, 'DisplayName','Desired orbit');  hold on;
plot(X(:,2), X(:,1), 'b-', 'LineWidth',1.5, 'DisplayName','MAV track');
plot(target_pos(2), target_pos(1), 'r*', 'MarkerSize',15, 'DisplayName','Target');
xlabel('East [ft]','FontSize',FS); ylabel('North [ft]','FontSize',FS);
title('MAV Orbit — 200 ft altitude','FontSize',FS);
legend; grid on; axis equal;

figure('Name','Gimbal Angles','Position',[0 0 800 400]);
subplot(2,1,1);
plot(t, rad2deg(AZ), 'b-', 'LineWidth',1.5);
ylabel('Azimuth [deg]','FontSize',FS); grid on;
title('Gimbal Azimuth over Orbit (Eq 13.x)','FontSize',FS);
subplot(2,1,2);
plot(t, rad2deg(EL), 'r-', 'LineWidth',1.5);
ylabel('Elevation [deg]','FontSize',FS); xlabel('Time [s]','FontSize',FS);
title('Gimbal Elevation over Orbit','FontSize',FS); grid on;

figure('Name','Image-Plane Track','Position',[0 0 600 500]);
plot(IMG(:,1), IMG(:,2), 'b-', 'LineWidth',1.5); hold on;
plot(0, 0, 'r+', 'MarkerSize',20, 'LineWidth',2, 'DisplayName','Image centre');
rectangle('Position', [-P.img_width/2, -P.img_height/2, P.img_width, P.img_height], ...
          'EdgeColor','k', 'LineStyle','--', 'LineWidth',1.5);
xlabel('x_{img} [px]','FontSize',FS); ylabel('y_{img} [px]','FontSize',FS);
title('Image-Plane Track of Ground Target (Pinhole Model)','FontSize',FS);
axis equal; grid on;
xlim([-P.img_width/2*1.2,  P.img_width/2*1.2]);
ylim([-P.img_height/2*1.2, P.img_height/2*1.2]);

fprintf('Section 13 complete.\n');
