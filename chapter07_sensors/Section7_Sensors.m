%% Section7_Sensors.m
% =========================================================
% AE700 — Section 7: Sensor Simulation
% IIT Bombay | F-4 Phantom
%
% Runs the equations of motion at trim for 30 s, calls
% sensors() at each timestep, and plots all sensor outputs
% against the true state.
%
% Requires: F4_params.m, eom.m, forces_moments.m,
%           compute_trim.m, sensors.m
%
% Run: >> Section7_Sensors
% =========================================================

clear; clc; close all;

% ---- Add sensor noise params (student adds to F4_params.m) ----
F4_params;                          % loads struct P

% Temporary noise defaults (move these into F4_params.m)
P.sigma_gyro   = 0.001;    % rad/s  gyro noise std     (Eq 7.5)
P.sigma_accel  = 0.001;    % ft/s^2 accel noise std    (Eq 7.3)
P.sigma_baro   = 0.5;      % lb/ft^2 baro noise std    (Eq 7.9)
P.sigma_pitot  = 0.01;     % lb/ft^2 pitot noise std   (Eq 7.10)
P.sigma_GPS    = 1.0;      % ft GPS position noise std (Eq 7.18)
P.tau_g        = 100;      % s  gyro bias time constant
P.tau_GPS      = 1000;     % s  GPS bias time constant
P.Ts           = 0.02;     % s  simulation timestep

%% ---- Trim ----
[x_trim, u_trim] = compute_trim(P.Va_trim, 0, Inf, P);
x_trim(3) = -3000;         % start at 3000 ft altitude

%% ---- Simulate ----
T  = 30;   dt = P.Ts;
t  = (0:dt:T)';
N  = length(t);
X  = zeros(N, 12);
X(1,:) = x_trim';

Y_gyro   = zeros(N, 3);
Y_accel  = zeros(N, 3);
Y_press  = zeros(N, 2);   % [static, dynamic]
Y_gps    = zeros(N, 5);   % [n, e, h, Vg, course]

wind = zeros(6,1);
oo   = odeset('RelTol',1e-6,'AbsTol',1e-8);

for k = 1:N-1
    x = X(k,:)';
    y = sensors(x, u_trim, wind, P);

    Y_gyro(k,:)  = [y.gyro_x,  y.gyro_y,  y.gyro_z];
    Y_accel(k,:) = [y.accel_x, y.accel_y, y.accel_z];
    Y_press(k,:) = [y.static_pressure, y.dynamic_pressure];
    Y_gps(k,:)   = [y.gps_n, y.gps_e, y.gps_h, y.gps_Vg, y.gps_course];

    [~, Xk]     = ode45(@(tt,xx) eom(xx, u_trim, wind, P), [0 dt], x, oo);
    X(k+1,:)    = Xk(end,:);
end

%% ---- Plots ----
FS = 12;

figure('Name','Gyros vs True Rates','Position',[0 0 900 500]);
names = {'\phi (p)', '\theta (q)', '\psi (r)'};
true_rates = [10, 11, 12];   % state indices
for i = 1:3
    subplot(3,1,i);
    plot(t, rad2deg(X(:,true_rates(i))), 'b-', 'DisplayName','True'); hold on;
    plot(t, rad2deg(Y_gyro(:,i)), 'r--', 'DisplayName','Gyro');
    ylabel('[deg/s]','FontSize',FS); legend; grid on;
    title(sprintf('Gyro %s', names{i}),'FontSize',FS);
end
xlabel('Time [s]','FontSize',FS);

figure('Name','Pressure Sensors','Position',[0 0 900 400]);
subplot(2,1,1);
plot(t, Y_press(:,1)); ylabel('Static P [lb/ft^2]','FontSize',FS);
title('Static Pressure (Eq 7.9)','FontSize',FS); grid on;
subplot(2,1,2);
plot(t, Y_press(:,2)); ylabel('Dynamic P [lb/ft^2]','FontSize',FS);
title('Dynamic Pressure (Eq 7.10)','FontSize',FS); grid on;
xlabel('Time [s]','FontSize',FS);

figure('Name','GPS vs True Position','Position',[0 0 900 500]);
labels = {'p_n [ft]','p_e [ft]','h [ft]','V_g [ft/s]','\chi [rad]'};
for i = 1:3
    subplot(3,1,i);
    plot(t, X(:,i), 'b-', 'DisplayName','True'); hold on;
    plot(t, Y_gps(:,i), 'r--', 'DisplayName','GPS');
    ylabel(labels{i},'FontSize',FS); legend; grid on;
end
xlabel('Time [s]','FontSize',FS);
sgtitle('GPS Position vs True (Eqs 7.18-7.20)','FontSize',FS+1);

fprintf('Section 7 complete.\n');
