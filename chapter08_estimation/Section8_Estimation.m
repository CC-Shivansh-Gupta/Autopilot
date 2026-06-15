%% Section8_Estimation.m
% =========================================================
% AE700 — Section 8: State Estimation
% IIT Bombay | F-4 Phantom
%
% Full closed-loop simulation:
%   eom -> sensors -> estimate_states -> autopilot (on xhat)
% Plots true vs estimated states for phi, theta, pn, pe, h, Va.
%
% Requires: F4_params.m, eom.m, forces_moments.m,
%           compute_trim.m, sensors.m, estimate_states.m
%
% Run: >> Section8_Estimation
% =========================================================

clear; clc; close all;

F4_params;

%% ---- Sensor noise params (copy from Section7 or add to F4_params.m) ----
P.sigma_gyro  = 0.001;    P.tau_g   = 100;
P.sigma_accel = 0.001;    P.sigma_baro  = 0.5;
P.sigma_pitot = 0.01;     P.sigma_GPS   = 1.0;
P.tau_GPS     = 1000;     P.Ts          = 0.02;

%% ---- EKF noise tuning (student tunes these) ----
P.Q_att  = diag([1e-6, 1e-6]);     % attitude EKF process noise
P.R_att  = diag([0.1, 0.1, 0.1]); % accelerometer measurement noise
P.Q_gps  = diag([0,0,0,0,0,0,0]); % GPS EKF process noise (Eq 8.34)
P.R_gps  = diag([1,1,1,1]);        % GPS measurement noise

%% ---- Trim ----
[x_trim, u_trim] = compute_trim(P.Va_trim, 0, Inf, P);
x_trim(3) = -3000;

%% ---- Simulation loop ----
T  = 60;  dt = P.Ts;
t  = (0:dt:T)';  N = length(t);
X     = zeros(N,12);
X(1,:) = x_trim';
XHAT  = zeros(N,15);    % estimated states (fields of xhat, unpacked)

wind = zeros(6,1);
oo   = odeset('RelTol',1e-6,'AbsTol',1e-8);

for k = 1:N-1
    x = X(k,:)';

    % Sensor measurements
    y_sens = sensors(x, u_trim, wind, P);

    % State estimation
    xhat = estimate_states(y_sens, u_trim, P);

    XHAT(k,:) = [xhat.pn, xhat.pe, xhat.h, xhat.Va, xhat.alpha, xhat.beta, ...
                 xhat.phi, xhat.theta, xhat.chi, xhat.psi, xhat.Vg, ...
                 xhat.wn, xhat.we, xhat.p, xhat.q];

    % Propagate true dynamics
    [~, Xk]   = ode45(@(tt,xx) eom(xx, u_trim, wind, P), [0 dt], x, oo);
    X(k+1,:)  = Xk(end,:);
end

%% ---- Plots: true vs estimated ----
FS = 12;  LW = 1.5;

figure('Name','Attitude Estimation','Position',[0 0 900 500]);
subplot(2,1,1);
plot(t, rad2deg(X(:,7)),  'b-',  'LineWidth',LW, 'DisplayName','True \phi'); hold on;
plot(t, rad2deg(XHAT(:,7)),'r--','LineWidth',LW, 'DisplayName','EKF \phi');
ylabel('\phi [deg]','FontSize',FS); legend; grid on;
title('Roll Angle — True vs Estimated (Sec 8.9)','FontSize',FS);

subplot(2,1,2);
plot(t, rad2deg(X(:,8)),  'b-',  'LineWidth',LW, 'DisplayName','True \theta'); hold on;
plot(t, rad2deg(XHAT(:,8)),'r--','LineWidth',LW, 'DisplayName','EKF \theta');
ylabel('\theta [deg]','FontSize',FS); xlabel('Time [s]','FontSize',FS);
legend; grid on;
title('Pitch Angle — True vs Estimated','FontSize',FS);

figure('Name','Position / Airspeed Estimation','Position',[0 0 900 600]);
vars   = {-X(:,3), XHAT(:,3)};     % altitude
labels = {'Altitude [ft]', 'p_n [ft]', 'p_e [ft]', 'V_a [ft/s]'};

subplot(2,2,1);
plot(t, -X(:,3),'b-','LineWidth',LW,'DisplayName','True h'); hold on;
plot(t, XHAT(:,3),'r--','LineWidth',LW,'DisplayName','Est h');
ylabel(labels{1},'FontSize',FS); legend; grid on; title('Altitude (Sec 8.10)','FontSize',FS);

subplot(2,2,2);
plot(t, X(:,1),'b-','LineWidth',LW,'DisplayName','True p_n'); hold on;
plot(t, XHAT(:,1),'r--','LineWidth',LW,'DisplayName','Est p_n');
ylabel(labels{2},'FontSize',FS); legend; grid on; title('North Position','FontSize',FS);

subplot(2,2,3);
plot(t, X(:,2),'b-','LineWidth',LW,'DisplayName','True p_e'); hold on;
plot(t, XHAT(:,2),'r--','LineWidth',LW,'DisplayName','Est p_e');
ylabel(labels{3},'FontSize',FS); xlabel('Time [s]','FontSize',FS);
legend; grid on; title('East Position','FontSize',FS);

subplot(2,2,4);
% True Va from forces_moments
Va_true = arrayfun(@(k) forces_moments(X(k,:)', u_trim, wind, P), 1:N, 'UniformOutput',false);
Va_true = cellfun(@(f) f(7), Va_true)';
plot(t, Va_true,'b-','LineWidth',LW,'DisplayName','True V_a'); hold on;
plot(t, XHAT(:,4),'r--','LineWidth',LW,'DisplayName','Est V_a');
ylabel(labels{4},'FontSize',FS); xlabel('Time [s]','FontSize',FS);
legend; grid on; title('Airspeed','FontSize',FS);

fprintf('Section 8 complete.\n');
