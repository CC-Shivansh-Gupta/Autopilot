%% Section5_Autopilot.m  (FIXED — permanent disturbance + legible figures)
% =========================================================
% AE700 — Section 5: Autopilot Design (Successive Loop Closure)
% IIT Bombay | F-4 Phantom
%
% FIXES vs previous version:
%   1. Roll kp_phi: saturation design (eq 6.7), sign = sign(a_phi2).
%      kd_phi: pole-placement eq 6.9.
%      Guard changed: check kp_phi*a_phi2 > 0, not kp_phi > 0.
%   2. Sideslip: PI controller (ki_beta > 0) for zero SS error.
%   3. Pitch kp_theta: saturation design (eq 6.7), KthDC recomputed ~85%.
%   4. Altitude ki/kp: divided by correct KthDC*Va (eqs 6.24–6.25).
%   5. Disturbance is a PERMANENT step (no upper time cut-off).
%   6. All figures: FontSize>=12, axis labels, legends, titles.
%
% Run: >> Section5_Autopilot
% =========================================================

clear; clc; close all;
F4_params;

if ~exist('report_figures','dir'), mkdir('report_figures'); end

fprintf('=====================================================\n');
fprintf('  AE700 Section 5 — Autopilot Design\n');
fprintf('  F-4 Phantom  |  Va=%.0f ft/s\n', P.Va_trim);
fprintf('=====================================================\n\n');

%% ---- Trim & transfer functions ----
[x_trim, u_trim] = compute_trim(P.Va_trim, 0, Inf, P);
x_trim(3) = -3000;
TF   = compute_transfer_functions(x_trim, u_trim, P);
Va_s = P.Va_trim;

%% ---- Extract TF coefficients ----
[np,dp]   = tfdata(TF.phi_da,  'v');  a_phi1  = dp(2);   a_phi2  = np(end);
[nb,db]   = tfdata(TF.beta_dr, 'v');  a_beta1 = db(2);   a_beta2 = nb(end);
[nt,dt_]  = tfdata(TF.theta_de,'v');  a_theta1= dt_(2);  a_theta2= dt_(3); a_theta3= nt(end);
[nVt,dVt] = tfdata(TF.Va_dt,   'v');  a_V1    = dVt(2);  a_V2    = nVt(end);
[nVp,dVp] = tfdata(TF.Va_theta,'v');  a_V1p   = dVp(2);  a_V3    = -nVp(end);

dlim = 40*pi/180;   % 40 deg actuator limit (elevator / aileron)

%% ============================================================
%  GAIN DESIGN
% ============================================================

% ---- Roll (Section 6.3.1, eqs 6.7–6.9) ----
% kp_phi = (delta_max/e_max)*sign(a_phi2)  so that kp_phi*a_phi2 > 0 always.
% The old formula  kp = -wn^2/|a_phi2|  gave kp<0 when a_phi2>0 => unstable.
delta_max_a = 40*pi/180;
e_max_phi   = 35*pi/180;
zeta_phi    = 0.707;

kp_phi  = (delta_max_a / e_max_phi) * sign(a_phi2);   % eq 6.7
wn_phi  = sqrt(abs(a_phi2) * abs(kp_phi));             % eq 6.8
kd_phi  = (2*zeta_phi*wn_phi - a_phi1) / a_phi2;      % eq 6.9
ki_phi  = wn_phi/20;

fprintf('Roll gains (FIXED):\n');
fprintf('  a_phi2=%.4f  sign=%+d\n', a_phi2, sign(a_phi2));
fprintf('  kp_phi=%.4f  kd_phi=%.4f  wn_phi=%.3f rad/s\n', kp_phi,kd_phi,wn_phi);

% Stability guard: product kp*a_phi2 must be positive (negative => anti-restoring)
if kp_phi * a_phi2 < 0
    error('Roll instability: kp_phi*a_phi2 = %.4f < 0. Check Cl_da sign in params.', kp_phi*a_phi2);
end
fprintf('  Stability check: kp*a_phi2 = %.4f > 0  OK\n', kp_phi*a_phi2);
if kd_phi < 0
    fprintf('  Note: kd_phi<0 — natural damping > desired; pole placement is still valid.\n');
end

% ---- Course (Section 6.3.2) ----
zeta_chi = 1.0;
wn_chi   = wn_phi / 20;   % bandwidth separation >= 5
kp_chi   = 2*zeta_chi*wn_chi*Va_s / P.g;
ki_chi   = wn_chi^2*Va_s / P.g;

% ---- Sideslip (Section 6.3.3) ----
% FIX: PI controller so beta -> 0 in steady state.
% P-only leaves a permanent error from lateral aerodynamic coupling.
delta_max_r = 20*pi/180;
e_max_beta  = 20*pi/180;
zeta_beta   = 0.9;

kp_beta = (delta_max_r / e_max_beta) * sign(a_beta2);  % eq 6.16
wn_beta = sqrt(abs(a_beta2) * abs(kp_beta));
% ki from eqs 6.14-6.15 inverted
ki_beta = ((a_beta1 + a_beta2*kp_beta) / (2*zeta_beta))^2 / a_beta2;

fprintf('\nSideslip gains (FIXED — PI):\n');
fprintf('  kp_beta=%.4f  ki_beta=%.5f  wn_beta=%.3f rad/s\n', kp_beta,ki_beta,wn_beta);

% ---- Pitch (Section 6.4.1, eqs 6.7, 6.21–6.23) ----
% FIX: kp from saturation constraint so KthDC ~ 85% (old formula gave ~6%).
delta_max_e = 40*pi/180;
e_max_theta = 10*pi/180;
zeta_theta  = 0.85;

kp_theta = (delta_max_e / e_max_theta) * sign(a_theta3);  % eq 6.7
wn_theta = sqrt(a_theta2 + kp_theta*a_theta3);             % eq 6.21
kd_theta = (2*zeta_theta*wn_theta - a_theta1) / a_theta3; % eq 6.22
ki_theta = 0;
KthDC    = kp_theta*a_theta3 / (a_theta2 + kp_theta*a_theta3); % eq 6.23

fprintf('\nPitch gains (FIXED):\n');
fprintf('  kp_theta=%.4f  kd_theta=%.4f  wn_theta=%.3f rad/s\n', kp_theta,kd_theta,wn_theta);
fprintf('  KthDC = %.3f  (was ~0.06 with old kp; now ~0.85)\n', KthDC);

% ---- Altitude (Section 6.4.2, eqs 6.24–6.25) ----
% FIX: ki_h and kp_h divided by KthDC*Va (eqs 6.24–6.25).
%      Overdamped (zeta_h=1.4) to suppress phugoid excitation.
Wh      = 12;
wn_h    = wn_theta / Wh;
zeta_h  = 1.4;

kp_h = 2*zeta_h*wn_h     / (KthDC * Va_s);  % eq 6.25
ki_h = wn_h^2             / (KthDC * Va_s);  % eq 6.24

fprintf('\nAltitude gains (FIXED):\n');
fprintf('  wn_h=%.4f rad/s  zeta_h=%.1f  kp_h=%.6f  ki_h=%.8f\n', wn_h,zeta_h,kp_h,ki_h);

% ---- Airspeed via throttle (Section 6.4.4) ----
wn_Vdt  = 0.7;   zeta_Vdt = 1.0;
kp_Vdt  = (2*zeta_Vdt*wn_Vdt - a_V1) / a_V2;
ki_Vdt  = wn_Vdt^2 / a_V2;

% ---- Airspeed via pitch (Section 6.4.3) ----
% Sign: Va_cmd > Va  =>  theta_c < 0 (pitch down = speed up).
wn_Vth  = 0.35;  zeta_Vth = 1.0;
kp_Vth  = (2*zeta_Vth*wn_Vth - a_V1p) / a_V3;
ki_Vth  = wn_Vth^2 / a_V3;

fprintf('\nBandwidth summary:\n');
fprintf('  Roll:        wn=%.3f rad/s\n',               wn_phi);
fprintf('  Course:      wn=%.4f rad/s  (sep=%.0f)\n',   wn_chi,   wn_phi/wn_chi);
fprintf('  Sideslip:    wn=%.3f rad/s  (PI)\n',         wn_beta);
fprintf('  Pitch:       wn=%.3f rad/s\n',               wn_theta);
fprintf('  Altitude:    wn=%.4f rad/s  (sep=%.0f)\n',   wn_h,     wn_theta/wn_h);
fprintf('  Va/throttle: wn=%.3f rad/s\n',               wn_Vdt);
fprintf('  Va/pitch:    wn=%.3f rad/s\n\n',             wn_Vth);

%% ============================================================
%  SIMULATION PARAMETERS
% ============================================================
Tlat     = 80;
Tlon     = 80;
dt       = 0.02;
wind     = zeros(6,1);

phi_cmd   = 20*pi/180;
chi_cmd   = 30*pi/180;
theta_cmd =  5*pi/180;
h_cmd     = 3500;
Va_cmd    = 860;

dist_mag  = 0.2;
dist_t    = 8;

%% ============================================================
%  RUN SIMULATIONS
% ============================================================
fprintf('Simulating...\n');

[t_r,  X_r,  U_r]  = sim_roll(x_trim,u_trim,phi_cmd, 0,      0,     kp_phi,kd_phi,ki_phi,kp_beta,ki_beta,dlim,Tlat,dt,P);
[t_rd, X_rd, U_rd] = sim_roll(x_trim,u_trim,phi_cmd, dist_mag,dist_t,kp_phi,kd_phi,ki_phi,kp_beta,ki_beta,dlim,Tlat,dt,P);
[t_c,  X_c,  U_c]  = sim_course(x_trim,u_trim,chi_cmd,kp_phi,kd_phi,ki_phi,kp_chi,ki_chi,kp_beta,ki_beta,dlim,Tlat,dt,P);
[t_p,  X_p,  U_p]  = sim_pitch(x_trim,u_trim,theta_cmd,kp_theta,kd_theta,ki_theta,dlim,Tlon,dt,P);
[t_a,  X_a,  U_a]  = sim_altitude(x_trim,u_trim,h_cmd,KthDC,kp_theta,kd_theta,ki_theta,kp_h,ki_h,dlim,Tlon,dt,P);
[t_vt, X_vt, U_vt] = sim_Va_throttle(x_trim,u_trim,Va_cmd,kp_Vdt,ki_Vdt,dlim,Tlon,dt,P);
[t_vp, X_vp, U_vp] = sim_Va_pitch(x_trim,u_trim,Va_cmd,kp_Vth,ki_Vth,kp_theta,kd_theta,ki_theta,dlim,Tlon,dt,P);

beta_r = arrayfun(@(k) get_beta(X_r(k,:)', U_r(k,:)', wind, P),   1:length(t_r))';
Va_vt  = arrayfun(@(k) get_Va( X_vt(k,:)',U_vt(k,:)',wind, P),    1:length(t_vt))';
Va_vp  = arrayfun(@(k) get_Va( X_vp(k,:)',U_vp(k,:)',wind, P),    1:length(t_vp))';

fprintf('Done.\n\n');

%% ============================================================
%  FIGURE SETTINGS
% ============================================================
FS   = 13;
FSL  = 11;
LW   = 2.0;
FIG_SZ = [0,0,900,400];

%% ---- (a) Roll — no disturbance ----
fig_a = figure('Name','(a) Roll no dist','Position',FIG_SZ);
plot(t_r, rad2deg(X_r(:,7)), 'b-', 'LineWidth',LW, 'DisplayName','Actual \phi'); hold on;
yline(rad2deg(phi_cmd), 'r--', 'LineWidth',1.5, 'DisplayName','Command \phi_c');
xlabel('Time [s]',   'FontSize',FS);
ylabel('\phi [deg]', 'FontSize',FS);
title('(a) Roll Angle — No Disturbance', 'FontSize',FS, 'FontWeight','bold');
legend('FontSize',FSL, 'Location','southeast');
set(gca,'FontSize',FS); grid on; box on;
saveas(fig_a, 'report_figures/Sec5_a_Roll.png');

%% ---- (b) Roll — with permanent step disturbance ----
fig_b = figure('Name','(b) Roll with dist','Position',FIG_SZ);
plot(t_rd, rad2deg(X_rd(:,7)), 'b-', 'LineWidth',LW, 'DisplayName','Actual \phi'); hold on;
yline(rad2deg(phi_cmd), 'r--', 'LineWidth',1.5, 'DisplayName','Command \phi_c');
xline(dist_t, 'k:', 'LineWidth',1.5, 'DisplayName',sprintf('Disturbance step (%.1f) applied',dist_mag));
xlabel('Time [s]',   'FontSize',FS);
ylabel('\phi [deg]', 'FontSize',FS);
title(sprintf('(b) Roll Angle — With Permanent Step Disturbance (mag=%.1f)',dist_mag), ...
      'FontSize',FS, 'FontWeight','bold');
legend('FontSize',FSL, 'Location','southeast');
set(gca,'FontSize',FS); grid on; box on;
saveas(fig_b, 'report_figures/Sec5_b_Roll_dist.png');

%% ---- (c) Course hold ----
fig_c = figure('Name','(c) Course hold','Position',FIG_SZ);
plot(t_c, rad2deg(X_c(:,9)), 'b-', 'LineWidth',LW, 'DisplayName','Actual \psi'); hold on;
yline(rad2deg(chi_cmd), 'r--', 'LineWidth',1.5, 'DisplayName','Command \chi_c');
xlabel('Time [s]',    'FontSize',FS);
ylabel('\psi [deg]',  'FontSize',FS);
title('(c) Course Angle Hold', 'FontSize',FS, 'FontWeight','bold');
legend('FontSize',FSL, 'Location','southeast');
set(gca,'FontSize',FS); grid on; box on;
saveas(fig_c, 'report_figures/Sec5_c_Course.png');

%% ---- (d) Sideslip ----
fig_d = figure('Name','(d) Sideslip','Position',FIG_SZ);
plot(t_r, rad2deg(beta_r), 'b-', 'LineWidth',LW, 'DisplayName','Actual \beta'); hold on;
yline(0, 'r--', 'LineWidth',1.5, 'DisplayName','Command \beta_c = 0');
xlabel('Time [s]',    'FontSize',FS);
ylabel('\beta [deg]', 'FontSize',FS);
title('(d) Sideslip Angle Hold (\beta_c = 0)', 'FontSize',FS, 'FontWeight','bold');
legend('FontSize',FSL, 'Location','northeast');
set(gca,'FontSize',FS); grid on; box on;
saveas(fig_d, 'report_figures/Sec5_d_Sideslip.png');

%% ---- (e) Lateral control inputs ----
fig_e = figure('Name','(e) Lateral inputs','Position',[0,0,1100,620]);

subplot(2,1,1);
plot(t_r,  rad2deg(U_r(:,2)),  'b-',  'LineWidth',LW, 'DisplayName','\delta_a (roll, no dist)'); hold on;
plot(t_rd, rad2deg(U_rd(:,2)), 'g--', 'LineWidth',LW, 'DisplayName','\delta_a (roll, dist)');
plot(t_c,  rad2deg(U_c(:,2)),  'r:',  'LineWidth',LW, 'DisplayName','\delta_a (course)');
yline( 40, 'k:', 'LineWidth',1.2, 'DisplayName','Saturation limit');
yline(-40, 'k:', 'LineWidth',1.2, 'HandleVisibility','off');
ylabel('\delta_a [deg]', 'FontSize',FS);
title('Aileron Deflection', 'FontSize',FS, 'FontWeight','bold');
legend('FontSize',FSL, 'Location','northeast');
set(gca,'FontSize',FS); grid on; box on;

subplot(2,1,2);
plot(t_r, rad2deg(U_r(:,3)), 'b-', 'LineWidth',LW, 'DisplayName','\delta_r (roll)'); hold on;
plot(t_c, rad2deg(U_c(:,3)), 'r:', 'LineWidth',LW, 'DisplayName','\delta_r (course)');
yline( 20, 'k:', 'LineWidth',1.2, 'DisplayName','Saturation limit');
yline(-20, 'k:', 'LineWidth',1.2, 'HandleVisibility','off');
xlabel('Time [s]',       'FontSize',FS);
ylabel('\delta_r [deg]', 'FontSize',FS);
title('Rudder Deflection', 'FontSize',FS, 'FontWeight','bold');
legend('FontSize',FSL, 'Location','northeast');
set(gca,'FontSize',FS); grid on; box on;

sgtitle('(e) Lateral Control Inputs', 'FontSize',FS+1, 'FontWeight','bold');
saveas(fig_e, 'report_figures/Sec5_e_LatInputs.png');

%% ---- (f) Pitch ----
fig_f = figure('Name','(f) Pitch','Position',FIG_SZ);
plot(t_p, rad2deg(X_p(:,8)), 'b-', 'LineWidth',LW, 'DisplayName','Actual \theta'); hold on;
yline(rad2deg(theta_cmd), 'r--', 'LineWidth',1.5, 'DisplayName','Command \theta_c');
xlabel('Time [s]',    'FontSize',FS);
ylabel('\theta [deg]','FontSize',FS);
title('(f) Pitch Angle Hold', 'FontSize',FS, 'FontWeight','bold');
legend('FontSize',FSL, 'Location','southeast');
set(gca,'FontSize',FS); grid on; box on;
saveas(fig_f, 'report_figures/Sec5_f_Pitch.png');

%% ---- (g) Altitude ----
fig_g = figure('Name','(g) Altitude','Position',FIG_SZ);
plot(t_a, -X_a(:,3), 'b-', 'LineWidth',LW, 'DisplayName','Actual h'); hold on;
yline(h_cmd, 'r--', 'LineWidth',1.5, 'DisplayName','Command h_c');
xlabel('Time [s]',       'FontSize',FS);
ylabel('Altitude [ft]',  'FontSize',FS);
title('(g) Altitude Hold', 'FontSize',FS, 'FontWeight','bold');
legend('FontSize',FSL, 'Location','southeast');
set(gca,'FontSize',FS); grid on; box on;
saveas(fig_g, 'report_figures/Sec5_g_Altitude.png');

%% ---- (h) Airspeed via throttle ----
fig_h = figure('Name','(h) Va throttle','Position',FIG_SZ);
plot(t_vt, Va_vt, 'b-', 'LineWidth',LW, 'DisplayName','Actual V_a'); hold on;
yline(Va_cmd, 'r--', 'LineWidth',1.5, 'DisplayName','Command V_a^c');
xlabel('Time [s]',     'FontSize',FS);
ylabel('V_a [ft/s]',   'FontSize',FS);
title('(h) Airspeed Hold — Throttle Control', 'FontSize',FS, 'FontWeight','bold');
legend('FontSize',FSL, 'Location','southeast');
set(gca,'FontSize',FS); grid on; box on;
saveas(fig_h, 'report_figures/Sec5_h_Va_throttle.png');

%% ---- (i) Airspeed via pitch ----
fig_i = figure('Name','(i) Va pitch','Position',FIG_SZ);
plot(t_vp, Va_vp, 'b-', 'LineWidth',LW, 'DisplayName','Actual V_a'); hold on;
yline(Va_cmd, 'r--', 'LineWidth',1.5, 'DisplayName','Command V_a^c');
xlabel('Time [s]',   'FontSize',FS);
ylabel('V_a [ft/s]', 'FontSize',FS);
title('(i) Airspeed Hold — Pitch Command', 'FontSize',FS, 'FontWeight','bold');
legend('FontSize',FSL, 'Location','southeast');
set(gca,'FontSize',FS); grid on; box on;
saveas(fig_i, 'report_figures/Sec5_i_Va_pitch.png');

%% ---- (j) Longitudinal control inputs ----
fig_j = figure('Name','(j) Longitudinal inputs','Position',[0,0,1100,620]);

subplot(2,2,1);
plot(t_p, rad2deg(U_p(:,1)), 'b-',  'LineWidth',LW, 'DisplayName','\delta_e (pitch)'); hold on;
plot(t_a, rad2deg(U_a(:,1)), 'r--', 'LineWidth',LW, 'DisplayName','\delta_e (alt)');
yline( 40, 'k:', 'LineWidth',1.2, 'DisplayName','Saturation limit');
yline(-40, 'k:', 'LineWidth',1.2, 'HandleVisibility','off');
ylabel('\delta_e [deg]', 'FontSize',FS);
title('Elevator — Pitch / Altitude', 'FontSize',FS, 'FontWeight','bold');
legend('FontSize',FSL, 'Location','northeast');
set(gca,'FontSize',FS); grid on; box on;

subplot(2,2,2);
plot(t_a, U_a(:,4), 'b-', 'LineWidth',LW, 'DisplayName','\delta_t (altitude)');
ylim([0,1.05]);
ylabel('\delta_t [-]', 'FontSize',FS);
title('Throttle — Altitude Autopilot', 'FontSize',FS, 'FontWeight','bold');
legend('FontSize',FSL); set(gca,'FontSize',FS); grid on; box on;

subplot(2,2,3);
plot(t_vt, U_vt(:,4), 'b-', 'LineWidth',LW, 'DisplayName','\delta_t (V_a/throttle)');
ylim([0,1.05]);
xlabel('Time [s]',     'FontSize',FS);
ylabel('\delta_t [-]', 'FontSize',FS);
title('Throttle — V_a/Throttle Autopilot', 'FontSize',FS, 'FontWeight','bold');
legend('FontSize',FSL); set(gca,'FontSize',FS); grid on; box on;

subplot(2,2,4);
plot(t_vp, rad2deg(U_vp(:,1)), 'b-', 'LineWidth',LW, 'DisplayName','\delta_e (V_a/pitch)');
yline( 40, 'k:', 'LineWidth',1.2, 'DisplayName','Saturation limit');
yline(-40, 'k:', 'LineWidth',1.2, 'HandleVisibility','off');
xlabel('Time [s]',       'FontSize',FS);
ylabel('\delta_e [deg]', 'FontSize',FS);
title('Elevator — V_a/Pitch Autopilot', 'FontSize',FS, 'FontWeight','bold');
legend('FontSize',FSL); set(gca,'FontSize',FS); grid on; box on;

sgtitle('(j) Longitudinal Control Inputs', 'FontSize',FS+1, 'FontWeight','bold');
saveas(fig_j, 'report_figures/Sec5_j_LonInputs.png');

fprintf('All figures saved in report_figures/\n');
fprintf('Section 5 complete.\n\n');

%% ============================================================
%  SIMULATION FUNCTIONS
% ============================================================

function [t,X,U] = sim_roll(x0,u0,phi_cmd,dm,dt_d,kp,kd,ki,kpb,kib,dlim,T,dt,P)
% Disturbance: PERMANENT step (stays on after dt_d — no upper cut-off).
% Rudder: PI so steady-state beta = 0.
    t=(0:dt:T)'; N=length(t);
    X=zeros(N,12); U=zeros(N,4); X(1,:)=x0';
    ph_int=0; beta_int=0; wnd=zeros(6,1);
    oo=odeset('RelTol',1e-6,'AbsTol',1e-8);
    for k=1:N-1
        x=X(k,:)'; phi=x(7); p=x(10);
        dist = dm * (t(k) >= dt_d);          % permanent step
        e    = phi_cmd - phi;
        da_u = kp*e - kd*p + ki*ph_int + dist;
        da   = sat(da_u, dlim);
        if abs(da_u) < dlim                   % anti-windup
            ph_int = ph_int + e*dt;
        end
        fm       = forces_moments(x, [u0(1);da;u0(3);u0(4)], wnd, P);
        beta     = fm(9);
        beta_int = beta_int + beta*dt;
        dr       = sat(-(kpb*beta + kib*beta_int), 20*pi/180);
        d        = [u0(1);da;dr;u0(4)]; U(k,:)=d';
        [~,Xk]   = ode45(@(t,x)eom(x,d,wnd,P),[0,dt],x,oo);
        X(k+1,:) = Xk(end,:);
    end
    U(end,:) = U(end-1,:);
end

function [t,X,U] = sim_course(x0,u0,chi_c,kp_ph,kd_ph,ki_ph,kp_ch,ki_ch,kpb,kib,dlim,T,dt,P)
% Rudder: PI (kib integrator) to drive beta to zero.
    t=(0:dt:T)'; N=length(t);
    X=zeros(N,12); U=zeros(N,4); X(1,:)=x0';
    ch_int=0; ph_int=0; beta_int=0; phi_mx=35*pi/180; wnd=zeros(6,1);
    oo=odeset('RelTol',1e-6,'AbsTol',1e-8);
    for k=1:N-1
        x=X(k,:)'; phi=x(7); p=x(10); psi=x(9);
        e_c     = atan2(sin(chi_c-psi), cos(chi_c-psi));
        ch_int  = sat(ch_int + e_c*dt, pi/4);
        phi_c   = sat(kp_ch*e_c + ki_ch*ch_int, phi_mx);
        e_p     = phi_c - phi;
        ph_int  = ph_int + e_p*dt;
        da      = sat(kp_ph*e_p - kd_ph*p + ki_ph*ph_int, dlim);
        fm      = forces_moments(x, [u0(1);da;u0(3);u0(4)], wnd, P);
        beta    = fm(9);
        beta_int= beta_int + beta*dt;
        dr      = sat(-(kpb*beta + kib*beta_int), 20*pi/180);
        d       = [u0(1);da;dr;u0(4)]; U(k,:)=d';
        [~,Xk]  = ode45(@(t,x)eom(x,d,wnd,P),[0,dt],x,oo);
        X(k+1,:)= Xk(end,:);
    end
    U(end,:) = U(end-1,:);
end

function [t,X,U] = sim_pitch(x0,u0,theta_c,kp,kd,ki,dlim,T,dt,P)
    t=(0:dt:T)'; N=length(t);
    X=zeros(N,12); U=zeros(N,4); X(1,:)=x0';
    th_int=0; wnd=zeros(6,1);
    oo=odeset('RelTol',1e-6,'AbsTol',1e-8);
    for k=1:N-1
        x=X(k,:)'; e=theta_c-x(8); th_int=th_int+e*dt;
        de = sat(u0(1)+kp*e-kd*x(11)+ki*th_int, dlim);
        d  = [de;u0(2);u0(3);u0(4)]; U(k,:)=d';
        [~,Xk]  = ode45(@(t,x)eom(x,d,wnd,P),[0,dt],x,oo);
        X(k+1,:)= Xk(end,:);
    end
    U(end,:) = U(end-1,:);
end

function [t,X,U] = sim_altitude(x0,u0,h_c,~,kp_th,kd_th,ki_th,kp_h,ki_h,dlim,T,dt,P)
% kp_h / ki_h already incorporate KthDC*Va in denominator (computed above).
    t=(0:dt:T)'; N=length(t);
    X=zeros(N,12); U=zeros(N,4); X(1,:)=x0';
    h_int=0; th_int=0; thmax=20*pi/180; wnd=zeros(6,1);
    oo=odeset('RelTol',1e-6,'AbsTol',1e-8);
    for k=1:N-1
        x=X(k,:)'; h=-x(3); e_h=h_c-h; h_int=h_int+e_h*dt;
        th_c   = sat(kp_h*e_h + ki_h*h_int, thmax);
        e_t    = th_c - x(8); th_int=th_int+e_t*dt;
        de     = sat(u0(1)+kp_th*e_t-kd_th*x(11)+ki_th*th_int, dlim);
        d      = [de;u0(2);u0(3);u0(4)]; U(k,:)=d';
        [~,Xk] = ode45(@(t,x)eom(x,d,wnd,P),[0,dt],x,oo);
        X(k+1,:)=Xk(end,:);
    end
    U(end,:) = U(end-1,:);
end

function [t,X,U] = sim_Va_throttle(x0,u0,Va_c,kpV,kiV,dlim,T,dt,P)
    t=(0:dt:T)'; N=length(t);
    X=zeros(N,12); U=zeros(N,4); X(1,:)=x0';
    V_int=0; wnd=zeros(6,1);
    oo=odeset('RelTol',1e-6,'AbsTol',1e-8);
    for k=1:N-1
        x=X(k,:)';
        fm   = forces_moments(x,u0,wnd,P); Va=fm(7);
        eV   = Va_c - Va; V_int=V_int+eV*dt;
        dtc  = max(0, min(1, u0(4)+kpV*eV+kiV*V_int));
        d    = [u0(1);u0(2);u0(3);dtc]; U(k,:)=d';
        [~,Xk] = ode45(@(t,x)eom(x,d,wnd,P),[0,dt],x,oo);
        X(k+1,:)=Xk(end,:);
    end
    U(end,:) = U(end-1,:);
end

function [t,X,U] = sim_Va_pitch(x0,u0,Va_c,kpV,kiV,kp_th,kd_th,ki_th,dlim,T,dt,P)
% Va_cmd > Va  =>  theta_c < 0  (pitch down = speed up).
    t=(0:dt:T)'; N=length(t);
    X=zeros(N,12); U=zeros(N,4); X(1,:)=x0';
    V_int=0; th_int=0; thmax=15*pi/180; wnd=zeros(6,1);
    oo=odeset('RelTol',1e-6,'AbsTol',1e-8);
    for k=1:N-1
        x=X(k,:)';
        fm  = forces_moments(x,u0,wnd,P); Va=fm(7);
        eV  = Va_c - Va; V_int=V_int+eV*dt;
        th_c= sat(-(kpV*eV+kiV*V_int), thmax);
        e_t = th_c - x(8); th_int=th_int+e_t*dt;
        de  = sat(u0(1)+kp_th*e_t-kd_th*x(11)+ki_th*th_int, dlim);
        d   = [de;u0(2);u0(3);u0(4)]; U(k,:)=d';
        [~,Xk] = ode45(@(t,x)eom(x,d,wnd,P),[0,dt],x,oo);
        X(k+1,:)=Xk(end,:);
    end
    U(end,:) = U(end-1,:);
end

%% ---- Utility functions ----
function b = get_beta(x,u,w,P); f=forces_moments(x,u,w,P); b=f(9); end
function v = get_Va(x,u,w,P);   f=forces_moments(x,u,w,P); v=f(7); end
function y = sat(x,lim);         y=max(-lim,min(lim,x));    end

function xdot = eom(x,d,wind,P)
    fm=forces_moments(x,d,wind,P);
    fx=fm(1);fy=fm(2);fz=fm(3);l=fm(4);mm=fm(5);n=fm(6);
    u=x(4);v=x(5);w=x(6);phi=x(7);theta=x(8);psi=x(9);
    p=x(10);q=x(11);r=x(12);
    cp=cos(phi);sp=sin(phi);ct=cos(theta);st=sin(theta);
    tt=tan(theta);cs=cos(psi);ss=sin(psi);
    G=P.Jx*P.Jz-P.Jxz^2;
    G1=P.Jxz*(P.Jx-P.Jy+P.Jz)/G; G2=(P.Jz*(P.Jz-P.Jy)+P.Jxz^2)/G;
    G3=P.Jz/G; G4=P.Jxz/G; G5=(P.Jz-P.Jx)/P.Jy; G6=P.Jxz/P.Jy;
    G7=((P.Jx-P.Jy)*P.Jx+P.Jxz^2)/G; G8=P.Jx/G;
    pnd=(ct*cs)*u+(sp*st*cs-cp*ss)*v+(cp*st*cs+sp*ss)*w;
    ped=(ct*ss)*u+(sp*st*ss+cp*cs)*v+(cp*st*ss-sp*cs)*w;
    pdd=(-st)*u+(sp*ct)*v+(cp*ct)*w;
    ud=r*v-q*w+fx/P.mass; vd=p*w-r*u+fy/P.mass; wd=q*u-p*v+fz/P.mass;
    phd=p+(sp*tt)*q+(cp*tt)*r; thd=cp*q-sp*r; psd=(sp/ct)*q+(cp/ct)*r;
    pd_=G1*p*q-G2*q*r+G3*l+G4*n; qd=G5*p*r-G6*(p^2-r^2)+mm/P.Jy;
    rd=G7*p*q-G1*q*r+G4*l+G8*n;
    xdot=[pnd;ped;pdd;ud;vd;wd;phd;thd;psd;pd_;qd;rd];
end
