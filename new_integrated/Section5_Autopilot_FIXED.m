%% Section5_Autopilot_FIXED.m
% AE700 - Section 5: Autopilot Design using Successive Loop Closure
% IIT Bombay | F-4 Phantom
%
% FIXES vs previous version:
%   1. Disturbance is a PERMANENT STEP (not a 0.5s pulse) per project spec
%   2. Disturbance magnitude = 0.2 rad as specified
%   3. All commanded values are step signals of suitable amplitude
%   4. Bandwidth separation properly enforced
%   5. All 9 required plots generated with labels, legends, titles, fontsize
%
% Run: >> Section5_Autopilot_FIXED

clear; clc; close all;
F4_chap4_params;   % loads P

fprintf('=====================================================\n');
fprintf('  AE700 Section 5 - Autopilot Design (FIXED)\n');
fprintf('  F-4 Phantom  Va=%.0f ft/s\n', P.Va_trim);
fprintf('=====================================================\n\n');

if ~exist('report_figures','dir'), mkdir('report_figures'); end

%% ---- Trim ----
[x_trim, u_trim] = compute_trim(P.Va_trim, 0, Inf, P);
x_trim(3) = -3000;   % 3000 ft altitude

TF = compute_transfer_functions(x_trim, u_trim, P);
Va_s = P.Va_trim;

%% ---- Extract TF coefficients ----
[num_phi, den_phi] = tfdata(TF.phi_da,  'v');
a_phi1 = den_phi(2);
a_phi2 = num_phi(end);

[num_beta, den_beta] = tfdata(TF.beta_dr,'v');
a_beta1 = den_beta(2);
a_beta2 = num_beta(end);

[num_th, den_th] = tfdata(TF.theta_de,'v');
a_theta1 = den_th(2);
a_theta2 = den_th(3);
a_theta3 = num_th(end);

[num_Vdt, den_Vdt] = tfdata(TF.Va_dt,'v');
a_V1 = den_Vdt(2);
a_V2 = num_Vdt(end);

[num_Vth, den_Vth] = tfdata(TF.Va_theta,'v');
a_V1_p = den_Vth(2);
a_V3   = -num_Vth(end);   % Va/theta: negative because nose-down->speed up

fprintf('TF coefficients:\n');
fprintf('  a_phi1=%.4f  a_phi2=%.5f\n', a_phi1, a_phi2);
fprintf('  a_theta1=%.4f  a_theta2=%.4f  a_theta3=%.5f\n',a_theta1,a_theta2,a_theta3);
fprintf('  a_V1=%.5f  a_V2=%.5f  a_V3=%.4f\n\n',a_V1,a_V2,a_V3);

delta_lim = 40*pi/180;   % 40 deg surface limit

%% ============================================================
%  GAIN DESIGN  (Successive Loop Closure)
%  Bandwidth rules:
%    wn_chi = wn_phi / W_chi  (W_chi >= 5)
%    wn_h   = wn_theta / W_h  (W_h >= 5)
%    wn_V   = wn_theta / W_V  (W_V >= 5)
% ============================================================

% ---- LATERAL ----
% Inner: roll
% kp_phi set from saturation: de_max / e_max_phi * sign(a_phi2)
e_max_phi = 15*pi/180;   % 15 deg max roll error before saturation
kp_phi    = (delta_lim / e_max_phi) * sign(a_phi2);
wn_phi    = sqrt(abs(a_phi2) * delta_lim / e_max_phi);
zeta_phi  = 0.707;
kd_phi    = (2*zeta_phi*wn_phi - a_phi1) / a_phi2;
ki_phi    = 0.2;   % small integrator to reject constant disturbances

% Outer: course
W_chi  = 8;
wn_chi = wn_phi / W_chi;
zeta_chi = 1.0;
kp_chi = 2*zeta_chi*wn_chi * Va_s / P.g;
ki_chi = wn_chi^2 * Va_s / P.g;

% Sideslip: P-only (F-4 natural weathercock handles most of it)
kp_beta = 0.8;
ki_beta = 0;

% ---- LONGITUDINAL ----
% Inner: pitch
e_max_theta = 10*pi/180;   % 10 deg max pitch error
kp_theta    = (delta_lim / e_max_theta) * sign(a_theta3);
wn_theta    = sqrt(abs(a_theta2) + abs(a_theta3)*delta_lim/e_max_theta);
zeta_theta  = 0.9;
kd_theta    = (2*zeta_theta*wn_theta - a_theta1) / a_theta3;
ki_theta    = 0;
KthDC       = kp_theta*a_theta3 / (a_theta2 + kp_theta*a_theta3);

% Altitude: outer (uses pitch inner loop)
W_h    = 10;
wn_h   = wn_theta / W_h;
zeta_h = 1.2;
ki_h   = wn_h^2 / (KthDC * Va_s);
kp_h   = 2*zeta_h*wn_h / (KthDC * Va_s);

% Airspeed via throttle
W_V    = 6;
wn_V_dt   = wn_theta / W_V;
zeta_V_dt = 1.0;
kp_V_dt = (2*zeta_V_dt*wn_V_dt - a_V1) / a_V2;
ki_V_dt = wn_V_dt^2 / a_V2;

% Airspeed via pitch
wn_V_th   = wn_theta / W_V;
zeta_V_th = 1.0;
kp_V_th = (a_V1_p - 2*zeta_V_th*wn_V_th) / (KthDC * P.g);
ki_V_th = -wn_V_th^2 / (KthDC * P.g);

fprintf('Gains (Successive Loop Closure):\n');
fprintf('  Roll:     kp=%+.4f  kd=%+.4f  ki=%.4f  wn=%.3f rad/s\n',kp_phi,kd_phi,ki_phi,wn_phi);
fprintf('  Course:   kp=%+.4f  ki=%+.6f  wn=%.4f rad/s  (W_chi=%d)\n',kp_chi,ki_chi,wn_chi,W_chi);
fprintf('  Sideslip: kp=%+.4f  (P-only)\n',kp_beta);
fprintf('  Pitch:    kp=%+.4f  kd=%+.4f  wn=%.3f rad/s  KthDC=%.3f\n',kp_theta,kd_theta,wn_theta,KthDC);
fprintf('  Altitude: kp=%+.6f  ki=%+.8f  wn=%.4f rad/s  (W_h=%d)\n',kp_h,ki_h,wn_h,W_h);
fprintf('  Va/thr:   kp=%+.4f  ki=%+.4f  wn=%.3f rad/s\n',kp_V_dt,ki_V_dt,wn_V_dt);
fprintf('  Va/ptch:  kp=%+.4f  ki=%+.4f  wn=%.3f rad/s\n\n',kp_V_th,ki_V_th,wn_V_th);

%% ---- Commanded values (step signals) ----
phi_cmd   = 25*pi/180;    % 25 deg roll step
chi_cmd   = 30*pi/180;    % 30 deg course step
theta_cmd = 5*pi/180;     % 5 deg pitch step
h_cmd     = 3500;         % 3500 ft altitude step
Va_cmd    = P.Va_trim + 30;  % +30 ft/s airspeed step
dist_mag  = 0.2;          % 0.2 rad = project specification
dist_t    = 5;            % disturbance applied at t=5s, PERMANENT (step)

Tlat = 50;    % lateral sim time
Tlon = 80;    % longitudinal sim time
wind = zeros(6,1);

%% ---- Run simulations ----
fprintf('Running simulations...\n');
[t_r,  X_r,  U_r]  = sim_roll(x_trim,u_trim,phi_cmd,0,0,...
    kp_phi,kd_phi,ki_phi,kp_beta,delta_lim,Tlat,P);
[t_rd, X_rd, U_rd] = sim_roll(x_trim,u_trim,phi_cmd,dist_mag,dist_t,...
    kp_phi,kd_phi,ki_phi,kp_beta,delta_lim,Tlat,P);
[t_c,  X_c,  U_c]  = sim_course(x_trim,u_trim,chi_cmd,...
    kp_phi,kd_phi,ki_phi,kp_chi,ki_chi,kp_beta,delta_lim,Tlat,P);
[t_p,  X_p,  U_p]  = sim_pitch(x_trim,u_trim,theta_cmd,...
    kp_theta,kd_theta,ki_theta,delta_lim,Tlon,P);
[t_a,  X_a,  U_a]  = sim_altitude(x_trim,u_trim,h_cmd,...
    kp_theta,kd_theta,ki_theta,kp_h,ki_h,delta_lim,Tlon,P);
[t_vt, X_vt, U_vt] = sim_Va_throttle(x_trim,u_trim,Va_cmd,...
    kp_V_dt,ki_V_dt,kp_theta,kd_theta,ki_theta,kp_h,ki_h,delta_lim,Tlon,P);
[t_vp, X_vp, U_vp] = sim_Va_pitch(x_trim,u_trim,Va_cmd,...
    kp_V_th,ki_V_th,kp_theta,kd_theta,ki_theta,delta_lim,Tlon,P);
fprintf('Simulations complete.\n\n');

%% ---- Compute secondary quantities ----
beta_r  = arrayfun(@(k) get_beta(X_r(k,:)',u_trim,wind,P),  1:length(t_r))';
Va_vt   = arrayfun(@(k) get_Va(X_vt(k,:)',U_vt(k,:)',wind,P), 1:length(t_vt))';
Va_vp   = arrayfun(@(k) get_Va(X_vp(k,:)',U_vp(k,:)',wind,P), 1:length(t_vp))';

%% ============================================================
%  PLOTS — all with label, legend, title, fontsize 12
%  Exactly matching Section 5 required list
% ============================================================
fs = 12;   % font size
fw = 'bold';
lw = 2;

% (a) Roll angle — no disturbance
fig_a = figure('Name','(a) Roll - No Disturbance','Position',[10,600,700,380]);
plot(t_r, rad2deg(X_r(:,7)), 'b-', 'LineWidth',lw, 'DisplayName','\phi actual'); hold on;
yline(rad2deg(phi_cmd),'r--','LineWidth',1.5,'DisplayName','\phi_{cmd}');
xlabel('Time [s]','FontSize',fs); ylabel('\phi [deg]','FontSize',fs);
title('(a) Roll Angle — No Disturbance','FontSize',fs,'FontWeight',fw);
legend('FontSize',fs,'Location','southeast'); grid on; box on;
saveas(fig_a,'report_figures/Sec5_a_RollNoDist.png');

% (b) Roll angle — with PERMANENT step disturbance (0.2 rad)
fig_b = figure('Name','(b) Roll - With Disturbance','Position',[720,600,700,380]);
plot(t_rd, rad2deg(X_rd(:,7)),'b-','LineWidth',lw,'DisplayName','\phi actual'); hold on;
yline(rad2deg(phi_cmd),'r--','LineWidth',1.5,'DisplayName','\phi_{cmd}');
xline(dist_t,'k:','LineWidth',1.5,'DisplayName',sprintf('Step dist (%.1f rad)',dist_mag));
xlabel('Time [s]','FontSize',fs); ylabel('\phi [deg]','FontSize',fs);
title('(b) Roll Angle — With Permanent Step Disturbance (0.2 rad)','FontSize',fs,'FontWeight',fw);
legend('FontSize',fs,'Location','southeast'); grid on; box on;
saveas(fig_b,'report_figures/Sec5_b_RollDist.png');

% (c) Course angle
fig_c = figure('Name','(c) Course Angle','Position',[10,160,700,380]);
plot(t_c, rad2deg(X_c(:,9)),'b-','LineWidth',lw,'DisplayName','\chi actual'); hold on;
yline(rad2deg(chi_cmd),'r--','LineWidth',1.5,'DisplayName','\chi_{cmd}');
xlabel('Time [s]','FontSize',fs); ylabel('\chi [deg]','FontSize',fs);
title('(c) Course Angle','FontSize',fs,'FontWeight',fw);
legend('FontSize',fs,'Location','southeast'); grid on; box on;
saveas(fig_c,'report_figures/Sec5_c_Course.png');

% (d) Sideslip angle
fig_d = figure('Name','(d) Sideslip Angle','Position',[720,160,700,380]);
plot(t_r, rad2deg(beta_r),'b-','LineWidth',lw,'DisplayName','\beta actual'); hold on;
yline(0,'r--','LineWidth',1.5,'DisplayName','\beta_{cmd} = 0');
xlabel('Time [s]','FontSize',fs); ylabel('\beta [deg]','FontSize',fs);
title('(d) Sideslip Angle','FontSize',fs,'FontWeight',fw);
legend('FontSize',fs,'Location','best'); grid on; box on;
saveas(fig_d,'report_figures/Sec5_d_Sideslip.png');

% (e) Lateral control inputs
fig_e = figure('Name','(e) Lateral Control Inputs','Position',[10,30,1400,500]);
subplot(2,1,1);
plot(t_r, rad2deg(U_r(:,2)),'b-','LineWidth',lw,'DisplayName','\delta_a (roll, no dist)'); hold on;
plot(t_rd,rad2deg(U_rd(:,2)),'g--','LineWidth',lw,'DisplayName','\delta_a (roll, dist)');
plot(t_c, rad2deg(U_c(:,2)),'r:','LineWidth',lw,'DisplayName','\delta_a (course)');
yline( 40,'k:','LineWidth',1,'HandleVisibility','off');
yline(-40,'k:','LineWidth',1,'HandleVisibility','off');
ylabel('\delta_a [deg]','FontSize',fs);
title('Aileron Deflection','FontSize',fs,'FontWeight',fw);
legend('FontSize',fs-1,'Location','northeast'); grid on; box on;
subplot(2,1,2);
plot(t_r, rad2deg(U_r(:,3)),'b-','LineWidth',lw,'DisplayName','\delta_r (roll)'); hold on;
plot(t_c, rad2deg(U_c(:,3)),'r:','LineWidth',lw,'DisplayName','\delta_r (course)');
yline( 20,'k:','LineWidth',1,'HandleVisibility','off');
yline(-20,'k:','LineWidth',1,'HandleVisibility','off');
xlabel('Time [s]','FontSize',fs); ylabel('\delta_r [deg]','FontSize',fs);
title('Rudder Deflection','FontSize',fs,'FontWeight',fw);
legend('FontSize',fs-1,'Location','northeast'); grid on; box on;
sgtitle('(e) Lateral Autopilot Control Inputs','FontSize',fs,'FontWeight',fw);
saveas(fig_e,'report_figures/Sec5_e_LateralInputs.png');

% (f) Pitch angle
fig_f = figure('Name','(f) Pitch Angle','Position',[10,600,700,380]);
plot(t_p, rad2deg(X_p(:,8)),'b-','LineWidth',lw,'DisplayName','\theta actual'); hold on;
yline(rad2deg(theta_cmd),'r--','LineWidth',1.5,'DisplayName','\theta_{cmd}');
xlabel('Time [s]','FontSize',fs); ylabel('\theta [deg]','FontSize',fs);
title('(f) Pitch Angle','FontSize',fs,'FontWeight',fw);
legend('FontSize',fs,'Location','southeast'); grid on; box on;
saveas(fig_f,'report_figures/Sec5_f_Pitch.png');

% (g) Altitude
fig_g = figure('Name','(g) Altitude','Position',[720,600,700,380]);
plot(t_a, -X_a(:,3),'b-','LineWidth',lw,'DisplayName','h actual'); hold on;
yline(h_cmd,'r--','LineWidth',1.5,'DisplayName','h_{cmd}');
xlabel('Time [s]','FontSize',fs); ylabel('Altitude [ft]','FontSize',fs);
title('(g) Altitude Hold','FontSize',fs,'FontWeight',fw);
legend('FontSize',fs,'Location','southeast'); grid on; box on;
saveas(fig_g,'report_figures/Sec5_g_Altitude.png');

% (h) Airspeed via throttle
fig_h = figure('Name','(h) Airspeed via Throttle','Position',[10,160,700,380]);
plot(t_vt, Va_vt,'b-','LineWidth',lw,'DisplayName','V_a actual'); hold on;
yline(Va_cmd,'r--','LineWidth',1.5,'DisplayName','V_{a,cmd}');
xlabel('Time [s]','FontSize',fs); ylabel('V_a [ft/s]','FontSize',fs);
title('(h) Airspeed Hold — Throttle Control','FontSize',fs,'FontWeight',fw);
legend('FontSize',fs,'Location','southeast'); grid on; box on;
saveas(fig_h,'report_figures/Sec5_h_Va_throttle.png');

% (i) Airspeed via pitch
fig_i = figure('Name','(i) Airspeed via Pitch','Position',[720,160,700,380]);
plot(t_vp, Va_vp,'b-','LineWidth',lw,'DisplayName','V_a actual'); hold on;
yline(Va_cmd,'r--','LineWidth',1.5,'DisplayName','V_{a,cmd}');
xlabel('Time [s]','FontSize',fs); ylabel('V_a [ft/s]','FontSize',fs);
title('(i) Airspeed Hold — Pitch Command','FontSize',fs,'FontWeight',fw);
legend('FontSize',fs,'Location','southeast'); grid on; box on;
saveas(fig_i,'report_figures/Sec5_i_Va_pitch.png');

% (j) Longitudinal control inputs
fig_j = figure('Name','(j) Longitudinal Control Inputs','Position',[10,30,1400,550]);
subplot(2,2,1);
plot(t_p, rad2deg(U_p(:,1)),'b-','LineWidth',lw,'DisplayName','\delta_e (pitch)'); hold on;
plot(t_a, rad2deg(U_a(:,1)),'r--','LineWidth',lw,'DisplayName','\delta_e (altitude)');
yline( 40,'k:','LineWidth',1,'HandleVisibility','off');
yline(-40,'k:','LineWidth',1,'HandleVisibility','off');
ylabel('\delta_e [deg]','FontSize',fs); title('Elevator — Pitch / Altitude','FontSize',fs);
legend('FontSize',fs-1,'Location','northeast'); grid on; box on;

subplot(2,2,2);
plot(t_a, U_a(:,4),'b-','LineWidth',lw); ylim([0,1.1]);
ylabel('\delta_t [-]','FontSize',fs); xlabel('Time [s]','FontSize',fs);
title('Throttle — Altitude Autopilot','FontSize',fs); grid on; box on;

subplot(2,2,3);
plot(t_vt, U_vt(:,4),'b-','LineWidth',lw); ylim([0,1.1]);
ylabel('\delta_t [-]','FontSize',fs); xlabel('Time [s]','FontSize',fs);
title('Throttle — V_a / Throttle Autopilot','FontSize',fs); grid on; box on;

subplot(2,2,4);
plot(t_vp, rad2deg(U_vp(:,1)),'b-','LineWidth',lw);
yline( 40,'k:','LineWidth',1);  yline(-40,'k:','LineWidth',1);
ylabel('\delta_e [deg]','FontSize',fs); xlabel('Time [s]','FontSize',fs);
title('Elevator — V_a / Pitch Autopilot','FontSize',fs); grid on; box on;
sgtitle('(j) Longitudinal Autopilot Control Inputs','FontSize',fs,'FontWeight',fw);
saveas(fig_j,'report_figures/Sec5_j_LongitudinalInputs.png');

fprintf('All 9 required plots saved to ./report_figures/\n');
fprintf('Section 5 complete.\n');

%% ============================================================
%  SIMULATION FUNCTIONS
% ============================================================

% ---- Roll hold (inner loop only) ----
function [t,X,U] = sim_roll(x0,u0,phi_cmd,dist_mag,dist_t, ...
                             kp_phi,kd_phi,ki_phi,kp_beta,dlim,Tsim,P)
    wind=zeros(6,1); dt=0.02;
    t=(0:dt:Tsim)'; N=length(t);
    X=zeros(N,12); U=zeros(N,4); X(1,:)=x0';
    phi_int=0;
    oo=odeset('RelTol',1e-6,'AbsTol',1e-8);
    for k=1:N-1
        x=X(k,:)'; phi=x(7); p=x(10);
        % PERMANENT step disturbance (project spec: step signal mag 0.2)
        dist = dist_mag * (t(k) >= dist_t);
        e = phi_cmd - phi;
        phi_int = phi_int + e*dt;
        da = kp_phi*e - kd_phi*p + ki_phi*phi_int + dist;
        da = max(-dlim,min(dlim,da));
        % Rudder: P-only sideslip hold
        fm0 = forces_moments(x,[u0(1);da;u0(3);u0(4)],wind,P);
        beta = fm0(9);
        dr = max(-20*pi/180, min(20*pi/180, -kp_beta*beta));
        d = [u0(1); da; dr; u0(4)]; U(k,:) = d';
        [~,Xk] = ode45(@(~,xx)feom(xx,d,wind,P),[0,dt],x,oo);
        X(k+1,:) = Xk(end,:);
    end
    U(end,:) = U(end-1,:);
end

% ---- Course hold (roll inner + course outer) ----
function [t,X,U] = sim_course(x0,u0,chi_cmd, ...
                               kp_phi,kd_phi,ki_phi,kp_chi,ki_chi,kp_beta,dlim,Tsim,P)
    wind=zeros(6,1); dt=0.02;
    t=(0:dt:Tsim)'; N=length(t);
    X=zeros(N,12); U=zeros(N,4); X(1,:)=x0';
    chi_int=0; phi_int=0;
    phi_max = 35*pi/180;
    oo=odeset('RelTol',1e-6,'AbsTol',1e-8);
    for k=1:N-1
        x=X(k,:)'; phi=x(7); p=x(10); psi=x(9);
        % Wrap course error to [-pi,pi]
        e_chi = atan2(sin(chi_cmd-psi), cos(chi_cmd-psi));
        chi_int = chi_int + e_chi*dt;
        chi_int = max(-pi/3, min(pi/3, chi_int));   % anti-windup
        phi_c = max(-phi_max, min(phi_max, kp_chi*e_chi + ki_chi*chi_int));
        e_phi = phi_c - phi;
        phi_int = phi_int + e_phi*dt;
        da = max(-dlim, min(dlim, kp_phi*e_phi - kd_phi*p + ki_phi*phi_int));
        fm0 = forces_moments(x,[u0(1);da;u0(3);u0(4)],wind,P);
        beta = fm0(9);
        dr = max(-20*pi/180, min(20*pi/180, -kp_beta*beta));
        d = [u0(1); da; dr; u0(4)]; U(k,:) = d';
        [~,Xk] = ode45(@(~,xx)feom(xx,d,wind,P),[0,dt],x,oo);
        X(k+1,:) = Xk(end,:);
    end
    U(end,:) = U(end-1,:);
end

% ---- Pitch hold ----
function [t,X,U] = sim_pitch(x0,u0,theta_cmd,kp,kd,ki,dlim,Tsim,P)
    wind=zeros(6,1); dt=0.02;
    t=(0:dt:Tsim)'; N=length(t);
    X=zeros(N,12); U=zeros(N,4); X(1,:)=x0'; th_int=0;
    oo=odeset('RelTol',1e-6,'AbsTol',1e-8);
    for k=1:N-1
        x=X(k,:)'; e=theta_cmd-x(8); th_int=th_int+e*dt;
        de = max(-dlim,min(dlim, u0(1)+kp*e - kd*x(11) + ki*th_int));
        d=[de;u0(2);u0(3);u0(4)]; U(k,:)=d';
        [~,Xk]=ode45(@(~,xx)feom(xx,d,wind,P),[0,dt],x,oo);
        X(k+1,:)=Xk(end,:);
    end
    U(end,:)=U(end-1,:);
end

% ---- Altitude hold (pitch inner + altitude outer) ----
function [t,X,U] = sim_altitude(x0,u0,h_cmd,kp_th,kd_th,ki_th,kp_h,ki_h,dlim,Tsim,P)
    wind=zeros(6,1); dt=0.02;
    t=(0:dt:Tsim)'; N=length(t);
    X=zeros(N,12); U=zeros(N,4); X(1,:)=x0';
    h_int=0; th_int=0;
    th_max=20*pi/180;
    oo=odeset('RelTol',1e-6,'AbsTol',1e-8);
    for k=1:N-1
        x=X(k,:)'; h=-x(3); theta=x(8); q=x(11);
        e_h = h_cmd - h; h_int = h_int + e_h*dt;
        theta_c = max(-th_max, min(th_max, kp_h*e_h + ki_h*h_int));
        e_t = theta_c - theta; th_int = th_int + e_t*dt;
        de = max(-dlim,min(dlim, u0(1)+kp_th*e_t - kd_th*q + ki_th*th_int));
        d=[de;u0(2);u0(3);u0(4)]; U(k,:)=d';
        [~,Xk]=ode45(@(~,xx)feom(xx,d,wind,P),[0,dt],x,oo);
        X(k+1,:)=Xk(end,:);
    end
    U(end,:)=U(end-1,:);
end

% ---- Airspeed hold via throttle (altitude also held) ----
function [t,X,U] = sim_Va_throttle(x0,u0,Va_cmd, ...
                                    kp_V,ki_V,kp_th,kd_th,ki_th,kp_h,ki_h,dlim,Tsim,P)
    wind=zeros(6,1); dt=0.02;
    t=(0:dt:Tsim)'; N=length(t);
    X=zeros(N,12); U=zeros(N,4); X(1,:)=x0';
    V_int=0; th_int=0; h_int=0;
    h_hold=-x0(3);   % hold initial altitude
    th_max=15*pi/180;
    oo=odeset('RelTol',1e-6,'AbsTol',1e-8);
    for k=1:N-1
        x=X(k,:)';
        fm0=forces_moments(x,u0,wind,P); Va=fm0(7);
        % Va loop -> throttle
        e_V  = Va_cmd - Va; V_int = V_int + e_V*dt;
        dtc  = max(0,min(1, u0(4) + kp_V*e_V + ki_V*V_int));
        % Altitude hold -> pitch -> elevator
        h    = -x(3); e_h = h_hold - h; h_int = h_int + e_h*dt;
        th_c = max(-th_max, min(th_max, kp_h*e_h + ki_h*h_int));
        e_t  = th_c - x(8); th_int = th_int + e_t*dt;
        de   = max(-dlim,min(dlim, u0(1)+kp_th*e_t - kd_th*x(11) + ki_th*th_int));
        d=[de;u0(2);u0(3);dtc]; U(k,:)=d';
        [~,Xk]=ode45(@(~,xx)feom(xx,d,wind,P),[0,dt],x,oo);
        X(k+1,:)=Xk(end,:);
    end
    U(end,:)=U(end-1,:);
end

% ---- Airspeed hold via pitch command ----
function [t,X,U] = sim_Va_pitch(x0,u0,Va_cmd, ...
                                 kp_V,ki_V,kp_th,kd_th,ki_th,dlim,Tsim,P)
    % Physics: nose-down (negative theta) -> speed increases
    % So theta_cmd = -(kp_V*e_V + ki_V*integral)
    wind=zeros(6,1); dt=0.02;
    t=(0:dt:Tsim)'; N=length(t);
    X=zeros(N,12); U=zeros(N,4); X(1,:)=x0';
    V_int=0; th_int=0;
    th_max=15*pi/180;
    oo=odeset('RelTol',1e-6,'AbsTol',1e-8);
    for k=1:N-1
        x=X(k,:)';
        fm0=forces_moments(x,u0,wind,P); Va=fm0(7);
        e_V  = Va_cmd - Va; V_int = V_int + e_V*dt;
        % Pitch down to increase speed: negate the Va error
        th_c = max(-th_max, min(th_max, -(kp_V*e_V + ki_V*V_int)));
        e_t  = th_c - x(8); th_int = th_int + e_t*dt;
        de   = max(-dlim,min(dlim, u0(1)+kp_th*e_t - kd_th*x(11) + ki_th*th_int));
        d=[de;u0(2);u0(3);u0(4)]; U(k,:)=d';
        [~,Xk]=ode45(@(~,xx)feom(xx,d,wind,P),[0,dt],x,oo);
        X(k+1,:)=Xk(end,:);
    end
    U(end,:)=U(end-1,:);
end

%% ---- Helper functions ----
function b = get_beta(x,u,wind,P)
    fm = forces_moments(x,u,wind,P); b=fm(9);
end
function v = get_Va(x,u,wind,P)
    fm = forces_moments(x,u,wind,P); v=fm(7);
end

function xdot = feom(x,d,wind,P)
    fm=forces_moments(x,d,wind,P);
    fx=fm(1);fy=fm(2);fz=fm(3);l=fm(4);mm=fm(5);n=fm(6);
    u=x(4);v=x(5);w=x(6);phi=x(7);theta=x(8);psi=x(9);
    p=x(10);q=x(11);r=x(12);
    cp=cos(phi);sp=sin(phi);ct=cos(theta);st=sin(theta);tt=tan(theta);
    cs=cos(psi);ss_=sin(psi);
    G=P.Jx*P.Jz-P.Jxz^2;
    G1=P.Jxz*(P.Jx-P.Jy+P.Jz)/G; G2=(P.Jz*(P.Jz-P.Jy)+P.Jxz^2)/G;
    G3=P.Jz/G; G4=P.Jxz/G; G5=(P.Jz-P.Jx)/P.Jy; G6=P.Jxz/P.Jy;
    G7=((P.Jx-P.Jy)*P.Jx+P.Jxz^2)/G; G8=P.Jx/G;
    pnd=(ct*cs)*u+(sp*st*cs-cp*ss_)*v+(cp*st*cs+sp*ss_)*w;
    ped=(ct*ss_)*u+(sp*st*ss_+cp*cs)*v+(cp*st*ss_-sp*cs)*w;
    pdd=(-st)*u+(sp*ct)*v+(cp*ct)*w;
    ud=r*v-q*w+fx/P.mass; vd=p*w-r*u+fy/P.mass; wd=q*u-p*v+fz/P.mass;
    phd=p+(sp*tt)*q+(cp*tt)*r; thd=cp*q-sp*r; psd=(sp/ct)*q+(cp/ct)*r;
    pd_=G1*p*q-G2*q*r+G3*l+G4*n; qd=G5*p*r-G6*(p^2-r^2)+mm/P.Jy;
    rd=G7*p*q-G1*q*r+G4*l+G8*n;
    xdot=[pnd;ped;pdd;ud;vd;wd;phd;thd;psd;pd_;qd;rd];
end
