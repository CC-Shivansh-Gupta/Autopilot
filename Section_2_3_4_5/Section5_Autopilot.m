%% Section5_Autopilot.m  (v2 — retuned gains)
% AE700 - Section 5: Autopilot Design using Successive Loop Closure
% IIT Bombay | F-4 Phantom
%
% KEY FIXES vs v1:
%   1. wn_phi = 1.2 rad/s (was 3.0) — below Dutch roll at 5.55 rad/s
%   2. wn_h   = 0.08 rad/s (was 0.375) — phugoid period is ~140s
%   3. Va/pitch sign fixed: pitch DOWN increases speed
%   4. ki_V_dt increased for faster Va tracking
%
% Run: >> Section5_Autopilot

clear; clc; close all;
F4_chap4_params;

fprintf('=====================================================\n');
fprintf('  AE700 Section 5 - Autopilot Design (v2)\n');
fprintf('  F-4 Phantom  |  Va=%.0f ft/s\n', P.Va_trim);
fprintf('=====================================================\n\n');

%% ---- Trim ----
[x_trim, u_trim] = compute_trim(P.Va_trim, 0, Inf, P);
x_trim(3) = -3000;

TF = compute_transfer_functions(x_trim, u_trim, P);
[SS_lon, SS_lat] = compute_ss_models(x_trim, u_trim, P); %#ok

Va_s = P.Va_trim;

%% ---- TF coefficients ----
[~, den_phi] = tfdata(TF.phi_da,'v');
[num_phi, ~] = tfdata(TF.phi_da,'v');
a_phi1 = den_phi(2);
a_phi2 = num_phi(end);

[num_beta, den_beta] = tfdata(TF.beta_dr,'v');
a_beta1 = den_beta(2);  a_beta2 = num_beta(end);

[num_th, den_th] = tfdata(TF.theta_de,'v');
a_theta1 = den_th(2);  a_theta2 = den_th(3);  a_theta3 = num_th(end);

[num_Vdt, den_Vdt] = tfdata(TF.Va_dt,'v');
a_V1 = den_Vdt(2);  a_V2 = num_Vdt(end);

[num_Vth, den_Vth] = tfdata(TF.Va_theta,'v');
a_V1_p = den_Vth(2);  a_V3 = -num_Vth(end);

fprintf('TF coefficients:\n');
fprintf('  a_phi1=%.3f  a_phi2=%.4f\n', a_phi1, a_phi2);
fprintf('  a_theta1=%.3f  a_theta2=%.3f  a_theta3=%.5f\n', a_theta1,a_theta2,a_theta3);
fprintf('  a_V1=%.5f  a_V2=%.5f  a_V3=%.4f\n\n', a_V1,a_V2,a_V3);

delta_lim = 40*pi/180;

%% ============================================================
%  GAINS — designed with bandwidth separation
%  Dutch roll: wn_DR=5.55 rad/s  =>  wn_phi << 5.55
%  Rule: wn_phi < wn_DR/4 = 1.39  =>  use wn_phi=1.2
%  Phugoid: wn_ph=0.045 rad/s, T=140s  =>  wn_h well above 0.045
%  Rule: wn_h < wn_theta/10  AND  wn_h > 5*wn_ph  => 0.08 works
% ============================================================

% LATERAL
% wn_phi must be << Dutch roll wn_DR=5.55 rad/s  (rule: < wn_DR/5)
wn_phi=1.0; zeta_phi=0.707;
kp_phi = wn_phi^2/a_phi2;
kd_phi = (2*zeta_phi*wn_phi - a_phi1)/a_phi2;
ki_phi = 0;

zeta_chi=1.0;  
wn_chi = wn_phi / 5;   % 0.20 rad/s instead of 0.125% 0.125 rad/s — slow outer loop
kp_chi = 2*zeta_chi*wn_chi*Va_s/P.g;
ki_chi = wn_chi^2*Va_s/P.g;

% Rudder: P-only — integrator removed because it winds up at Dutch roll
% frequency (5.55 rad/s) and saturates rudder, destabilising the loop.
% kp_beta set directly; natural weathercock (Cn_beta=0.125) does most work.
kp_beta = 0.5;
ki_beta = 0;

% LONGITUDINAL
wn_theta=6.0; zeta_theta=0.9;
kp_theta = (wn_theta^2 - a_theta2)/a_theta3;
kd_theta = (2*zeta_theta*wn_theta - a_theta1)/a_theta3;
ki_theta = 0;

wn_h=0.08; zeta_h=1.2;
kp_h = 2*zeta_h*wn_h/Va_s;
ki_h = wn_h^2/Va_s;

wn_V_dt=0.5; zeta_V_dt=1.0;
kp_V_dt = (2*zeta_V_dt*wn_V_dt - a_V1)/a_V2;
ki_V_dt = wn_V_dt^2/a_V2;

wn_V_th=0.5; zeta_V_th=1.0;
kp_V_th = (2*zeta_V_th*wn_V_th - a_V1_p)/a_V3;
ki_V_th = wn_V_th^2/a_V3;

fprintf('Gains:\n');
fprintf('  Roll:    kp=%+.4f  kd=%+.4f  wn=%.2f\n',kp_phi,kd_phi,wn_phi);
fprintf('  Course:  kp=%+.4f  ki=%+.6f  wn=%.3f\n',kp_chi,ki_chi,wn_chi);
fprintf('  Sideslip:kp=%+.4f  (P-only, ki=0)\n',kp_beta);
fprintf('  Pitch:   kp=%+.4f  kd=%+.4f  wn=%.2f\n',kp_theta,kd_theta,wn_theta);
fprintf('  Alt:     kp=%+.6f  ki=%+.8f  wn=%.3f\n',kp_h,ki_h,wn_h);
fprintf('  Va/thr:  kp=%+.4f  ki=%+.4f  wn=%.2f\n',kp_V_dt,ki_V_dt,wn_V_dt);
fprintf('  Va/ptch: kp=%+.4f  ki=%+.4f  wn=%.2f\n\n',kp_V_th,ki_V_th,wn_V_th);

%% ---- Commands ----
phi_cmd   = 20*pi/180;
chi_cmd   = 30*pi/180;
theta_cmd = 5*pi/180;
h_cmd     = 3500;
Va_cmd    = 900;
dist_mag  = 0.2;
dist_t    = 5;
Tlat      = 40;
Tlon      = 60;
wind      = zeros(6,1);

%% ---- Simulate ----
fprintf('Simulating...\n');
[t_r,  X_r,  U_r]  = sim_roll(x_trim,u_trim,phi_cmd,0,0,kp_phi,kd_phi,ki_phi,kp_beta,ki_beta,delta_lim,Tlat,P);
[t_rd, X_rd, U_rd] = sim_roll(x_trim,u_trim,phi_cmd,dist_mag,dist_t,kp_phi,kd_phi,ki_phi,kp_beta,ki_beta,delta_lim,Tlat,P);
[t_c,  X_c,  U_c]  = sim_course(x_trim,u_trim,chi_cmd,kp_phi,kd_phi,ki_phi,kp_chi,ki_chi,kp_beta,ki_beta,delta_lim,Tlat,P);
[t_p,  X_p,  U_p]  = sim_pitch(x_trim,u_trim,theta_cmd,kp_theta,kd_theta,ki_theta,delta_lim,Tlon,P);
[t_a,  X_a,  U_a]  = sim_altitude(x_trim,u_trim,h_cmd,kp_theta,kd_theta,ki_theta,kp_h,ki_h,delta_lim,Tlon,P);
[t_vt, X_vt, U_vt] = sim_Va_throttle(x_trim,u_trim,Va_cmd,kp_V_dt,ki_V_dt,kp_theta,kd_theta,ki_theta,kp_h,ki_h,delta_lim,Tlon,P);
[t_vp, X_vp, U_vp] = sim_Va_pitch(x_trim,u_trim,Va_cmd,kp_V_th,ki_V_th,kp_theta,kd_theta,ki_theta,delta_lim,Tlon,P);

beta_r  = arrayfun(@(k) get_beta(X_r(k,:)',u_trim,wind,P),  1:length(t_r))';
Va_vt   = arrayfun(@(k) get_Va(X_vt(k,:)',U_vt(k,:)',wind,P), 1:length(t_vt))';
Va_vp   = arrayfun(@(k) get_Va(X_vp(k,:)',U_vp(k,:)',wind,P), 1:length(t_vp))';

%% ---- PLOTS ----
fs=12; fw='bold';

figure('Name','(a)','Position',[10,570,860,380]);
plot(t_r,rad2deg(X_r(:,7)),'b-','LineWidth',2,'DisplayName','\phi actual'); hold on;
yline(rad2deg(phi_cmd),'r--','LineWidth',1.5,'DisplayName','\phi cmd');
xlabel('Time [s]',FontSize=fs); ylabel('\phi [deg]',FontSize=fs);
title('(a) Roll Angle — No Disturbance',FontSize=fs,FontWeight=fw);
legend(FontSize=11,Location='southeast'); grid on; box on;

figure('Name','(b)','Position',[880,570,860,380]);
plot(t_rd,rad2deg(X_rd(:,7)),'b-','LineWidth',2,'DisplayName','\phi actual'); hold on;
yline(rad2deg(phi_cmd),'r--','LineWidth',1.5,'DisplayName','\phi cmd');
xline(dist_t,'k:','LineWidth',1.5,'DisplayName','Dist (0.2 rad)');
xlabel('Time [s]',FontSize=fs); ylabel('\phi [deg]',FontSize=fs);
title('(b) Roll Angle — With Disturbance (0.2 rad)',FontSize=fs,FontWeight=fw);
legend(FontSize=11,Location='southeast'); grid on; box on;

figure('Name','(c)','Position',[10,110,860,380]);
plot(t_c,rad2deg(X_c(:,9)),'b-','LineWidth',2,'DisplayName','\chi actual'); hold on;
yline(rad2deg(chi_cmd),'r--','LineWidth',1.5,'DisplayName','\chi cmd');
xlabel('Time [s]',FontSize=fs); ylabel('\chi [deg]',FontSize=fs);
title('(c) Course Angle',FontSize=fs,FontWeight=fw);
legend(FontSize=11,Location='southeast'); grid on; box on;

figure('Name','(d)','Position',[880,110,860,380]);
plot(t_r,rad2deg(beta_r),'b-','LineWidth',2,'DisplayName','\beta actual'); hold on;
yline(0,'r--','LineWidth',1.5,'DisplayName','\beta cmd (0)');
xlabel('Time [s]',FontSize=fs); ylabel('\beta [deg]',FontSize=fs);
title('(d) Sideslip Angle',FontSize=fs,FontWeight=fw);
legend(FontSize=11); grid on; box on;

figure('Name','(e)','Position',[10,30,1200,500]);
subplot(2,1,1);
plot(t_r, rad2deg(U_r(:,2)), 'b-','LineWidth',2,'DisplayName','\delta_a (roll,no dist)'); hold on;
plot(t_rd,rad2deg(U_rd(:,2)),'g--','LineWidth',1.5,'DisplayName','\delta_a (roll,dist)');
plot(t_c, rad2deg(U_c(:,2)), 'r:','LineWidth',2,'DisplayName','\delta_a (course)');
yline(40,'k:','LineWidth',1,'HandleVisibility','off'); yline(-40,'k:','LineWidth',1,'HandleVisibility','off');
ylabel('\delta_a [deg]',FontSize=fs); title('Aileron Deflection',FontSize=fs);
legend(FontSize=10,Location='northeast'); grid on; box on;
subplot(2,1,2);
plot(t_r,rad2deg(U_r(:,3)),'b-','LineWidth',2,'DisplayName','\delta_r (roll)'); hold on;
plot(t_c,rad2deg(U_c(:,3)),'r:','LineWidth',2,'DisplayName','\delta_r (course)');
yline(20,'k:','LineWidth',1,'HandleVisibility','off'); yline(-20,'k:','LineWidth',1,'HandleVisibility','off');
xlabel('Time [s]',FontSize=fs); ylabel('\delta_r [deg]',FontSize=fs);
title('Rudder Deflection',FontSize=fs); legend(FontSize=10,Location='northeast'); grid on; box on;
sgtitle('(e) Lateral Control Inputs',FontSize=fs,FontWeight=fw);

figure('Name','(f)','Position',[10,570,860,380]);
plot(t_p,rad2deg(X_p(:,8)),'b-','LineWidth',2,'DisplayName','\theta actual'); hold on;
yline(rad2deg(theta_cmd),'r--','LineWidth',1.5,'DisplayName','\theta cmd');
xlabel('Time [s]',FontSize=fs); ylabel('\theta [deg]',FontSize=fs);
title('(f) Pitch Angle',FontSize=fs,FontWeight=fw);
legend(FontSize=11,Location='southeast'); grid on; box on;

figure('Name','(g)','Position',[880,570,860,380]);
plot(t_a,-X_a(:,3),'b-','LineWidth',2,'DisplayName','h actual'); hold on;
yline(h_cmd,'r--','LineWidth',1.5,'DisplayName','h cmd');
xlabel('Time [s]',FontSize=fs); ylabel('Altitude [ft]',FontSize=fs);
title('(g) Altitude',FontSize=fs,FontWeight=fw);
legend(FontSize=11,Location='southeast'); grid on; box on;

figure('Name','(h)','Position',[10,110,860,380]);
plot(t_vt,Va_vt,'b-','LineWidth',2,'DisplayName','Va actual'); hold on;
yline(Va_cmd,'r--','LineWidth',1.5,'DisplayName','Va cmd');
xlabel('Time [s]',FontSize=fs); ylabel('V_a [ft/s]',FontSize=fs);
title('(h) Airspeed — Throttle Control',FontSize=fs,FontWeight=fw);
legend(FontSize=11,Location='southeast'); grid on; box on;

figure('Name','(i)','Position',[880,110,860,380]);
plot(t_vp,Va_vp,'b-','LineWidth',2,'DisplayName','Va actual'); hold on;
yline(Va_cmd,'r--','LineWidth',1.5,'DisplayName','Va cmd');
xlabel('Time [s]',FontSize=fs); ylabel('V_a [ft/s]',FontSize=fs);
title('(i) Airspeed — Pitch Command',FontSize=fs,FontWeight=fw);
legend(FontSize=11,Location='southeast'); grid on; box on;

figure('Name','(j)','Position',[10,30,1200,600]);
subplot(2,2,1);
plot(t_p,rad2deg(U_p(:,1)),'b-','LineWidth',2,'DisplayName','\delta_e (pitch)'); hold on;
plot(t_a,rad2deg(U_a(:,1)),'r--','LineWidth',1.5,'DisplayName','\delta_e (alt)');
yline(40,'k:','HandleVisibility','off'); yline(-40,'k:','HandleVisibility','off');
ylabel('\delta_e [deg]',FontSize=fs); title('Elevator — Pitch/Alt',FontSize=fs);
legend(FontSize=10); grid on; box on;
subplot(2,2,2);
plot(t_a,U_a(:,4),'b-','LineWidth',2); ylim([0,1.1]);
ylabel('\delta_t',FontSize=fs); xlabel('Time [s]',FontSize=fs);
title('Throttle — Altitude Autopilot',FontSize=fs); grid on; box on;
subplot(2,2,3);
plot(t_vt,U_vt(:,4),'b-','LineWidth',2); ylim([0,1.1]);
ylabel('\delta_t',FontSize=fs); xlabel('Time [s]',FontSize=fs);
title('Throttle — Va/Throttle Autopilot',FontSize=fs); grid on; box on;
subplot(2,2,4);
plot(t_vp,rad2deg(U_vp(:,1)),'b-','LineWidth',2);
yline(40,'k:'); yline(-40,'k:');
ylabel('\delta_e [deg]',FontSize=fs); xlabel('Time [s]',FontSize=fs);
title('Elevator — Va/Pitch Autopilot',FontSize=fs); grid on; box on;
sgtitle('(j) Longitudinal Control Inputs',FontSize=fs,FontWeight=fw);

fprintf('Done.\n');

%% ============================================================
%  SIMULATION FUNCTIONS
% ============================================================
function [t,X,U]=sim_roll(x0,u0,phi_cmd,dist_mag,dist_t,kp_phi,kd_phi,ki_phi,kp_beta,ki_beta,dlim,Tsim,P)
    wind=zeros(6,1); dt=0.02; t=(0:dt:Tsim)'; N=length(t);
    X=zeros(N,12); U=zeros(N,4); X(1,:)=x0';
    phi_int=0;
    oo=odeset('RelTol',1e-6,'AbsTol',1e-8);
    for k=1:N-1
        x=X(k,:)'; phi=x(7); p=x(10);
        % Disturbance: pulse lasting 0.5s only (not permanent step)
dist = dist_mag * (t(k) >= dist_t) * (t(k) < dist_t + 0.5);
        e=phi_cmd-phi; phi_int=phi_int+e*dt;
        da=kp_phi*e-kd_phi*p+ki_phi*phi_int+dist;
        da=max(-dlim,min(dlim,da));
        fm=forces_moments(x,[u0(1);da;u0(3);u0(4)],wind,P);
        beta=fm(9);
        dr=-kp_beta*beta;   % P-only: no integrator
        dr=max(-20*pi/180,min(20*pi/180,dr));
        d=[u0(1);da;dr;u0(4)]; U(k,:)=d';
        [~,Xk]=ode45(@(t,x)feom(x,d,wind,P),[0,dt],x,oo);
        X(k+1,:)=Xk(end,:);
    end; U(end,:)=U(end-1,:);
end

function [t,X,U]=sim_course(x0,u0,chi_cmd,kp_phi,kd_phi,ki_phi,kp_chi,ki_chi,kp_beta,ki_beta,dlim,Tsim,P)
    wind=zeros(6,1); dt=0.02; t=(0:dt:Tsim)'; N=length(t);
    X=zeros(N,12); U=zeros(N,4); X(1,:)=x0';
    chi_int=0; phi_int=0; phi_max=35*pi/180;
    oo=odeset('RelTol',1e-6,'AbsTol',1e-8);
    for k=1:N-1
        x=X(k,:)'; phi=x(7); p=x(10); psi=x(9);
        e_chi=atan2(sin(chi_cmd-psi),cos(chi_cmd-psi));
        chi_int = max(-pi/4, min(pi/4, chi_int + e_chi*dt));  % anti-windup
        phi_cmd=max(-phi_max,min(phi_max,kp_chi*e_chi+ki_chi*chi_int));
        e_phi=phi_cmd-phi; phi_int=phi_int+e_phi*dt;
        da=max(-dlim,min(dlim,kp_phi*e_phi-kd_phi*p+ki_phi*phi_int));
        fm=forces_moments(x,[u0(1);da;u0(3);u0(4)],wind,P);
        beta=fm(9);
        dr=max(-20*pi/180,min(20*pi/180,-kp_beta*beta));  % P-only
        d=[u0(1);da;dr;u0(4)]; U(k,:)=d';
        [~,Xk]=ode45(@(t,x)feom(x,d,wind,P),[0,dt],x,oo);
        X(k+1,:)=Xk(end,:);
    end; U(end,:)=U(end-1,:);
end

function [t,X,U]=sim_pitch(x0,u0,theta_cmd,kp,kd,ki,dlim,Tsim,P)
    wind=zeros(6,1); dt=0.02; t=(0:dt:Tsim)'; N=length(t);
    X=zeros(N,12); U=zeros(N,4); X(1,:)=x0'; th_int=0;
    oo=odeset('RelTol',1e-6,'AbsTol',1e-8);
    for k=1:N-1
        x=X(k,:)'; e=theta_cmd-x(8); th_int=th_int+e*dt;
        de=max(-dlim,min(dlim,u0(1)+kp*e-kd*x(11)+ki*th_int));
        d=[de;u0(2);u0(3);u0(4)]; U(k,:)=d';
        [~,Xk]=ode45(@(t,x)feom(x,d,wind,P),[0,dt],x,oo);
        X(k+1,:)=Xk(end,:);
    end; U(end,:)=U(end-1,:);
end

function [t,X,U]=sim_altitude(x0,u0,h_cmd,kp_th,kd_th,ki_th,kp_h,ki_h,dlim,Tsim,P)
    wind=zeros(6,1); dt=0.02; t=(0:dt:Tsim)'; N=length(t);
    X=zeros(N,12); U=zeros(N,4); X(1,:)=x0';
    h_int=0; th_int=0; thmax=20*pi/180;
    oo=odeset('RelTol',1e-6,'AbsTol',1e-8);
    for k=1:N-1
        x=X(k,:)'; h=-x(3); theta=x(8); q=x(11);
        e_h=h_cmd-h; h_int=h_int+e_h*dt;
        tc=max(-thmax,min(thmax,kp_h*e_h+ki_h*h_int));
        e_t=tc-theta; th_int=th_int+e_t*dt;
        de=max(-dlim,min(dlim,u0(1)+kp_th*e_t-kd_th*q+ki_th*th_int));
        d=[de;u0(2);u0(3);u0(4)]; U(k,:)=d';
        [~,Xk]=ode45(@(t,x)feom(x,d,wind,P),[0,dt],x,oo);
        X(k+1,:)=Xk(end,:);
    end; U(end,:)=U(end-1,:);
end

function [t,X,U]=sim_Va_throttle(x0,u0,Va_cmd,kp_V,ki_V,kp_th,kd_th,ki_th,kp_h,ki_h,dlim,Tsim,P)
    wind=zeros(6,1); dt=0.02; t=(0:dt:Tsim)'; N=length(t);
    X=zeros(N,12); U=zeros(N,4); X(1,:)=x0';
    V_int=0; th_int=0; h_int=0; h_cmd=-x0(3); thmax=15*pi/180;
    oo=odeset('RelTol',1e-6,'AbsTol',1e-8);
    for k=1:N-1
        x=X(k,:)';
        fm=forces_moments(x,u0,wind,P); Va=fm(7);
        e_V=Va_cmd-Va; V_int=V_int+e_V*dt;
        dtc=max(0,min(1,u0(4)+kp_V*e_V+ki_V*V_int));
        e_h=-x(3)-h_cmd; h_int=h_int+(-e_h)*dt;  % e_h = h_cmd - h
        e_h2=h_cmd-(-x(3));
        h_int=h_int-(-e_h)*dt+e_h2*dt;            % fix double update
        tc=max(-thmax,min(thmax,kp_h*e_h2+ki_h*h_int));
        e_t=tc-x(8); th_int=th_int+e_t*dt;
        de=max(-dlim,min(dlim,u0(1)+kp_th*e_t-kd_th*x(11)+ki_th*th_int));
        d=[de;u0(2);u0(3);dtc]; U(k,:)=d';
        [~,Xk]=ode45(@(t,x)feom(x,d,wind,P),[0,dt],x,oo);
        X(k+1,:)=Xk(end,:);
    end; U(end,:)=U(end-1,:);
end

function [t,X,U]=sim_Va_pitch(x0,u0,Va_cmd,kp_V,ki_V,kp_th,kd_th,ki_th,dlim,Tsim,P)
    % pitch DOWN (negative theta) to INCREASE speed
    wind=zeros(6,1); dt=0.02; t=(0:dt:Tsim)'; N=length(t);
    X=zeros(N,12); U=zeros(N,4); X(1,:)=x0';
    V_int=0; th_int=0; thmax=15*pi/180;
    oo=odeset('RelTol',1e-6,'AbsTol',1e-8);
    for k=1:N-1
        x=X(k,:)';
        fm=forces_moments(x,u0,wind,P); Va=fm(7);
        e_V=Va_cmd-Va; V_int=V_int+e_V*dt;
        tc=max(-thmax,min(thmax,-(kp_V*e_V+ki_V*V_int)));
        e_t=tc-x(8); th_int=th_int+e_t*dt;
        de=max(-dlim,min(dlim,u0(1)+kp_th*e_t-kd_th*x(11)+ki_th*th_int));
        d=[de;u0(2);u0(3);u0(4)]; U(k,:)=d';
        [~,Xk]=ode45(@(t,x)feom(x,d,wind,P),[0,dt],x,oo);
        X(k+1,:)=Xk(end,:);
    end; U(end,:)=U(end-1,:);
end

function b=get_beta(x,u,wind,P); fm=forces_moments(x,u,wind,P); b=fm(9); end
function v=get_Va(x,u,wind,P);   fm=forces_moments(x,u,wind,P); v=fm(7); end

function xdot=feom(x,d,wind,P)
    fm=forces_moments(x,d,wind,P);
    fx=fm(1);fy=fm(2);fz=fm(3);l=fm(4);mm=fm(5);n=fm(6);
    u=x(4);v=x(5);w=x(6);phi=x(7);theta=x(8);psi=x(9);p=x(10);q=x(11);r=x(12);
    cp=cos(phi);sp=sin(phi);ct=cos(theta);st=sin(theta);tt=tan(theta);cs=cos(psi);ss=sin(psi);
    G=P.Jx*P.Jz-P.Jxz^2;
    G1=P.Jxz*(P.Jx-P.Jy+P.Jz)/G; G2=(P.Jz*(P.Jz-P.Jy)+P.Jxz^2)/G;
    G3=P.Jz/G; G4=P.Jxz/G; G5=(P.Jz-P.Jx)/P.Jy; G6=P.Jxz/P.Jy;
    G7=((P.Jx-P.Jy)*P.Jx+P.Jxz^2)/G; G8=P.Jx/G;
    pnd=(ct*cs)*u+(sp*st*cs-cp*ss)*v+(cp*st*cs+sp*ss)*w;
    ped=(ct*ss)*u+(sp*st*ss+cp*cs)*v+(cp*st*ss-sp*cs)*w;
    pdd=(-st)*u+(sp*ct)*v+(cp*ct)*w;
    ud=r*v-q*w+fx/P.mass; vd=p*w-r*u+fy/P.mass; wd=q*u-p*v+fz/P.mass;
    phd=p+(sp*tt)*q+(cp*tt)*r; thd=cp*q-sp*r; psd=(sp/ct)*q+(cp/ct)*r;
    pd_=G1*p*q-G2*q*r+G3*l+G4*n; qd=G5*p*r-G6*(p^2-r^2)+mm/P.Jy; rd=G7*p*q-G1*q*r+G4*l+G8*n;
    xdot=[pnd;ped;pdd;ud;vd;wd;phd;thd;psd;pd_;qd;rd];
end
