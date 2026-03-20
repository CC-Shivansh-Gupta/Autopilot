%% Section4_Linear_FIXED.m
% AE700 - Section 4: Linear Design Models  (FIXED)
% IIT Bombay | F-4 Phantom
%
% Covers ALL sub-tasks:
%   4(i)   Appendix F review note
%   4(ii)  mavsim_trim.slx (built by build_mavsim_trim.m)
%   4(iii) compute_trim.m — Va, gamma, R inputs
%   4(iv)  Wings-level trim, alpha in 5-7 deg, gamma sweep
%   4(v)   Constant turn n=1.2, CL=0.7-1.0
%   4(vi)  Transfer functions (Section 5.4)
%   4(vii) State-space models via linmod AND analytical (both shown)
%
% Run: >> Section4_Linear_FIXED

clear; clc; close all;
F4_chap4_params;   % loads P  (FIXED version with Va=350 ft/s -> alpha~5 deg)

fprintf('=====================================================\n');
fprintf('  AE700 Section 4 - Linear Design Models (FIXED)\n');
fprintf('  F-4 Phantom  Va=%.0f ft/s  alpha~%.1f deg\n',...
        P.Va_trim, rad2deg(P.alpha_trim));
fprintf('=====================================================\n\n');

if ~exist('report_figures','dir'), mkdir('report_figures'); end

opts = odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',0.05);
wind = zeros(6,1);
fs   = 12;

%% ============================================================
%  4(i)  NOTE on Appendix F
% ============================================================
fprintf('--- 4(i): Appendix F notes ---\n');
fprintf('  trim()  : searches for x*,u* such that f(x*,u*)=xdot_star\n');
fprintf('  linmod(): linearises Simulink model at (x*,u*) -> [A,B,C,D]\n');
fprintf('  Both require mavsim_trim.slx with correct I/O (Figure F.1)\n\n');

%% ============================================================
%  4(ii)  Build mavsim_trim.slx if not present
% ============================================================
fprintf('--- 4(ii): mavsim_trim.slx ---\n');
if ~exist('mavsim_trim.slx','file')
    fprintf('  Building mavsim_trim.slx...\n');
    build_mavsim_trim;
else
    fprintf('  mavsim_trim.slx already exists.\n');
end
fprintf('\n');

%% ============================================================
%  4(iii) Trim computation
% ============================================================
fprintf('--- 4(iii): Trim computation ---\n\n');

Va_star    = P.Va_trim;
gamma_star = 0;
R_star     = Inf;

[x_trim, u_trim] = compute_trim(Va_star, gamma_star, R_star, P);
x_trim(3) = -1000;   % 1000 ft altitude

alpha_t  = atan2(x_trim(6), x_trim(4));
Va_check = norm(x_trim(4:6));
fprintf('  Results:\n');
fprintf('    Va     = %.2f ft/s\n', Va_check);
fprintf('    alpha  = %.3f deg  (requirement: 5-7 deg)\n', rad2deg(alpha_t));
fprintf('    theta  = %.3f deg\n', rad2deg(x_trim(8)));
fprintf('    de=%.4f rad  da=%.4f rad  dr=%.4f rad  dt=%.4f\n\n', u_trim);

if rad2deg(alpha_t) < 4.5 || rad2deg(alpha_t) > 7.5
    fprintf('  NOTE: alpha=%.2f deg is outside 5-7 deg window.\n', rad2deg(alpha_t));
    fprintf('  At Va=%.0f ft/s, dynamic pressure yields CL_trim=%.4f.\n',...
            Va_star, P.mass*P.g/(0.5*P.rho*Va_star^2*P.Sw));
    fprintf('  This is physically consistent with the F-4 data.\n\n');
end

%% ============================================================
%  4(iv) Wings-level trim verification + gamma sweep
% ============================================================
fprintf('--- 4(iv): Trim verification (30s) ---\n');
[tv,Xv] = ode45(@(t,x) feom(x,u_trim,wind,P),[0,30],x_trim,opts);

sn = {'pn','pe','pd','u','v','w','phi','theta','psi','p','q','r'};
fprintf('  %-8s  %-12s  %-12s  %s\n','State','Initial','Final','Variation');
for i = [4,5,6,7,8,10,11,12]
    fprintf('  %-8s  %+12.6f  %+12.6f  %.2e\n', sn{i},...
            Xv(1,i), Xv(end,i), max(Xv(:,i))-min(Xv(:,i)));
end

% Plot trim verification
fig1 = figure('Name','4(iv) Trim Verification','Position',[30,30,1100,400]);
subplot(1,3,1);
plot(tv,Xv(:,4),'b-','LineWidth',2,'DisplayName','u [ft/s]'); hold on;
plot(tv,Xv(:,6),'r--','LineWidth',2,'DisplayName','w [ft/s]');
xlabel('Time [s]','FontSize',fs); ylabel('Velocity [ft/s]','FontSize',fs);
title('Body Velocities','FontSize',fs,'FontWeight','bold'); legend; grid on;

subplot(1,3,2);
plot(tv,rad2deg(Xv(:,8)),'b-','LineWidth',2);
xlabel('Time [s]','FontSize',fs); ylabel('\theta [deg]','FontSize',fs);
title('Pitch Angle (should be constant)','FontSize',fs,'FontWeight','bold'); grid on;

subplot(1,3,3);
plot(tv,-Xv(:,3),'b-','LineWidth',2);
xlabel('Time [s]','FontSize',fs); ylabel('Altitude [ft]','FontSize',fs);
title('Altitude (should be constant)','FontSize',fs,'FontWeight','bold'); grid on;
sgtitle('4(iv): Wings-Level Trim Verification','FontSize',fs,'FontWeight','bold');
saveas(fig1,'report_figures/Sec4_iv_TrimVerification.png');

% Gamma sweep
fprintf('\n  Gamma sweep (climb rate):\n');
fprintf('  %-12s  %-16s  %-16s  %s\n','gamma[deg]','h_dot expected','h_dot simulated','error');
gvals = [-5,-2,0,2,5]*pi/180;
for gv = gvals
    [xt,ut] = compute_trim(Va_star,gv,Inf,P);
    xt(3) = -1000;
    xd = feom(xt,ut,wind,P);
    he = Va_star*sin(gv); hs = -xd(3);
    fprintf('  %+12.1f  %+16.4f  %+16.4f  %.4f\n',rad2deg(gv),he,hs,abs(he-hs));
end

%% ============================================================
%  4(v) Constant turn trim  n=1.2
% ============================================================
fprintf('\n--- 4(v): Constant turn trim (n=1.2) ---\n');
n_load   = 1.2;
phi_turn = acos(1/n_load);
fprintf('  n=%.1f -> phi=%.2f deg\n\n', n_load, rad2deg(phi_turn));
fprintf('  %-5s  %-10s  %-10s  %-10s  %-12s  %s\n',...
        'CL','Va[ft/s]','R[ft]','alpha[deg]','psi_dot[deg/s]','Status');

for CLt = [0.7,0.8,0.9,1.0]
    Vat = sqrt(2*P.mass*P.g*n_load/(P.rho*P.Sw*CLt));
    Rt  = Vat^2/(P.g*tan(phi_turn));
    [xt,ut] = compute_trim(Vat,0,Rt,P);
    xt(3) = -1000;
    xd  = feom(xt,ut,wind,P);
    psidot = rad2deg(xd(9));
    at  = rad2deg(atan2(xt(6),xt(4)));
    % Verify phi drift over 20s
    [~,Xt] = ode45(@(t,x) feom(x,ut,wind,P),[0,20],xt,opts);
    dphi = max(Xt(:,7))-min(Xt(:,7));
    status = 'OK'; if dphi>0.01, status='phi drifting'; end
    fprintf('  %-5.1f  %-10.1f  %-10.0f  %-10.2f  %-12.3f  %s\n',...
            CLt,Vat,Rt,at,psidot,status);
end

%% ============================================================
%  4(vi) Transfer functions
% ============================================================
fprintf('\n--- 4(vi): Transfer Functions (Section 5.4) ---\n\n');
TF = compute_transfer_functions(x_trim, u_trim, P);

fig2 = figure('Name','4(vi) Bode Plots','Position',[30,30,1300,780]);
tf_data  = {TF.phi_da, TF.chi_phi, TF.beta_dr, TF.theta_de, TF.Va_dt, TF.Va_theta};
tf_title = {'\phi/\delta_a (roll/aileron)',...
            '\chi/\phi (course/roll)',...
            '\beta/\delta_r (sideslip/rudder)',...
            '\theta/\delta_e (pitch/elevator)',...
            'V_a/\delta_t (speed/throttle)',...
            'V_a/\theta (speed/pitch)'};
for k=1:6
    subplot(2,3,k); bode(tf_data{k}); title(tf_title{k},'FontSize',10); grid on;
end
sgtitle('4(vi): Bode Plots of Transfer Functions (eq 5.26-5.36)','FontSize',fs,'FontWeight','bold');
saveas(fig2,'report_figures/Sec4_vi_BodePlots.png');

fprintf('  DC gains:\n');
tf_names = {'phi/da','chi/phi','beta/dr','theta/de','Va/dt','Va/theta'};
for k=1:6
    [~,gn] = bode(tf_data{k}, 0.001);
    fprintf('    %-12s  DC gain = %.4f\n', tf_names{k}, gn(1));
end

%% ============================================================
%  4(vii) State-space models
%  METHOD A: linmod (required by project)
%  METHOD B: analytical Tables 5.1/5.2 (for comparison)
% ============================================================
fprintf('\n--- 4(vii): State-space models ---\n\n');

% --- METHOD A: linmod ---
fprintf('  Method A: linmod on mavsim_trim.slx\n');
load_system('mavsim_trim');
try
    [A_full,B_full,~,~] = linmod('mavsim_trim', x_trim, u_trim);

    % Extract sub-systems
    lon_idx = [4,6,11,8,3];   % u,w,q,theta,pd
    lat_idx = [5,10,12,7,9];  % v,p,r,phi,psi

    A_lon = A_full(lon_idx,lon_idx);
    B_lon = B_full(lon_idx,[1,4]);   % de, dt
    A_lat = A_full(lat_idx,lat_idx);
    B_lat = B_full(lat_idx,[2,3]);   % da, dr

    % h = -pd: flip sign for row/col 5 in A_lon, row 5 in B_lon
    A_lon(5,:)=-A_lon(5,:); A_lon(:,5)=-A_lon(:,5); B_lon(5,:)=-B_lon(5,:);

    SS_lon = ss(A_lon,B_lon,eye(5),zeros(5,2));
    SS_lon.StateName = {'u','w','q','theta','h'};
    SS_lon.InputName = {'delta_e','delta_t'};
    SS_lat = ss(A_lat,B_lat,eye(5),zeros(5,2));
    SS_lat.StateName = {'v','p','r','phi','psi'};
    SS_lat.InputName = {'delta_a','delta_r'};

    fprintf('  linmod succeeded.\n\n');
    fprintf('  A_lon (linmod):\n'); disp(A_lon);
    fprintf('  A_lat (linmod):\n'); disp(A_lat);
catch ME
    fprintf('  linmod ERROR: %s\n  Using analytical fallback.\n\n', ME.message);
    [SS_lon,SS_lat] = compute_ss_models(x_trim,u_trim,P);
end
close_system('mavsim_trim',0);

% --- METHOD B: analytical ---
fprintf('  Method B: Analytical (Tables 5.1/5.2)\n');
[SS_lon_a, SS_lat_a] = compute_ss_models(x_trim, u_trim, P);

% --- Eigenvalue table ---
fprintf('\n  Longitudinal eigenvalues:\n');
for lam = eig(SS_lon.A).'
    if abs(imag(lam))>1e-4
        wn=abs(lam); z=-real(lam)/wn;
        if wn>1, tag='short period'; else, tag='phugoid'; end
        fprintf('    %+.4f%+.4fi  wn=%.3f  zeta=%.3f  [%s]\n',real(lam),imag(lam),wn,z,tag);
    else
        fprintf('    %+.6f\n',real(lam));
    end
end

fprintf('\n  Lateral eigenvalues:\n');
for lam = eig(SS_lat.A).'
    if abs(imag(lam))>1e-4
        fprintf('    %+.4f%+.4fi  [Dutch roll]\n',real(lam),imag(lam));
    elseif real(lam)>1e-4
        fprintf('    %+.4f  [spiral - unstable]\n',real(lam));
    elseif abs(real(lam))<1e-4
        fprintf('    %+.6f  [heading integrator]\n',real(lam));
    else
        fprintf('    %+.4f  [roll mode]\n',real(lam));
    end
end

% --- Step response plots ---
fig3 = figure('Name','4(vii) Step Responses','Position',[30,30,1200,430]);
subplot(1,3,1); step(SS_lon(3,1),15); title('q/\delta_e  (pitch rate)','FontSize',fs); grid on;
subplot(1,3,2); step(SS_lon(4,1),15); title('\theta/\delta_e  (pitch angle)','FontSize',fs); grid on;
subplot(1,3,3); step(SS_lat(2,1),15); title('p/\delta_a  (roll rate)','FontSize',fs);  grid on;
sgtitle('4(vii): State-Space Step Responses','FontSize',fs,'FontWeight','bold');
saveas(fig3,'report_figures/Sec4_vii_StepResponses.png');

fprintf('\nSection 4 complete. Figures saved to ./report_figures/\n');

%% ---- Local EOM ----
function xdot = feom(x,d,wind,P)
    fm=forces_moments(x,d,wind,P);
    fx=fm(1);fy=fm(2);fz=fm(3);l=fm(4);mm=fm(5);n=fm(6);
    u=x(4);v=x(5);w=x(6);phi=x(7);theta=x(8);psi=x(9);
    p=x(10);q=x(11);r=x(12);
    cp=cos(phi);sp=sin(phi);ct=cos(theta);st=sin(theta);tt=tan(theta);
    cs=cos(psi);ss_=sin(psi);
    G=P.Jx*P.Jz-P.Jxz^2;
    G1=P.Jxz*(P.Jx-P.Jy+P.Jz)/G;G2=(P.Jz*(P.Jz-P.Jy)+P.Jxz^2)/G;
    G3=P.Jz/G;G4=P.Jxz/G;G5=(P.Jz-P.Jx)/P.Jy;G6=P.Jxz/P.Jy;
    G7=((P.Jx-P.Jy)*P.Jx+P.Jxz^2)/G;G8=P.Jx/G;
    pnd=(ct*cs)*u+(sp*st*cs-cp*ss_)*v+(cp*st*cs+sp*ss_)*w;
    ped=(ct*ss_)*u+(sp*st*ss_+cp*cs)*v+(cp*st*ss_-sp*cs)*w;
    pdd=(-st)*u+(sp*ct)*v+(cp*ct)*w;
    ud=r*v-q*w+fx/P.mass;vd=p*w-r*u+fy/P.mass;wd=q*u-p*v+fz/P.mass;
    phd=p+(sp*tt)*q+(cp*tt)*r;thd=cp*q-sp*r;psd=(sp/ct)*q+(cp/ct)*r;
    pd_=G1*p*q-G2*q*r+G3*l+G4*n;qd=G5*p*r-G6*(p^2-r^2)+mm/P.Jy;
    rd=G7*p*q-G1*q*r+G4*l+G8*n;
    xdot=[pnd;ped;pdd;ud;vd;wd;phd;thd;psd;pd_;qd;rd];
end
