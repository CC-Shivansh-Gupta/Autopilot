%% Section4_Linear.m  (FIXED v2)
% =========================================================
% AE700 - Section 4: Linear Design Models
% IIT Bombay | F-4 Phantom
%
% KEY FIXES vs v1:
%   1. Va computed FROM target alpha (5-7 deg) instead of using
%      Va_trim=845 ft/s which gave alpha=-0.5 deg (far too low AoA).
%      Lift balance: CL = W/(qbar*Sw)  =>  Va = sqrt(2W/(rho*Sw*CL))
%   2. compute_ss_linmod added alongside analytical compute_ss_models
%      to satisfy the "use linmod" requirement of task 4(vii).
%   3. Gamma sweep extended to show correct climb rates.
%
% Tasks covered:
%   (iii)  compute_trim(Va, gamma, R, P)
%   (iv)   Wings-level trim, alpha=5-7 deg, gamma sweep
%   (v)    Constant turn, n=1.2, CL=0.7-1.0
%   (vi)   Transfer functions  (compute_transfer_functions)
%   (vii)  State-space: analytical (compute_ss_models)
%                     + linmod     (compute_ss_linmod)
%
% Dependencies: F4_params.m, forces_moments.m, compute_trim.m,
%               compute_transfer_functions.m, compute_ss_models.m,
%               compute_ss_linmod.m, mavsim_trim.slx
% =========================================================

clear; clc; close all;
F4_params;

if ~exist('report_figures','dir'), mkdir('report_figures'); end

fprintf('=====================================================\n');
fprintf('  AE700 Section 4 - Linear Design Models (FIXED v2)\n');
fprintf('  F-4 Phantom\n');
fprintf('=====================================================\n\n');

opts = odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',0.05);
wind = zeros(6,1);

%% ============================================================
%  SECTION 4(iii)+(iv): Wings-level trim
%
%  FIX: choose Va so that alpha is in the 5-7 deg range.
%
%  Lift balance at level flight:
%    CL_trim = W / (0.5*rho*Va^2*Sw)
%    CL = CLo + CL_a*alpha  =>  alpha = (CL-CLo)/CL_a
%
%  Target alpha = 6 deg = 0.1047 rad
%    CL_target = 0.1 + 3.75*0.1047 = 0.4926
%    Va = sqrt(2*39000/(0.002377*530*0.4926)) = 355 ft/s
%
%  Verify: alpha at Va=355 ft/s should come out ~6 deg
% ============================================================
fprintf('--- Section 4(iii)+(iv): Wings-level trim ---\n\n');

% --- Compute Va that gives alpha in 5-7 deg range ---
alpha_target = 6 * pi/180;                          % 6 degrees
CL_target    = P.CLo + P.CL_a * alpha_target;       % = 0.4926
Va_trim_new  = sqrt(2*P.mass*P.g / (P.rho*P.Sw*CL_target));

fprintf('  Target alpha = %.1f deg\n', rad2deg(alpha_target));
fprintf('  Required CL  = %.4f\n', CL_target);
fprintf('  Required Va  = %.1f ft/s  (was 845 ft/s)\n\n', Va_trim_new);

% Update P with new trim airspeed
P.Va_trim = Va_trim_new;

gamma = 0;
R     = Inf;

[x_trim, u_trim] = compute_trim(P.Va_trim, gamma, R, P);
x_trim(3) = -1000;   % 1000 ft altitude

% Verify alpha at trim
alpha_actual = atan2(x_trim(6), x_trim(4));
fprintf('\n  Achieved trim alpha = %.2f deg  (target: 5-7 deg)\n', rad2deg(alpha_actual));
fprintf('  Trim theta          = %.2f deg\n', rad2deg(x_trim(8)));
fprintf('  de=%.4f rad  da=%.4f rad  dr=%.4f rad  dt=%.4f\n\n', ...
        u_trim(1),u_trim(2),u_trim(3),u_trim(4));

% --- Verify: simulate 30s, states should be constant ---
[tv, Xv] = ode45(@(t,x) full_eom(x,u_trim,wind,P), [0,30], x_trim, opts);

fprintf('  30s trim verification (all variations should be ~0):\n');
fprintf('  %-8s  %s\n','State','Max variation');
state_names = {'pn','pe','pd','u','v','w','phi','theta','psi','p','q','r'};
for i = [4,5,6,7,8,10,11,12]
    fprintf('  %-8s  %.2e\n', state_names{i}, max(Xv(:,i))-min(Xv(:,i)));
end

%% ---- 4(iv) Gamma sweep ----
fprintf('\n  Gamma sweep — climb rate verification:\n');
fprintf('  %-12s  %-16s  %-16s  %s\n','gamma[deg]','h_dot expected','h_dot simulated','error [ft/s]');

gamma_vals = [-5,-3,0,3,5] * pi/180;
fig_gamma = figure('Name','4(iv): Gamma Sweep','Position',[30,30,900,500]);
colors = lines(length(gamma_vals));
hold on;

for gk = 1:length(gamma_vals)
    gv = gamma_vals(gk);
    [xt,ut] = compute_trim(P.Va_trim, gv, Inf, P);
    xt(3) = -1000;
    xdot0 = full_eom(xt, ut, wind, P);
    hdot_exp = P.Va_trim * sin(gv);
    hdot_sim = -xdot0(3);
    fprintf('  %+12.1f  %+16.3f  %+16.3f  %.4f\n', ...
            rad2deg(gv), hdot_exp, hdot_sim, abs(hdot_exp-hdot_sim));

    % Simulate to show altitude changing correctly
    [tg,Xg] = ode45(@(t,x) full_eom(x,ut,wind,P),[0,20],xt,opts);
    plot(tg, -Xg(:,3),'Color',colors(gk,:),'LineWidth',2, ...
         'DisplayName',sprintf('\\gamma=%+.0f°',rad2deg(gv)));
end
xlabel('Time [s]'); ylabel('Altitude [ft]');
title('4(iv): Altitude vs Time for Gamma Sweep','FontWeight','bold');
legend('Location','best'); grid on; box on;
saveas(fig_gamma,'report_figures/Sec4_iv_GammaSweep.png');
fprintf('\n  Figure saved: report_figures/Sec4_iv_GammaSweep.png\n\n');

%% ============================================================
%  SECTION 4(v): Constant turn, n=1.2, CL=0.7-1.0
% ============================================================
fprintf('--- Section 4(v): Constant turn trim (n=1.2) ---\n\n');

n_load   = 1.2;
phi_turn = acos(1/n_load);
fprintf('  n=%.1f -> bank angle phi=%.2f deg\n\n', n_load, rad2deg(phi_turn));
fprintf('  %-6s  %-10s  %-10s  %-10s  %-12s  %s\n', ...
        'CL','Va [ft/s]','R [ft]','alpha [deg]','phi_var','psi_change');

CL_vals = [0.7, 0.8, 0.9, 1.0];
fig_turn = figure('Name','4(v): Constant Turn','Position',[30,500,1000,400]);
colors2  = lines(length(CL_vals));

for ck = 1:length(CL_vals)
    CLt = CL_vals(ck);
    Vat = sqrt(2*P.mass*P.g*n_load / (P.rho*P.Sw*CLt));
    Rt  = Vat^2 / (P.g*tan(phi_turn));

    [xt,ut] = compute_trim(Vat, 0, Rt, P);
    xt(3) = -1000;

    [tt,Xt] = ode45(@(t,x) full_eom(x,ut,wind,P),[0,30],xt,opts);
    dphi = max(Xt(:,7)) - min(Xt(:,7));
    dpsi = Xt(end,9)    - Xt(1,9);
    at   = atan2(xt(6), xt(4));

    fprintf('  %-6.1f  %-10.1f  %-10.0f  %-10.2f  %-12.4f  %.2f deg\n', ...
            CLt, Vat, Rt, rad2deg(at), dphi, rad2deg(dpsi));

    subplot(1,4,ck);
    plot(tt, rad2deg(Xt(:,9)), 'Color',colors2(ck,:),'LineWidth',2);
    xlabel('Time [s]'); ylabel('\psi [deg]');
    title(sprintf('CL=%.1f\nVa=%.0f ft/s',CLt,Vat),'FontSize',9);
    grid on; box on;
end
sgtitle('4(v): Heading \psi during constant turn (should grow linearly)', ...
        'FontWeight','bold');
saveas(fig_turn,'report_figures/Sec4_v_ConstantTurn.png');
fprintf('\n  Figure saved: report_figures/Sec4_v_ConstantTurn.png\n\n');

%% ============================================================
%  SECTION 4(vi): Transfer functions
% ============================================================
fprintf('--- Section 4(vi): Transfer Functions ---\n\n');

TF = compute_transfer_functions(x_trim, u_trim, P);

fig_bode = figure('Name','4(vi): Bode Plots','Position',[30,30,1300,800]);
tf_titles = {'\phi/\delta_a','\chi/\phi','\beta/\delta_r', ...
             '\theta/\delta_e','V_a/\delta_t','V_a/\theta'};
tf_list   = {TF.phi_da, TF.chi_phi, TF.beta_dr, ...
             TF.theta_de, TF.Va_dt, TF.Va_theta};
for k = 1:6
    subplot(2,3,k); bode(tf_list{k}); title(tf_titles{k},'FontSize',10); grid on;
end
sgtitle('Section 4(vi): Bode Plots of Transfer Functions','FontSize',13,'FontWeight','bold');
saveas(fig_bode,'report_figures/Sec4_vi_BodePlots.png');
fprintf('  Figure saved: report_figures/Sec4_vi_BodePlots.png\n\n');

fprintf('  DC gains:\n');
tf_names = {'phi/da','chi/phi','beta/dr','theta/de','Va/dt','Va/theta'};
for k = 1:6
    [~,gn] = bode(tf_list{k}, 0.001);
    fprintf('    %-12s : %.4f\n', tf_names{k}, gn(1));
end

%% ============================================================
%  SECTION 4(vii): State-space models
%  TWO METHODS:
%    A) Analytical  — compute_ss_models  (Tables 5.1/5.2)
%    B) Linmod      — compute_ss_linmod  (Appendix F.3)  **NEW**
% ============================================================
fprintf('\n--- Section 4(vii): State-Space Models ---\n\n');

%% --- Method A: Analytical ---
fprintf('METHOD A: Analytical (Tables 5.1/5.2)\n');
[SS_lon_an, SS_lat_an] = compute_ss_models(x_trim, u_trim, P);

%% --- Method B: linmod ---
fprintf('\nMETHOD B: Simulink linmod (Appendix F.3)\n');
linmod_available = false;
try
    [SS_lon_lm, SS_lat_lm] = compute_ss_linmod(x_trim, u_trim, P);
    linmod_available = true;
catch ME
    fprintf('  linmod not available: %s\n', ME.message);
    fprintf('  Run build_mavsim_trim.m first to create mavsim_trim.slx\n');
    fprintf('  Using analytical results only.\n\n');
    SS_lon_lm = SS_lon_an;
    SS_lat_lm = SS_lat_an;
end

%% --- Compare eigenvalues ---
fprintf('\n  Eigenvalue comparison (should match):\n');
fprintf('  %-35s  %-35s\n','Analytical','linmod');
ev_an = sort(eig(SS_lon_an.A));
ev_lm = sort(eig(SS_lon_lm.A));
fprintf('  LONGITUDINAL:\n');
for k = 1:length(ev_an)
    fprintf('    %+.4f%+.4fi   vs   %+.4f%+.4fi\n', ...
            real(ev_an(k)),imag(ev_an(k)), real(ev_lm(k)),imag(ev_lm(k)));
end

ev_an = sort(eig(SS_lat_an.A));
ev_lm = sort(eig(SS_lat_lm.A));
fprintf('  LATERAL:\n');
for k = 1:length(ev_an)
    fprintf('    %+.4f%+.4fi   vs   %+.4f%+.4fi\n', ...
            real(ev_an(k)),imag(ev_an(k)), real(ev_lm(k)),imag(ev_lm(k)));
end

%% --- Step response plots ---
fig_step = figure('Name','4(vii): Step Responses','Position',[30,500,1200,400]);
subplot(1,3,1);
step(SS_lon_an(3,1),10); hold on;
if linmod_available, step(SS_lon_lm(3,1),10); legend('Analytical','linmod'); end
title('q/\delta_e  (longitudinal)'); grid on;

subplot(1,3,2);
step(SS_lon_an(4,1),10); hold on;
if linmod_available, step(SS_lon_lm(4,1),10); legend('Analytical','linmod'); end
title('\theta/\delta_e'); grid on;

subplot(1,3,3);
step(SS_lat_an(2,1),10); hold on;
if linmod_available, step(SS_lat_lm(2,1),10); legend('Analytical','linmod'); end
title('p/\delta_a  (lateral)'); grid on;

sgtitle('Section 4(vii): State-Space Step Responses — Analytical vs linmod', ...
        'FontSize',12,'FontWeight','bold');
saveas(fig_step,'report_figures/Sec4_vii_StepResponses.png');
fprintf('\n  Figure saved: report_figures/Sec4_vii_StepResponses.png\n\n');

fprintf('Section 4 complete.\n\n');

%% ---- Extract short-period and phugoid from longitudinal eigenvalues ----
fprintf('=== Modal Analysis ===\n');
lons = eig(SS_lon_an.A);
complex_lons = lons(abs(imag(lons)) > 1e-3);
if length(complex_lons) >= 4
    wns = abs(complex_lons);
    [wns_sorted, idx] = sort(wns,'descend');
    zetas = -real(complex_lons(idx))./wns_sorted;
    fprintf('Short-period: wn=%.3f rad/s  zeta=%.3f  (T=%.1f s)\n', ...
            wns_sorted(1), zetas(1), 2*pi/wns_sorted(1));
    fprintf('Phugoid:      wn=%.3f rad/s  zeta=%.3f  (T=%.1f s)\n', ...
            wns_sorted(end), zetas(end), 2*pi/wns_sorted(end));
end

lats = eig(SS_lat_an.A);
complex_lats = lats(abs(imag(lats)) > 1e-3);
if ~isempty(complex_lats)
    wn_dr  = abs(complex_lats(1));
    zeta_dr = -real(complex_lats(1))/wn_dr;
    fprintf('Dutch roll:   wn=%.3f rad/s  zeta=%.3f\n', wn_dr, zeta_dr);
end
real_lats = sort(real(lats(abs(imag(lats)) < 1e-3)),'ascend');
if length(real_lats) >= 2
    fprintf('Roll mode:    lambda=%.4f\n', real_lats(1));
    fprintf('Spiral:       lambda=%.4f\n', real_lats(end));
end

%% ---- Local EOM ----
function xdot = full_eom(x, delta, wind, P)
    fm=forces_moments(x,delta,wind,P);
    fx=fm(1);fy=fm(2);fz=fm(3);l=fm(4);mm=fm(5);n=fm(6);
    u=x(4);v=x(5);w=x(6);phi=x(7);theta=x(8);psi=x(9);p=x(10);q=x(11);r=x(12);
    cp=cos(phi);sp=sin(phi);ct=cos(theta);st=sin(theta);tt=tan(theta);
    cs=cos(psi);ss=sin(psi);
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