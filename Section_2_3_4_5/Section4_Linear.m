%% Section4_Linear.m
% AE700 - Section 4: Linear Design Models
% IIT Bombay | F-4 Phantom
%
% Covers:
%   (iii) Trim computation: compute_trim(Va, gamma, R, P)
%   (iv)  Wings-level trim verification + gamma sweep
%   (v)   Constant turn trim (n=1.2, CL=0.7-1.0)
%   (vi)  Transfer functions (Section 5.4)
%   (vii) State-space models (eq 5.43, 5.50)
%
% Dependencies: F4_chap4_params.m, forces_moments.m,
%               compute_trim.m, compute_transfer_functions.m,
%               compute_ss_models.m
% Run: >> Section4_Linear

clear; clc; close all;
F4_chap4_params;   % loads struct P

fprintf('=====================================================\n');
fprintf('  AE700 Section 4 - Linear Design Models\n');
fprintf('  F-4 Phantom\n');
fprintf('=====================================================\n\n');

opts = odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',0.05);
wind = zeros(6,1);

%% ============================================================
%  SECTION 4(iii)+(iv): Wings-level trim  gamma=0, R=Inf
% ============================================================
fprintf('--- Section 4(iii)+(iv): Wings-level trim ---\n\n');

Va    = P.Va_trim;
gamma = 0;
R     = Inf;

[x_trim, u_trim] = compute_trim(Va, gamma, R, P);
x_trim(3) = -1000;   % set altitude to 1000 ft

% Verify: simulate for 30s, states should stay constant
[tv, Xv] = ode45(@(t,x) full_eom(x,u_trim,wind,P), [0,30], x_trim, opts);

fprintf('\n  Trim state verification (30s simulation):\n');
fprintf('  %-10s  %-12s  %-12s  %s\n','State','Initial','Final','Variation');
state_names={'pn','pe','pd','u','v','w','phi','theta','psi','p','q','r'};
for i=[4,5,6,7,8,10,11,12]
    fprintf('  %-10s  %+12.6f  %+12.6f  %.2e\n', state_names{i}, Xv(1,i), Xv(end,i), max(Xv(:,i))-min(Xv(:,i)));
end

%% ---- Gamma sweep: verify climb rate ----
fprintf('\n  Gamma sweep (climb rate verification):\n');
fprintf('  %-12s  %-16s  %-16s  %s\n','gamma [deg]','h_dot expected','h_dot simulated','error');

gamma_vals = [-5,-2,0,2,5]*pi/180;
trim_results = struct();

for gk=1:length(gamma_vals)
    gv = gamma_vals(gk);
    [xt,ut] = compute_trim(Va, gv, Inf, P);
    xt(3) = -1000;
    xdot0 = full_eom(xt, ut, wind, P);
    hdot_expected = Va*sin(gv);
    hdot_sim      = -xdot0(3);
    fprintf('  %+12.1f  %+16.3f  %+16.3f  %.4f\n', rad2deg(gv), hdot_expected, hdot_sim, abs(hdot_expected-hdot_sim));
    trim_results(gk).gamma=gv; trim_results(gk).x=xt; trim_results(gk).u=ut;
    trim_results(gk).hdot_exp=hdot_expected; trim_results(gk).hdot_sim=hdot_sim;
end

%% ============================================================
%  SECTION 4(v): Constant turn trim (n=1.2, CL=0.7-1.0)
% ============================================================
fprintf('\n--- Section 4(v): Constant turn trim ---\n\n');

n_load   = 1.2;
phi_turn = acos(1/n_load);
fprintf('  n=%.1f -> bank angle phi=%.2f deg\n\n', n_load, rad2deg(phi_turn));
fprintf('  %-6s  %-10s  %-10s  %-10s  %-12s  %-12s  %s\n','CL','Va [ft/s]','R [ft]','alpha [deg]','phi_var','theta_var','psi_change');

CL_vals = [0.7, 0.8, 0.9, 1.0];
turn_results = struct();

for ck=1:length(CL_vals)
    CLt  = CL_vals(ck);
    Vat  = sqrt(2*P.mass*P.g*n_load/(P.rho*P.Sw*CLt));
    Rt   = Vat^2 / (P.g*tan(phi_turn));
    [xt,ut] = compute_trim(Vat, 0, Rt, P);
    xt(3) = -1000;
    [tt,Xt] = ode45(@(t,x) full_eom(x,ut,wind,P), [0,20], xt, opts);
    dphi   = max(Xt(:,7))-min(Xt(:,7));
    dtheta = max(Xt(:,8))-min(Xt(:,8));
    dpsi   = Xt(end,9)-Xt(1,9);
    at     = atan2(xt(6),xt(4));
    fprintf('  %-6.1f  %-10.1f  %-10.0f  %-10.2f  %-12.4f  %-12.4f  %.2f deg\n', CLt,Vat,Rt,rad2deg(at),dphi,dtheta,rad2deg(dpsi));
    turn_results(ck).CL=CLt; turn_results(ck).Va=Vat; turn_results(ck).R=Rt;
    turn_results(ck).x=xt;   turn_results(ck).u=ut;   turn_results(ck).dpsi=dpsi;
end

%% ============================================================
%  SECTION 4(vi): Transfer functions (Section 5.4)
% ============================================================
fprintf('\n--- Section 4(vi): Transfer Functions ---\n\n');
TF = compute_transfer_functions(x_trim, u_trim, P);

fig_bode = figure('Name','Section 4(vi): Bode Plots','NumberTitle','off','Position',[30,30,1300,800]);
titles_bode = {'\phi/\delta_a (Roll/Aileron)', '\chi/\phi (Course/Roll)', '\beta/\delta_r (Sideslip/Rudder)', '\theta/\delta_e (Pitch/Elevator)', 'V_a/\delta_t (Speed/Throttle)', 'V_a/\theta (Speed/Pitch)'};
tf_list = {TF.phi_da, TF.chi_phi, TF.beta_dr, TF.theta_de, TF.Va_dt, TF.Va_theta};
for k=1:6
    subplot(2,3,k); bode(tf_list{k}); title(titles_bode{k},'FontSize',10); grid on;
end
sgtitle('Section 4(vi): Bode Plots of Transfer Functions','FontSize',13,'FontWeight','bold');

%% ============================================================
%  SECTION 4(vii): State-space models (eq 5.43, 5.50)
% ============================================================
fprintf('\n--- Section 4(vii): State-Space Models ---\n\n');
[SS_lon, SS_lat] = compute_ss_models(x_trim, u_trim, P);

fprintf('\nA_lon =\n'); disp(SS_lon.A);
fprintf('B_lon =\n');   disp(SS_lon.B);
fprintf('A_lat =\n');   disp(SS_lat.A);
fprintf('B_lat =\n');   disp(SS_lat.B);

fig_step = figure('Name','Section 4(vii): Step Responses','NumberTitle','off','Position',[30,500,1200,400]);
subplot(1,3,1); step(SS_lon(3,1),10); title('q to \delta_e (longitudinal)'); grid on;
subplot(1,3,2); step(SS_lon(4,1),10); title('\theta to \delta_e');            grid on;
subplot(1,3,3); step(SS_lat(2,1),10); title('p to \delta_a (lateral)');       grid on;
sgtitle('Section 4(vii): State-Space Step Responses','FontSize',13,'FontWeight','bold');

fprintf('\nSection 4 complete.\n');

%% ---- Local EOM function ----
function xdot = full_eom(x, delta, wind, P)
    fm=forces_moments(x,delta,wind,P);
    fx=fm(1);fy=fm(2);fz=fm(3); l=fm(4);mm=fm(5);n=fm(6);
    u=x(4);v=x(5);w=x(6); phi=x(7);theta=x(8);psi=x(9); p=x(10);q=x(11);r=x(12);
    cp=cos(phi);sp=sin(phi); ct=cos(theta);st=sin(theta);tt=tan(theta); cs=cos(psi);ss=sin(psi);
    G=P.Jx*P.Jz-P.Jxz^2; G1=P.Jxz*(P.Jx-P.Jy+P.Jz)/G; G2=(P.Jz*(P.Jz-P.Jy)+P.Jxz^2)/G;
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
