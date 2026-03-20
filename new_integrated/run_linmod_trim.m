%% run_linmod_trim.m
% Section 4(ii)(iii)(iv)(vii) - FIXED VERSION
%
% Uses MATLAB's built-in  trim()  and  linmod()  commands on mavsim_trim.slx,
% as explicitly required by the project statement and Appendix F.
%
% Also calls compute_trim.m (analytical / fminsearch) for comparison.
%
% Outputs:
%   x_trim, u_trim   - trim state and controls
%   SS_lon, SS_lat   - longitudinal & lateral state-space models (linmod)
%   A_lon, B_lon     - extracted from full linmod result
%   A_lat, B_lat     - extracted from full linmod result
%
% Run: >> run_linmod_trim
% Prerequisite: build_mavsim_trim.m must have been run first.

clear; clc; close all;
F4_chap4_params;   % loads P

fprintf('=====================================================\n');
fprintf('  AE700 Section 4 - Trim + linmod (FIXED)\n');
fprintf('  F-4 Phantom  Va=%.0f ft/s  alpha~%.1f deg\n', ...
        P.Va_trim, rad2deg(P.alpha_trim));
fprintf('=====================================================\n\n');

if ~exist('mavsim_trim.slx','file')
    fprintf('mavsim_trim.slx not found. Running build_mavsim_trim...\n');
    build_mavsim_trim;
end
load_system('mavsim_trim');

Va_star    = P.Va_trim;
gamma_star = 0;
R_star     = Inf;
opts_ode   = odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',0.05);
wind_zero  = zeros(6,1);

%% ============================================================
%  4(iii): Analytical trim (compute_trim.m)
% ============================================================
fprintf('--- 4(iii): Analytical trim ---\n');
[x_trim, u_trim] = compute_trim(Va_star, gamma_star, R_star, P);
x_trim(3) = -1000;

alpha_t = atan2(x_trim(6), x_trim(4));
fprintf('  Va    = %.2f ft/s\n', sqrt(sum(x_trim(4:6).^2)));
fprintf('  alpha = %.3f deg  (target: 5-7 deg)\n', rad2deg(alpha_t));
fprintf('  theta = %.3f deg\n', rad2deg(x_trim(8)));
fprintf('  de=%.4f rad  da=%.4f rad  dr=%.4f rad  dt=%.4f\n\n', u_trim);

%% ============================================================
%  4(iv): Verify trim — 30s simulation, states stay constant
% ============================================================
fprintf('--- 4(iv): Trim verification (30s) ---\n');
[tv,Xv] = ode45(@(t,x) feom(x,u_trim,wind_zero,P),[0,30],x_trim,opts_ode);

state_names = {'pn','pe','pd','u','v','w','phi','theta','psi','p','q','r'};
fprintf('  %-8s  %-12s  %-12s  %s\n','State','Initial','Final','Variation');
for i = [4,5,6,7,8,10,11,12]
    fprintf('  %-8s  %+12.6f  %+12.6f  %.2e\n', state_names{i}, ...
            Xv(1,i), Xv(end,i), max(Xv(:,i))-min(Xv(:,i)));
end

% Check h_dot for gamma sweep
fprintf('\n  Gamma sweep:\n');
fprintf('  %-10s  %-16s  %-16s  %s\n','gamma[deg]','h_dot expected','h_dot simulated','error');
for gv = [-5,-2,0,2,5]*pi/180
    [xt,ut] = compute_trim(Va_star,gv,Inf,P);
    xt(3) = -1000;
    xd = feom(xt,ut,wind_zero,P);
    he = Va_star*sin(gv); hs = -xd(3);
    fprintf('  %+10.1f  %+16.4f  %+16.4f  %.4f\n', rad2deg(gv),he,hs,abs(he-hs));
end

%% ============================================================
%  4(v): Constant turn trim  n=1.2, CL=0.7-1.0
% ============================================================
fprintf('\n--- 4(v): Constant turn trim  n=1.2 ---\n');
n_load   = 1.2;
phi_turn = acos(1/n_load);
fprintf('  Bank angle for n=%.1f: phi=%.2f deg\n\n', n_load, rad2deg(phi_turn));
fprintf('  %-5s  %-10s  %-10s  %-10s  %-12s\n','CL','Va[ft/s]','R[ft]','alpha[deg]','psi_change[deg]');

for CLt = [0.7,0.8,0.9,1.0]
    Vat = sqrt(2*P.mass*P.g*n_load/(P.rho*P.Sw*CLt));
    Rt  = Vat^2/(P.g*tan(phi_turn));
    [xt,ut] = compute_trim(Vat,0,Rt,P);
    xt(3) = -1000;
    [~,Xt] = ode45(@(t,x) feom(x,ut,wind_zero,P),[0,20],xt,opts_ode);
    dpsi = rad2deg(Xt(end,9)-Xt(1,9));
    at   = rad2deg(atan2(xt(6),xt(4)));
    fprintf('  %-5.1f  %-10.1f  %-10.0f  %-10.2f  %-12.2f\n',CLt,Vat,Rt,at,dpsi);
end

%% ============================================================
%  4(vii): State-space models using linmod  (PROJECT REQUIREMENT)
% ============================================================
fprintf('\n--- 4(vii): State-space models via linmod ---\n\n');

% Set initial state and control for linmod operating point
set_param('mavsim_trim','StopTime','60');

% linmod linearises the Simulink model at (x_trim, u_trim)
% x_trim is the 12-state vector; u_trim is [de,da,dr,dt]
fprintf('  Running linmod(''mavsim_trim'', x_trim, u_trim) ...\n');
try
    [A_full, B_full, C_full, D_full] = linmod('mavsim_trim', x_trim, u_trim);
    fprintf('  linmod succeeded. A is %dx%d.\n\n', size(A_full,1),size(A_full,2));
catch ME
    fprintf('  WARNING: linmod failed: %s\n', ME.message);
    fprintf('  Falling back to analytical state-space.\n\n');
    [SS_lon_a, SS_lat_a] = compute_ss_models(x_trim, u_trim, P);
    SS_lon = SS_lon_a;  SS_lat = SS_lat_a;
    A_full = []; B_full = [];
    goto_analytical = true;
end

if ~isempty(A_full)
    % ---- Extract longitudinal sub-system ----
    % Longitudinal states: u(4), w(6), q(11), theta(8), h=-pd(3)
    % Lateral    states:   v(5), p(10), r(12), phi(7), psi(9)
    lon_idx = [4,6,11,8,3];   % indices in state vector
    lat_idx = [5,10,12,7,9];
    ctrl_lon = [1,4];          % de=1, dt=4  in u_trim
    ctrl_lat = [2,3];          % da=2, dr=3

    A_lon = A_full(lon_idx, lon_idx);
    B_lon = B_full(lon_idx, ctrl_lon);
    A_lat = A_full(lat_idx, lat_idx);
    B_lat = B_full(lat_idx, ctrl_lat);

    % Sign fix: h = -pd, so row/col for pd need sign flip
    % pd is 3rd in lon_idx (position 3 in lon_idx corresponds to state 3=pd)
    h_pos = 3;   % position of pd in lon_idx
    A_lon(h_pos,:) = -A_lon(h_pos,:);
    A_lon(:,h_pos) = -A_lon(:,h_pos);
    B_lon(h_pos,:) = -B_lon(h_pos,:);

    SS_lon = ss(A_lon, B_lon, eye(5), zeros(5,2));
    SS_lon.StateName  = {'u','w','q','theta','h'};
    SS_lon.InputName  = {'delta_e','delta_t'};

    SS_lat = ss(A_lat, B_lat, eye(5), zeros(5,2));
    SS_lat.StateName  = {'v','p','r','phi','psi'};
    SS_lat.InputName  = {'delta_a','delta_r'};

    fprintf('A_lon (from linmod):\n'); disp(A_lon);
    fprintf('B_lon (from linmod):\n'); disp(B_lon);
    fprintf('A_lat (from linmod):\n'); disp(A_lat);
    fprintf('B_lat (from linmod):\n'); disp(B_lat);

    fprintf('Longitudinal eigenvalues:\n');
    for lam = eig(A_lon).'
        if abs(imag(lam))>1e-4
            wn=abs(lam); z=-real(lam)/wn;
            tag=''; if wn>1,'short period'; else; tag='phugoid'; end
            fprintf('  %+.4f%+.4fi  wn=%.3f rad/s  zeta=%.3f  [%s]\n',...
                    real(lam),imag(lam),wn,z,tag);
        else
            fprintf('  %+.6f\n',real(lam));
        end
    end

    fprintf('\nLateral eigenvalues:\n');
    for lam = eig(A_lat).'
        if abs(imag(lam))>1e-4
            fprintf('  %+.4f%+.4fi  (Dutch roll)\n',real(lam),imag(lam));
        elseif real(lam)>1e-4
            fprintf('  %+.4f  (spiral - unstable)\n',real(lam));
        elseif abs(real(lam))<1e-4
            fprintf('  %+.6f  (heading integrator)\n',real(lam));
        else
            fprintf('  %+.4f  (roll mode)\n',real(lam));
        end
    end
end

%% ---- Also compute analytically for comparison ----
fprintf('\n--- Analytical state-space (Tables 5.1/5.2) for comparison ---\n');
[SS_lon_a, SS_lat_a] = compute_ss_models(x_trim, u_trim, P);

%% ---- Step response plots ----
figure('Name','4(vii) Step Responses','Position',[50,50,1200,450]);
subplot(1,3,1); step(SS_lon(3,1),20); title('q / \delta_e  (pitch rate)'); grid on;
subplot(1,3,2); step(SS_lon(4,1),20); title('\theta / \delta_e  (pitch angle)'); grid on;
subplot(1,3,3); step(SS_lat(2,1),20); title('p / \delta_a  (roll rate)');  grid on;
sgtitle('Section 4(vii): State-Space Step Responses (linmod)','FontSize',13,'FontWeight','bold');
saveas(gcf,'report_figures/Sec4_vii_linmod_StepResponses.png');
fprintf('\nFigure saved: Sec4_vii_linmod_StepResponses.png\n');

close_system('mavsim_trim',0);
fprintf('\nSection 4 (linmod) complete.\n');

%% ---- Local EOM ----
function xdot = feom(x,d,wind,P)
    fm=forces_moments(x,d,wind,P);
    fx=fm(1);fy=fm(2);fz=fm(3);l=fm(4);mm=fm(5);n=fm(6);
    u=x(4);v=x(5);w=x(6);phi=x(7);theta=x(8);psi=x(9);p=x(10);q=x(11);r=x(12);
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
