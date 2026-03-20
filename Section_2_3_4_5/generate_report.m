%% generate_report.m
% AE700 - Sections 2, 3, 4 Report Generator
% IIT Bombay | F-4 Phantom
%
% Runs all sections, saves all figures as PNG, and writes
% a text summary with numerical results for report writing.
%
% Run: >> generate_report
% Output: figures saved to ./report_figures/
%         results saved to ./AE700_Results.txt

clear; clc; close all;

%% Setup output folder
if ~exist('report_figures','dir'), mkdir('report_figures'); end
fid = fopen('AE700_Results.txt','w');
fprintf(fid,'AE700 - Guidance and Control of Unmanned Autonomous Vehicles\n');
fprintf(fid,'F-4 Phantom | IIT Bombay\n');
fprintf(fid,'%s\n\n', datestr(now));

tee = @(varargin) fprintf_tee(fid, varargin{:});

%% ============================================================
%  SECTION 2
% ============================================================
tee('=======================================================\n');
tee('SECTION 2: KINEMATICS AND DYNAMICS\n');
tee('=======================================================\n\n');

F4_Phantom_params;
ac.mass=mass; ac.Jx=I_xx; ac.Jy=I_yy; ac.Jz=I_zz; ac.Jxz=I_xz;

x0=zeros(12,1); x0(3)=-300;
opts=odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',0.05);

% --- 2(ii) Individual axis tests ---
tee('--- 2(ii): Individual Force/Moment Axis Tests ---\n\n');
fm_tests = {[5000,0,0,0,0,0],'fx=5000 lb (thrust)',[4,11],'u','q';
            [0,800,0,0,0,0], 'fy=800 lb (side)',   [5,9], 'v','psi';
            [0,0,800,0,0,0], 'fz=800 lb (down)',   [6,3], 'w','pd';
            [0,0,0,8000,0,0],'l=8000 (roll)',       [10,7],'p','phi';
            [0,0,0,0,8000,0],'m=8000 (pitch)',      [11,8],'q','theta';
            [0,0,0,0,0,8000],'n=8000 (yaw)',        [12,9],'r','psi'};

fig2 = figure('Position',[0,0,1400,800],'Visible','off');
for k=1:6
    fm=fm_tests{k,1}'; lbl=fm_tests{k,2}; si=fm_tests{k,3}; s1=fm_tests{k,4}; s2=fm_tests{k,5};
    [t,X]=ode45(@(t,x) eom2(x,fm,ac),[0,15],x0,opts);
    subplot(2,3,k);
    yyaxis left;  plot(t,X(:,si(1)),'b-','LineWidth',2); ylabel(s1);
    yyaxis right; plot(t,X(:,si(2)),'r--','LineWidth',2); ylabel(s2);
    xlabel('Time [s]'); title(lbl,'FontSize',9,'FontWeight','bold'); grid on;
    tee('  Input: %s\n', lbl);
    tee('    %s(15s) = %+.4f  |  %s(15s) = %+.4f\n\n', s1,X(end,si(1)), s2,X(end,si(2)));
end
sgtitle('Section 2(ii): Individual Axis Tests','FontSize',12,'FontWeight','bold');
saveas(fig2,'report_figures/Sec2_ii_AxisTests.png');
tee('  Figure saved: Sec2_ii_AxisTests.png\n\n');

% --- 2(iii) Jxz coupling ---
tee('--- 2(iii): Jxz Gyroscopic Coupling ---\n\n');
fm_c=[0;0;0;3000;0;3000];
ac0=ac; ac0.Jxz=0;
[tA,XA]=ode45(@(t,x) eom2(x,fm_c,ac0),[0,10],x0,opts);
[tB,XB]=ode45(@(t,x) eom2(x,fm_c,ac), [0,10],x0,opts);

fig3 = figure('Position',[0,0,1000,400],'Visible','off');
subplot(1,2,1);
plot(tA,rad2deg(XA(:,10)),'b-','LineWidth',2,'DisplayName','p'); hold on;
plot(tA,rad2deg(XA(:,12)),'r--','LineWidth',2,'DisplayName','r');
xlabel('Time [s]'); ylabel('Rate [deg/s]'); title('Jxz=0 (no coupling)'); legend; grid on;
subplot(1,2,2);
plot(tB,rad2deg(XB(:,10)),'b-','LineWidth',2,'DisplayName','p'); hold on;
plot(tB,rad2deg(XB(:,12)),'r--','LineWidth',2,'DisplayName','r');
xlabel('Time [s]'); ylabel('Rate [deg/s]'); title(sprintf('Jxz=%.0f (coupled)',ac.Jxz)); legend; grid on;
sgtitle('Section 2(iii): Jxz Coupling','FontSize',12,'FontWeight','bold');
saveas(fig3,'report_figures/Sec2_iii_JxzCoupling.png');

tee('  Jxz = 0:\n');
tee('    p(10s) = %.4f rad/s  |  r(10s) = %.4f rad/s  -> INDEPENDENT\n', XA(end,10),XA(end,12));
tee('  Jxz = %g slug-ft^2:\n', ac.Jxz);
tee('    p(10s) = %.4f rad/s  |  r(10s) = %.4f rad/s  -> COUPLED (different)\n', XB(end,10),XB(end,12));
tee('  Coupling arises from Gamma1 and Gamma4 terms in eq (3.17)\n');
tee('  Figure saved: Sec2_iii_JxzCoupling.png\n\n');

%% ============================================================
%  SECTION 3
% ============================================================
tee('=======================================================\n');
tee('SECTION 3: FORCES AND MOMENTS\n');
tee('=======================================================\n\n');

F4_chap4_params;
x_trim=zeros(12,1); x_trim(3)=-1000;
de_trim=-(P.Cmo+P.Cm_a*P.alpha_trim)/P.Cm_de;
dt_trim=0.5;
delta_trim=[de_trim;0;0;dt_trim];
wind_zero=zeros(6,1);

% --- 3(i) Trim forces ---
tee('--- 3(i): forces_moments.m output at trim ---\n');
fo=forces_moments(x_trim,delta_trim,wind_zero,P);
tee('  Va=%.2f ft/s  alpha=%.3f deg  beta=%.4f deg\n', fo(7),rad2deg(fo(8)),rad2deg(fo(9)));
tee('  fx=%.2f lb  fy=%.2f lb  fz=%.2f lb\n', fo(1),fo(2),fo(3));
tee('  l=%.2f  m=%.2f  n=%.2f lb.ft\n\n', fo(4),fo(5),fo(6));

% --- 3(ii) Wind gust ---
tee('--- 3(ii): Wind gust response ---\n');
xw=zeros(12,1); xw(3)=-1000; xw(4)=P.Va_trim; xw(8)=P.alpha_trim;
[tw,Xw]=ode45(@(t,x) full_eom3(x,delta_trim,[0;0;0;50;0;0],P),[0,20],xw,opts);
Va_h=arrayfun(@(k) forces_moments(Xw(k,:)',delta_trim,[0;0;0;50;0;0],P), 1:length(tw),'UniformOutput',false);
Va_arr=cellfun(@(f) f(7), Va_h)';
al_arr=cellfun(@(f) f(8), Va_h)';

fig4=figure('Position',[0,0,900,500],'Visible','off');
subplot(2,1,1); plot(tw,Va_arr,'b-','LineWidth',2); ylabel('Va [ft/s]'); grid on; title('Airspeed');
subplot(2,1,2); plot(tw,rad2deg(al_arr),'r-','LineWidth',2); ylabel('\alpha [deg]'); xlabel('Time [s]'); grid on; title('Angle of Attack');
sgtitle('Section 3(ii): Head-on Gust Response','FontSize',12,'FontWeight','bold');
saveas(fig4,'report_figures/Sec3_ii_GustResponse.png');
tee('  Head-on gust ug=50 ft/s: Va initial=%.1f, final=%.1f ft/s\n', Va_arr(1),Va_arr(end));
tee('  alpha initial=%.2f, final=%.2f deg\n', rad2deg(al_arr(1)),rad2deg(al_arr(end)));
tee('  Figure saved: Sec3_ii_GustResponse.png\n\n');

% --- 3(iii) Control surface tests ---
tee('--- 3(iii): Control surface deflection tests ---\n\n');
cs_cases={[de_trim+deg2rad(10);0;0;dt_trim],[8,11],'theta','q','Elev+10: theta/q increase';
          [de_trim;deg2rad(15);0;dt_trim],  [7,10],'phi','p','Ail+15: phi/p increase';
          [de_trim;0;deg2rad(10);dt_trim],  [9,12],'psi','r','Rudd+10: psi/r increase';
          [de_trim;0;0;0.8],                [4,3], 'u','pd','Thr 0.8: u increase';};

fig5=figure('Position',[0,0,1200,700],'Visible','off');
for k=1:4
    delta=cs_cases{k,1}; si=cs_cases{k,2}; s1=cs_cases{k,3}; s2=cs_cases{k,4}; lbl=cs_cases{k,5};
    [t,X]=ode45(@(t,x) full_eom3(x,delta,wind_zero,P),[0,20],x_trim,opts);
    subplot(2,2,k);
    yyaxis left; plot(t,X(:,si(1)),'b-','LineWidth',2); ylabel(s1);
    yyaxis right; plot(t,X(:,si(2)),'r--','LineWidth',2); ylabel(s2);
    xlabel('Time [s]'); title(lbl,'FontSize',9,'FontWeight','bold'); grid on;
    tee('  %s:  %s(20s)=%+.4f  |  %s(20s)=%+.4f\n', lbl, s1,X(end,si(1)), s2,X(end,si(2)));
end
sgtitle('Section 3(iii): Control Surface Tests','FontSize',12,'FontWeight','bold');
saveas(fig5,'report_figures/Sec3_iii_ControlSurface.png');
tee('  Figure saved: Sec3_iii_ControlSurface.png\n\n');

%% ============================================================
%  SECTION 4
% ============================================================
tee('=======================================================\n');
tee('SECTION 4: LINEAR DESIGN MODELS\n');
tee('=======================================================\n\n');

% --- 4(iii)+(iv) Trim ---
tee('--- 4(iii)+(iv): Trim computation ---\n\n');
[x_trim4, u_trim4] = compute_trim(P.Va_trim, 0, Inf, P);
x_trim4(3) = -1000;
[tv,Xv]=ode45(@(t,x) full_eom3(x,u_trim4,wind_zero,P),[0,30],x_trim4,opts);

tee('  Trim state (Va=%.0f ft/s, gamma=0, R=Inf):\n', P.Va_trim);
tee('    alpha = %.3f deg  |  theta = %.3f deg\n', rad2deg(atan2(x_trim4(6),x_trim4(4))), rad2deg(x_trim4(8)));
tee('    de=%.4f rad  da=%.4f rad  dr=%.4f rad  dt=%.4f\n\n', u_trim4(1),u_trim4(2),u_trim4(3),u_trim4(4));
tee('  30s simulation state variation (should be ~0):\n');
for i=[4,7,8,11]
    tee('    %-8s: max variation = %.2e\n', state_nm(i), max(Xv(:,i))-min(Xv(:,i)));
end
tee('\n');

% Gamma sweep
tee('  Gamma sweep (climb rate check):\n');
tee('  %-12s  %-18s  %-18s  %s\n','gamma[deg]','h_dot expected','h_dot simulated','error');
for gv=[-5,-2,0,2,5]*pi/180
    [xt,ut]=compute_trim(P.Va_trim,gv,Inf,P); xt(3)=-1000;
    xd=full_eom3(xt,ut,wind_zero,P);
    he=P.Va_trim*sin(gv); hs=-xd(3);
    tee('  %+12.1f  %+18.4f  %+18.4f  %.4f\n', rad2deg(gv),he,hs,abs(he-hs));
end
tee('\n');

% --- 4(v) Constant turn ---
tee('--- 4(v): Constant turn trim (n=1.2) ---\n\n');
phi_turn=acos(1/1.2);
tee('  n=1.2 -> phi=%.2f deg\n\n', rad2deg(phi_turn));
tee('  %-5s  %-10s  %-10s  %-12s  %s\n','CL','Va [ft/s]','R [ft]','psi_change','Status');
for CLt=[0.7,0.8,0.9,1.0]
    Vat=sqrt(2*P.mass*P.g*1.2/(P.rho*P.Sw*CLt));
    Rt=Vat^2/(P.g*tan(phi_turn));
    [xt,ut]=compute_trim(Vat,0,Rt,P); xt(3)=-1000;
    [~,Xt]=ode45(@(t,x) full_eom3(x,ut,wind_zero,P),[0,20],xt,opts);
    dpsi=Xt(end,9)-Xt(1,9);
    dphi=max(Xt(:,7))-min(Xt(:,7));
    status = 'OK';
    if dphi > 0.01, status = 'phi drifting'; end
    tee('  %-5.1f  %-10.1f  %-10.0f  %-12.2f  %s\n', CLt,Vat,Rt,rad2deg(dpsi),status);
end
tee('\n');

% --- 4(vi) Transfer functions ---
tee('--- 4(vi): Transfer functions ---\n\n');
TF = compute_transfer_functions(x_trim4, u_trim4, P);

fig6=figure('Position',[0,0,1300,800],'Visible','off');
tf_titles={'\phi/\delta_a','\chi/\phi','\beta/\delta_r','\theta/\delta_e','V_a/\delta_t','V_a/\theta'};
tf_list={TF.phi_da,TF.chi_phi,TF.beta_dr,TF.theta_de,TF.Va_dt,TF.Va_theta};
for k=1:6; subplot(2,3,k); bode(tf_list{k}); title(tf_titles{k},'FontSize',10); grid on; end
sgtitle('Section 4(vi): Bode Plots','FontSize',12,'FontWeight','bold');
saveas(fig6,'report_figures/Sec4_vi_BodePlots.png');

tee('  Transfer function DC gains:\n');
tf_names={'phi/da','chi/phi','beta/dr','theta/de','Va/dt','Va/theta'};
for k=1:6
    [~,gn]=bode(tf_list{k},0.001);
    tee('    %-12s : DC gain = %.4f\n', tf_names{k}, gn(1));
end
tee('  Figure saved: Sec4_vi_BodePlots.png\n\n');

% --- 4(vii) State-space models ---
tee('--- 4(vii): State-space models ---\n\n');
[SS_lon, SS_lat] = compute_ss_models(x_trim4, u_trim4, P);

fig7=figure('Position',[0,0,1200,400],'Visible','off');
subplot(1,3,1); step(SS_lon(3,1),10); title('q/\delta_e'); grid on;
subplot(1,3,2); step(SS_lon(4,1),10); title('\theta/\delta_e'); grid on;
subplot(1,3,3); step(SS_lat(2,1),10); title('p/\delta_a'); grid on;
sgtitle('Section 4(vii): State-Space Step Responses','FontSize',12,'FontWeight','bold');
saveas(fig7,'report_figures/Sec4_vii_StepResponses.png');

tee('  Longitudinal eigenvalues:\n');
for lam=eig(SS_lon.A).'
    if abs(imag(lam))>1e-4, wn=abs(lam); zeta=-real(lam)/wn;
        tee('    %+.4f%+.4fi  wn=%.3f rad/s  zeta=%.3f\n',real(lam),imag(lam),wn,zeta);
    else, tee('    %+.6f\n',real(lam)); end
end
tee('\n  Lateral eigenvalues:\n');
for lam=eig(SS_lat.A).'
    if abs(imag(lam))>1e-4, tee('    %+.4f%+.4fi  (Dutch roll)\n',real(lam),imag(lam));
    elseif real(lam)>1e-4,   tee('    %+.4f  (Spiral — unstable)\n',real(lam));
    elseif abs(real(lam))<1e-4, tee('    %+.6f  (heading integrator)\n',real(lam));
    else, tee('    %+.4f  (Roll mode)\n',real(lam)); end
end
tee('\n  Figures saved: Sec4_vii_StepResponses.png\n\n');

fclose(fid);
fprintf('\n=== Report generation complete ===\n');
fprintf('  Figures : ./report_figures/ (%d PNG files)\n', length(dir('report_figures/*.png')));
fprintf('  Summary : AE700_Results.txt\n\n');

%% ---- Local functions ----
function xdot = eom2(x,fm,ac)
    mass=ac.mass;Jx=ac.Jx;Jy=ac.Jy;Jz=ac.Jz;Jxz=ac.Jxz;
    u=x(4);v=x(5);w=x(6);phi=x(7);theta=x(8);psi=x(9);p=x(10);q=x(11);r=x(12);
    fx=fm(1);fy=fm(2);fz=fm(3);l=fm(4);mm=fm(5);n=fm(6);
    cp=cos(phi);sp=sin(phi);ct=cos(theta);st=sin(theta);tt=tan(theta);cs=cos(psi);ss=sin(psi);
    G=Jx*Jz-Jxz^2;G1=Jxz*(Jx-Jy+Jz)/G;G2=(Jz*(Jz-Jy)+Jxz^2)/G;G3=Jz/G;G4=Jxz/G;
    G5=(Jz-Jx)/Jy;G6=Jxz/Jy;G7=((Jx-Jy)*Jx+Jxz^2)/G;G8=Jx/G;
    pnd=(ct*cs)*u+(sp*st*cs-cp*ss)*v+(cp*st*cs+sp*ss)*w;
    ped=(ct*ss)*u+(sp*st*ss+cp*cs)*v+(cp*st*ss-sp*cs)*w;
    pdd=(-st)*u+(sp*ct)*v+(cp*ct)*w;
    ud=r*v-q*w+fx/mass;vd=p*w-r*u+fy/mass;wd=q*u-p*v+fz/mass;
    phd=p+(sp*tt)*q+(cp*tt)*r;thd=cp*q-sp*r;psd=(sp/ct)*q+(cp/ct)*r;
    pd_=G1*p*q-G2*q*r+G3*l+G4*n;qd=G5*p*r-G6*(p^2-r^2)+mm/Jy;rd=G7*p*q-G1*q*r+G4*l+G8*n;
    xdot=[pnd;ped;pdd;ud;vd;wd;phd;thd;psd;pd_;qd;rd];
end

function xdot = full_eom3(x,delta,wind,P)
    fm=forces_moments(x,delta,wind,P);
    fx=fm(1);fy=fm(2);fz=fm(3);l=fm(4);mm=fm(5);n=fm(6);
    u=x(4);v=x(5);w=x(6);phi=x(7);theta=x(8);psi=x(9);p=x(10);q=x(11);r=x(12);
    cp=cos(phi);sp=sin(phi);ct=cos(theta);st=sin(theta);tt=tan(theta);cs=cos(psi);ss=sin(psi);
    G=P.Jx*P.Jz-P.Jxz^2;G1=P.Jxz*(P.Jx-P.Jy+P.Jz)/G;G2=(P.Jz*(P.Jz-P.Jy)+P.Jxz^2)/G;
    G3=P.Jz/G;G4=P.Jxz/G;G5=(P.Jz-P.Jx)/P.Jy;G6=P.Jxz/P.Jy;
    G7=((P.Jx-P.Jy)*P.Jx+P.Jxz^2)/G;G8=P.Jx/G;
    pnd=(ct*cs)*u+(sp*st*cs-cp*ss)*v+(cp*st*cs+sp*ss)*w;
    ped=(ct*ss)*u+(sp*st*ss+cp*cs)*v+(cp*st*ss-sp*cs)*w;
    pdd=(-st)*u+(sp*ct)*v+(cp*ct)*w;
    ud=r*v-q*w+fx/P.mass;vd=p*w-r*u+fy/P.mass;wd=q*u-p*v+fz/P.mass;
    phd=p+(sp*tt)*q+(cp*tt)*r;thd=cp*q-sp*r;psd=(sp/ct)*q+(cp/ct)*r;
    pd_=G1*p*q-G2*q*r+G3*l+G4*n;qd=G5*p*r-G6*(p^2-r^2)+mm/P.Jy;rd=G7*p*q-G1*q*r+G4*l+G8*n;
    xdot=[pnd;ped;pdd;ud;vd;wd;phd;thd;psd;pd_;qd;rd];
end

function s = state_nm(i)
    names={'pn','pe','pd','u','v','w','phi','theta','psi','p','q','r'};
    s=names{i};
end

function fprintf_tee(fid, varargin)
    str = sprintf(varargin{:});
    fprintf('%s', str);
    fprintf(fid, '%s', str);
end
