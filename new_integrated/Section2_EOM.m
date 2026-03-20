%% Section2_EOM.m
% =========================================================
% AE700 — Section 2: Kinematics and Dynamics
% IIT Bombay | F-4 Phantom
%
% Covers:
%   (ii)  Individual force/moment axis tests (verify EOM)
%   (iii) Jxz gyroscopic coupling demonstration
%         With Jxz=0: p and r respond independently
%         With Jxz≠0: cross-coupling via Gamma1, Gamma4
%
% Run: >> Section2_EOM
% =========================================================

clear; clc; close all;
F4_params;   % loads P

if ~exist('report_figures','dir'), mkdir('report_figures'); end

fprintf('=====================================================\n');
fprintf('  AE700 Section 2 — Kinematics and Dynamics\n');
fprintf('  F-4 Phantom\n');
fprintf('=====================================================\n\n');

x0=zeros(12,1); x0(3)=-300;   % stationary, 300 ft altitude
opts=odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',0.05);
wind=zeros(6,1);
d0=zeros(4,1);    % no control inputs

%% ============================================================
%  SECTION 2(ii): Individual axis tests
%  Apply one nonzero force/moment at a time, verify response
% ============================================================
fprintf('--- 2(ii): Individual Force/Moment Axis Tests ---\n\n');
fprintf('  Expected behaviour per axis:\n');
fprintf('  fx > 0 : forward acceleration (u increases)\n');
fprintf('  fy > 0 : rightward acceleration (v increases, drift east)\n');
fprintf('  fz > 0 : downward force (w increases, altitude drops)\n');
fprintf('  l  > 0 : roll RIGHT (p increases, phi increases)\n');
fprintf('  m  > 0 : nose UP  (q increases, theta increases)\n');
fprintf('  n  > 0 : yaw RIGHT (r increases, psi increases)\n\n');

% [fx, fy, fz, l, m, n]  — pure external inputs (no aero)
tests = {
    [5000,  0,    0,    0,    0,    0], 'fx=5000 lb  (forward)', [4,11], 'u [ft/s]',     'q [rad/s]';
    [0,     800,  0,    0,    0,    0], 'fy=800 lb   (side)',    [5, 9], 'v [ft/s]',     'psi [rad]';
    [0,     0,    800,  0,    0,    0], 'fz=800 lb   (down)',    [6, 3], 'w [ft/s]',     'pd [ft]';
    [0,     0,    0,  8000,   0,    0], 'l=8000 lb.ft (roll)',   [10,7], 'p [rad/s]',    'phi [rad]';
    [0,     0,    0,    0,  8000,   0], 'm=8000 lb.ft (pitch)',  [11,8], 'q [rad/s]',    'theta [rad]';
    [0,     0,    0,    0,    0,  8000],'n=8000 lb.ft (yaw)',    [12,9], 'r [rad/s]',    'psi [rad]';
};

fig_2ii=figure('Name','Section 2(ii): Axis Tests','Position',[30,30,1400,820]);
for k=1:6
    fm=tests{k,1}'; lbl=tests{k,2}; si=tests{k,3}; sl1=tests{k,4}; sl2=tests{k,5};

    [t,X]=ode45(@(t,x) eom_bare(x,fm,P),[0,15],x0,opts);

    subplot(2,3,k);
    yyaxis left;  plot(t,X(:,si(1)),'b-','LineWidth',2); ylabel(sl1,'FontSize',10);
    yyaxis right; plot(t,X(:,si(2)),'r--','LineWidth',2); ylabel(sl2,'FontSize',10);
    xlabel('Time [s]','FontSize',10); title(lbl,'FontSize',10,'FontWeight','bold');
    legend(sl1,sl2,'Location','northwest','FontSize',8); grid on; box on;

    fprintf('  %s\n    %s(%ds) = %+.4f  |  %s(%ds) = %+.4f\n\n', ...
            lbl, sl1,t(end), X(end,si(1)), sl2,t(end), X(end,si(2)));
end
sgtitle('Section 2(ii): Individual Axis Tests — F-4 Phantom','FontSize',13,'FontWeight','bold');
saveas(fig_2ii,'report_figures/Sec2_ii_AxisTests.png');
fprintf('  Figure saved: report_figures/Sec2_ii_AxisTests.png\n\n');

%% ============================================================
%  SECTION 2(iii): Jxz Gyroscopic Coupling
%  Apply l=3000 and n=3000 simultaneously.
%  Case A: Jxz=0  -> p and r grow proportionally (no coupling)
%  Case B: Jxz≠0  -> Gamma1 and Gamma4 terms couple p and r
% ============================================================
fprintf('--- 2(iii): Jxz Gyroscopic Coupling ---\n\n');

fm_c=[0;0;0;30000;0;0];

% Case A: zero Jxz
P_A=P; P_A.Jxz=0;
[tA,XA]=ode45(@(t,x) eom_bare(x,fm_c,P_A),[0,30],x0,opts);

% Case B: actual Jxz=2200
[tB,XB]=ode45(@(t,x) eom_bare(x,fm_c,P), [0,30],x0,opts);

fig_2iii=figure('Name','Section 2(iii): Jxz Coupling','Position',[30,500,1100,600]);
% subplot(1,2,1);
% plot(tA,rad2deg(XA(:,10)),'b-','LineWidth',2,'DisplayName','p (roll)'); hold on;
% plot(tA,rad2deg(XA(:,12)),'r--','LineWidth',2,'DisplayName','r (yaw)');
% xlabel('Time [s]'); ylabel('Angular rate [deg/s]');
% title('Jxz = 0  (no coupling)','FontSize',12,'FontWeight','bold');
% legend('FontSize',10); grid on; box on;
% 
% subplot(1,2,2);
% plot(tB,rad2deg(XB(:,10)),'b-','LineWidth',2,'DisplayName','p (roll)'); hold on;
% plot(tB,rad2deg(XB(:,12)),'r--','LineWidth',2,'DisplayName','r (yaw)');
% xlabel('Time [s]'); ylabel('Angular rate [deg/s]');
% title(sprintf('Jxz=%.0f slug-ft²  (coupled)',P.Jxz),'FontSize',12,'FontWeight','bold');
% legend('FontSize',10); grid on; box on;
% sgtitle('Section 2(iii): Jxz Gyroscopic Coupling','FontSize',13,'FontWeight','bold');
% saveas(fig_2iii,'report_figures/Sec2_iii_JxzCoupling.png');
subplot(2,2,1);
plot(tA, rad2deg(XA(:,10)), 'b-', 'LineWidth', 2);
ylabel('p [deg/s]', 'FontSize', 11);
title('Jxz = 0 : Roll rate p', 'FontSize', 11, 'FontWeight', 'bold');
grid on; box on;

subplot(2,2,2);
plot(tB, rad2deg(XB(:,10)), 'b-', 'LineWidth', 2);
ylabel('p [deg/s]', 'FontSize', 11);
title('Jxz = 2200 : Roll rate p', 'FontSize', 11, 'FontWeight', 'bold');
grid on; box on;

% --- Bottom row: yaw rate r (KEY: zero vs nonzero shows coupling) ---
subplot(2,2,3);
plot(tA, rad2deg(XA(:,12)), 'r-', 'LineWidth', 2);
ylabel('r [deg/s]', 'FontSize', 11);
xlabel('Time [s]', 'FontSize', 11);
title('Jxz = 0 : Induced yaw rate r  (should be \approx 0)', ...
      'FontSize', 11, 'FontWeight', 'bold');
yline(0, 'k--', 'LineWidth', 1);
grid on; box on;

subplot(2,2,4);
plot(tB, rad2deg(XB(:,12)), 'r-', 'LineWidth', 2);
ylabel('r [deg/s]', 'FontSize', 11);
xlabel('Time [s]', 'FontSize', 11);
title('Jxz = 2200 : Induced yaw rate r  (nonzero — coupling!)', ...
      'FontSize', 11, 'FontWeight', 'bold');
yline(0, 'k--', 'LineWidth', 1);
grid on; box on;

sgtitle({'Section 2(iii): Jxz Gyroscopic Coupling', ...
         'Pure roll moment applied — r induced only when Jxz \neq 0'}, ...
        'FontSize', 13, 'FontWeight', 'bold');
saveas(fig_2iii, 'report_figures/Sec2_iii_JxzCoupling.png');

fprintf('  Jxz = 0:\n');
fprintf('    p(10s) = %.4f rad/s  |  r(10s) = %.4f rad/s  (INDEPENDENT)\n', XA(end,10),XA(end,12));
fprintf('  Jxz = %.0f slug-ft²:\n',P.Jxz);
fprintf('    p(10s) = %.4f rad/s  |  r(10s) = %.4f rad/s  (COUPLED — different)\n', XB(end,10),XB(end,12));
fprintf('  Physical interpretation: Jxz couples roll & yaw via Gamma1 and Gamma4\n');
fprintf('  in eq (3.17). Symmetric aircraft (Jxz=0) has no such coupling.\n\n');
fprintf('  Figure saved: report_figures/Sec2_iii_JxzCoupling.png\n\n');

%% ============================================================
%  ANIMATION: pitch-up manoeuvre
% ============================================================
fprintf('--- Animation: pitch-up moment (m=8000 lb.ft) ---\n');
[ta,Xa]=ode45(@(t,x) eom_bare(x,[0;0;0;0;8000;0],P),[0,10],x0,opts);
fig_anim=figure('Name','Sec2 Pitch-up','Position',[30,30,700,600]);
h_an=[];
for k=1:4:length(ta)
    uu=[Xa(k,1);Xa(k,2);Xa(k,3);Xa(k,7);Xa(k,8);Xa(k,9)];
    h_an=drawAircraft(uu,h_an,1,fig_anim.Number);
    title(sprintf('Pitch-up | t=%.1fs | \\theta=%.1f° | alt=%.0f ft',...
                  ta(k),rad2deg(Xa(k,8)),-Xa(k,3)),'FontSize',11);
    drawnow limitrate; pause(0.01);
end
fprintf('Section 2 complete.\n\n');

%% ---- Bare EOM (no forces_moments, pure external inputs) ----
function xdot=eom_bare(x,fm,P)
    u=x(4);v=x(5);w=x(6); phi=x(7);theta=x(8);psi=x(9); p=x(10);q=x(11);r=x(12);
    fx=fm(1);fy=fm(2);fz=fm(3); l=fm(4);mm=fm(5);n=fm(6);
    cp=cos(phi);sp=sin(phi); ct=cos(theta);st=sin(theta);tt=tan(theta);
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
