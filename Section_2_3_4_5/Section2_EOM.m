%% Section2_EOM.m
% AE700 - Section 2: Kinematics and Dynamics
% IIT Bombay | F-4 Phantom
%
% Covers:
%   (ii)  Verify EOM - individual force/moment axis tests
%   (iii) Jxz gyroscopic coupling demonstration
%
% Dependencies: F4_Phantom_params.m, drawAircraft.m
% Run: >> Section2_EOM

clear; clc; close all;
F4_Phantom_params;

ac.mass=mass; ac.Jx=I_xx; ac.Jy=I_yy; ac.Jz=I_zz; ac.Jxz=I_xz;

fprintf('=====================================================\n');
fprintf('  AE700 Section 2 - Kinematics and Dynamics\n');
fprintf('  F-4 Phantom\n');
fprintf('=====================================================\n\n');

%% ---- Initial conditions: all zero (stationary aircraft at 300 ft) ----
x0 = zeros(12,1);
x0(3) = -300;   % 300 ft altitude
opts  = odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',0.05);

%% ============================================================
%  SECTION 2(ii): Individual axis tests
%  Apply ONE nonzero force/moment at a time, verify response
% ============================================================
fprintf('--- Section 2(ii): Individual Axis Tests ---\n\n');

%         [fx    fy    fz    l     m     n  ]
tests = { [5000,  0,    0,    0,    0,    0  ], 'fx=5000 lb  (forward thrust)',  [4,11],  'u [ft/s]',      'q [rad/s]',  'u increases (forward accel), q~0';
          [0,     800,  0,    0,    0,    0  ], 'fy=800 lb   (side force)',      [5, 9],  'v [ft/s]',      'psi [rad]',  'v increases (sideslip)';
          [0,     0,    800,  0,    0,    0  ], 'fz=800 lb   (down force)',      [6, 3],  'w [ft/s]',      'pd [ft]',    'w increases, altitude drops';
          [0,     0,    0,    8000, 0,    0  ], 'l=8000 lb.ft (roll moment)',    [10, 7], 'p [rad/s]',     'phi [rad]',  'p and phi increase (right roll)';
          [0,     0,    0,    0,    8000, 0  ], 'm=8000 lb.ft (pitch moment)',   [11, 8], 'q [rad/s]',     'theta [rad]','q and theta increase (nose up)';
          [0,     0,    0,    0,    0,    8000], 'n=8000 lb.ft (yaw moment)',    [12, 9], 'r [rad/s]',     'psi [rad]',  'r and psi increase (yaw right)'; };

tspan = [0, 15];
fig2 = figure('Name','Section 2(ii): Individual Axis Tests','NumberTitle','off','Position',[30,30,1400,800]);

results2 = struct();

for k = 1:6
    fm    = tests{k,1}';
    lbl   = tests{k,2};
    si    = tests{k,3};
    sl1   = tests{k,4};
    sl2   = tests{k,5};
    exp_s = tests{k,6};

    [t,X] = ode45(@(t,x) eom_rhs(x,fm,ac), tspan, x0, opts);

    % Store results for report
    results2(k).label    = lbl;
    results2(k).expected = exp_s;
    results2(k).t        = t;
    results2(k).X        = X;
    results2(k).state1   = sl1;
    results2(k).state2   = sl2;
    results2(k).idx1     = si(1);
    results2(k).idx2     = si(2);

    subplot(2,3,k);
    yyaxis left;  plot(t,X(:,si(1)),'b-','LineWidth',2); ylabel(sl1,'FontSize',10);
    yyaxis right; plot(t,X(:,si(2)),'r--','LineWidth',2); ylabel(sl2,'FontSize',10);
    xlabel('Time [s]','FontSize',10); title(lbl,'FontSize',10,'FontWeight','bold');
    legend(sl1,sl2,'Location','northwest','FontSize',8); grid on; box on;

    fprintf('  Test %d: %s\n', k, lbl);
    fprintf('    Expected : %s\n', exp_s);
    fprintf('    At t=15s : %s = %+.4f  |  %s = %+.4f\n\n', sl1,X(end,si(1)), sl2,X(end,si(2)));
end
sgtitle('Section 2(ii): Individual Force/Moment Axis Tests — F-4 Phantom','FontSize',13,'FontWeight','bold');

%% ============================================================
%  SECTION 2(iii): Jxz Gyroscopic Coupling
%  l=3000, n=3000 applied simultaneously
%  Case A: Jxz=0  -> p and r grow independently
%  Case B: Jxz=2200 -> p and r are COUPLED (different rates)
% ============================================================
fprintf('--- Section 2(iii): Jxz Gyroscopic Coupling ---\n\n');

fm_coup = [0;0;0;3000;0;3000];
tspan_c = [0,10];

ac_A = ac; ac_A.Jxz = 0;
[tA,XA] = ode45(@(t,x) eom_rhs(x,fm_coup,ac_A), tspan_c, x0, opts);
[tB,XB] = ode45(@(t,x) eom_rhs(x,fm_coup,ac),   tspan_c, x0, opts);

figure('Name','Section 2(iii): Jxz Coupling','NumberTitle','off','Position',[30,500,1000,420]);
subplot(1,2,1);
plot(tA,rad2deg(XA(:,10)),'b-','LineWidth',2,'DisplayName','p (roll rate)'); hold on;
plot(tA,rad2deg(XA(:,12)),'r--','LineWidth',2,'DisplayName','r (yaw rate)');
xlabel('Time [s]'); ylabel('Angular rate [deg/s]');
title('Jxz = 0  (NO coupling)','FontSize',12,'FontWeight','bold'); legend; grid on;

subplot(1,2,2);
plot(tB,rad2deg(XB(:,10)),'b-','LineWidth',2,'DisplayName','p (roll rate)'); hold on;
plot(tB,rad2deg(XB(:,12)),'r--','LineWidth',2,'DisplayName','r (yaw rate)');
xlabel('Time [s]'); ylabel('Angular rate [deg/s]');
title(sprintf('Jxz = %.0f slug-ft²  (COUPLED)',ac.Jxz),'FontSize',12,'FontWeight','bold'); legend; grid on;
sgtitle('Section 2(iii): Jxz Gyroscopic Coupling Test','FontSize',13,'FontWeight','bold');

fprintf('  Jxz = 0:\n');
fprintf('    p(10s) = %.4f rad/s  |  r(10s) = %.4f rad/s  (independent)\n', XA(end,10), XA(end,12));
fprintf('  Jxz = %g:\n', ac.Jxz);
fprintf('    p(10s) = %.4f rad/s  |  r(10s) = %.4f rad/s  (coupled — different values)\n\n', XB(end,10), XB(end,12));

%% ============================================================
%  ANIMATION: Pitch-up moment
% ============================================================
fprintf('--- Animation: pitch-up moment applied ---\n');
[ta,Xa] = ode45(@(t,x) eom_rhs(x,[0;0;0;0;8000;0],ac), [0,12], x0, opts);
figure('Name','Section 2 Animation','NumberTitle','off');
h_a=[];
for k=1:4:length(ta)
    uu=[Xa(k,1);Xa(k,2);Xa(k,3);Xa(k,7);Xa(k,8);Xa(k,9)];
    h_a=drawAircraft(uu,h_a,1,gcf().Number);
    title(sprintf('Pitch-up | t=%.1fs | theta=%.1f° | alt=%.0f ft', ta(k),rad2deg(Xa(k,8)),-Xa(k,3)));
    pause(0.02);
end

fprintf('\nSection 2 complete.\n');

%% ---- Local EOM function ----
function xdot = eom_rhs(x, fm, ac)
    mass=ac.mass; Jx=ac.Jx; Jy=ac.Jy; Jz=ac.Jz; Jxz=ac.Jxz;
    u=x(4);v=x(5);w=x(6); phi=x(7);theta=x(8);psi=x(9); p=x(10);q=x(11);r=x(12);
    fx=fm(1);fy=fm(2);fz=fm(3); l=fm(4);mm=fm(5);n=fm(6);
    cp=cos(phi);sp=sin(phi); ct=cos(theta);st=sin(theta);tt=tan(theta); cs=cos(psi);ss=sin(psi);
    G=Jx*Jz-Jxz^2; G1=Jxz*(Jx-Jy+Jz)/G; G2=(Jz*(Jz-Jy)+Jxz^2)/G;
    G3=Jz/G; G4=Jxz/G; G5=(Jz-Jx)/Jy; G6=Jxz/Jy; G7=((Jx-Jy)*Jx+Jxz^2)/G; G8=Jx/G;
    pnd=(ct*cs)*u+(sp*st*cs-cp*ss)*v+(cp*st*cs+sp*ss)*w;
    ped=(ct*ss)*u+(sp*st*ss+cp*cs)*v+(cp*st*ss-sp*cs)*w;
    pdd=(-st)*u+(sp*ct)*v+(cp*ct)*w;
    ud=r*v-q*w+fx/mass; vd=p*w-r*u+fy/mass; wd=q*u-p*v+fz/mass;
    phd=p+(sp*tt)*q+(cp*tt)*r; thd=cp*q-sp*r; psd=(sp/ct)*q+(cp/ct)*r;
    pd_=G1*p*q-G2*q*r+G3*l+G4*n; qd=G5*p*r-G6*(p^2-r^2)+mm/Jy; rd=G7*p*q-G1*q*r+G4*l+G8*n;
    xdot=[pnd;ped;pdd;ud;vd;wd;phd;thd;psd;pd_;qd;rd];
end
