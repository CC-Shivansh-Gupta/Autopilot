%% Section3_Forces.m
% AE700 - Section 3: Forces and Moments
% IIT Bombay | F-4 Phantom
%
% Covers:
%   (i)   forces_moments.m implements gravity, aero, propulsion
%   (ii)  Wind gust response: Va, alpha, beta outputs verified
%   (iii) Control surface deflection tests
%
% Dependencies: F4_chap4_params.m, forces_moments.m, drawAircraft.m
% Run: >> Section3_Forces

clear; clc; close all;
F4_chap4_params;   % loads struct P

fprintf('=====================================================\n');
fprintf('  AE700 Section 3 - Forces and Moments\n');
fprintf('  F-4 Phantom\n');
fprintf('=====================================================\n\n');

%% ---- Trim state (stationary at 1000 ft, zero velocity) ----
% Using zero-velocity start so zero-force = zero motion (clean tests)
x_trim = zeros(12,1);
x_trim(3) = -1000;    % 1000 ft altitude

% Trim elevator to balance pitching moment at alpha_trim
de_trim = -(P.Cmo + P.Cm_a*P.alpha_trim) / P.Cm_de;
dt_trim = 0.5;
delta_trim = [de_trim; 0; 0; dt_trim];
wind_zero  = zeros(6,1);

opts = odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',0.05);
tspan = [0,20];

fprintf('Trim elevator: de = %.4f rad = %.2f deg\n\n', de_trim, rad2deg(de_trim));

%% ============================================================
%  SECTION 3(i): Verify forces_moments output at trim
% ============================================================
fprintf('--- Section 3(i): forces_moments.m at trim ---\n');
fm_out = forces_moments(x_trim, delta_trim, wind_zero, P);
fprintf('  Va     = %.2f ft/s\n',   fm_out(7));
fprintf('  alpha  = %.4f rad = %.2f deg\n', fm_out(8), rad2deg(fm_out(8)));
fprintf('  beta   = %.4f rad\n',    fm_out(9));
fprintf('  fx,fy,fz = [%.2f, %.2f, %.2f] lb\n', fm_out(1),fm_out(2),fm_out(3));
fprintf('  l,m,n    = [%.2f, %.2f, %.2f] lb.ft\n\n', fm_out(4),fm_out(5),fm_out(6));

%% ============================================================
%  SECTION 3(ii): Wind gust response
%  Verify: Va, alpha, beta, and wind vector outputs
% ============================================================
fprintf('--- Section 3(ii): Wind gust response ---\n');

% Trim state with velocity (for meaningful wind test)
x_wind = zeros(12,1);
x_wind(3) = -1000;
x_wind(4) = P.Va_trim;
x_wind(8) = P.alpha_trim;

wind_headon = [0;0;0; 50;0;0];   % 50 ft/s head-on body gust

[tw, Xw] = ode45(@(t,x) full_eom(x,delta_trim,wind_headon,P), tspan, x_wind, opts);

Va_h=zeros(length(tw),1); alpha_h=Va_h; beta_h=Va_h;
for k=1:length(tw)
    fo=forces_moments(Xw(k,:)',delta_trim,wind_headon,P);
    Va_h(k)=fo(7); alpha_h(k)=fo(8); beta_h(k)=fo(9);
end

fig3ii = figure('Name','Section 3(ii): Wind Gust Response','NumberTitle','off','Position',[30,30,1000,500]);
subplot(3,1,1); plot(tw,Va_h,'b-','LineWidth',2); ylabel('Va [ft/s]'); grid on;
title('Head-on gust: ug=50 ft/s body-x');
subplot(3,1,2); plot(tw,rad2deg(alpha_h),'r-','LineWidth',2); ylabel('\alpha [deg]'); grid on;
subplot(3,1,3); plot(tw,rad2deg(beta_h),'g-','LineWidth',2); ylabel('\beta [deg]'); xlabel('Time [s]'); grid on;
sgtitle('Section 3(ii): Wind Gust Response — Va, \alpha, \beta','FontSize',12,'FontWeight','bold');

fprintf('  Head-on gust ug=50 ft/s:\n');
fprintf('    Va  : initial=%.1f  final=%.1f ft/s\n', Va_h(1), Va_h(end));
fprintf('    alpha: initial=%.2f  final=%.2f deg\n', rad2deg(alpha_h(1)),rad2deg(alpha_h(end)));
fprintf('    beta : initial=%.4f  final=%.4f deg\n\n', rad2deg(beta_h(1)),rad2deg(beta_h(end)));

%% ============================================================
%  SECTION 3(iii): Control surface deflection tests
% ============================================================
fprintf('--- Section 3(iii): Control Surface Tests ---\n\n');

%          delta=[de, da, dr, dt],              plot states,   labels
cs_tests = {
    [de_trim+deg2rad(10); 0;            0;          dt_trim], [8,11], '\theta [rad]','q [rad/s]','Elevator+10°: nose up (theta,q increase)';
    [de_trim-deg2rad(10); 0;            0;          dt_trim], [8,11], '\theta [rad]','q [rad/s]','Elevator-10°: nose down (theta,q decrease)';
    [de_trim;             deg2rad(15);  0;          dt_trim], [7,10], '\phi [rad]',  'p [rad/s]','Aileron+15°: right roll (phi,p increase)';
    [de_trim;             0;            deg2rad(10);dt_trim], [9,12], '\psi [rad]',  'r [rad/s]','Rudder+10°: yaw right (psi,r increase)';
    [de_trim;             0;            0;          0.8    ], [4, 3], 'u [ft/s]',   'pd [ft]',  'Throttle 0.8: u increases, altitude rises';
    [de_trim;             0;            0;          0.0    ], [4, 3], 'u [ft/s]',   'pd [ft]',  'Throttle 0.0: u decays, altitude drops';
};
cs_labels = {'Elevator +10°','Elevator -10°','Aileron +15°','Rudder +10°','Throttle 0.8','Throttle 0.0'};

fig3iii = figure('Name','Section 3(iii): Control Surface Tests','NumberTitle','off','Position',[30,30,1400,820]);
results3 = struct();

for k=1:6
    delta = cs_tests{k,1};
    si    = cs_tests{k,2};
    sl1   = cs_tests{k,3};
    sl2   = cs_tests{k,4};
    exp_s = cs_tests{k,5};

    [t,X] = ode45(@(t,x) full_eom(x,delta,wind_zero,P), tspan, x_trim, opts);

    results3(k).label    = cs_labels{k};
    results3(k).expected = exp_s;
    results3(k).t = t; results3(k).X = X;
    results3(k).sl1=sl1; results3(k).sl2=sl2;
    results3(k).idx1=si(1); results3(k).idx2=si(2);

    subplot(2,3,k);
    yyaxis left;  plot(t,X(:,si(1)),'b-','LineWidth',2); ylabel(sl1,'FontSize',10);
    yyaxis right; plot(t,X(:,si(2)),'r--','LineWidth',2); ylabel(sl2,'FontSize',10);
    xlabel('Time [s]','FontSize',10); title(cs_labels{k},'FontSize',10,'FontWeight','bold');
    legend(sl1,sl2,'Location','northwest','FontSize',8); grid on; box on;

    fprintf('  %s\n', cs_labels{k});
    fprintf('    Expected : %s\n', exp_s);
    fprintf('    At t=20s : %s=%+.4f  |  %s=%+.4f\n\n', sl1,X(end,si(1)), sl2,X(end,si(2)));
end
sgtitle('Section 3(iii): Control Surface Deflection Tests — F-4 Phantom','FontSize',13,'FontWeight','bold');

fprintf('Section 3 complete.\n');

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
