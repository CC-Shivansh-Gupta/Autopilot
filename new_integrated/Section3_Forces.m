%% Section3_Forces.m
% =========================================================
% AE700 — Section 3: Forces and Moments
% IIT Bombay | F-4 Phantom
%
% Covers:
%   (i)   forces_moments.m — verify outputs at trim
%   (ii)  Wind gust response: Va, alpha, beta
%   (iii) Control surface deflection tests
%
% Run: >> Section3_Forces
% =========================================================

clear; clc; close all;
F4_params;   % loads P

if ~exist('report_figures','dir'), mkdir('report_figures'); end

fprintf('=====================================================\n');
fprintf('  AE700 Section 3 — Forces and Moments\n');
fprintf('  F-4 Phantom\n');
fprintf('=====================================================\n\n');

wind0=zeros(6,1);
opts=odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',0.05);
tspan=[0,20];

% Trim state at 1000 ft, with velocity
x_trim=zeros(12,1); x_trim(3)=-1000;
x_trim(4)=P.Va_trim; x_trim(8)=P.alpha_trim;

% Trim controls
de_trim=P.de_trim; dt_trim=P.dt_trim;
d_trim=[de_trim;0;0;dt_trim];

%% ============================================================
%  SECTION 3(i): forces_moments.m output at trim
% ============================================================
fprintf('--- 3(i): forces_moments.m at trim ---\n');
fo=forces_moments(x_trim,d_trim,wind0,P);
fprintf('  Va     = %.3f ft/s\n',  fo(7));
fprintf('  alpha  = %.4f rad = %.2f deg\n', fo(8),rad2deg(fo(8)));
fprintf('  beta   = %.5f rad\n',   fo(9));
fprintf('  [fx,fy,fz] = [%+.2f, %+.2f, %+.2f] lb\n', fo(1),fo(2),fo(3));
fprintf('  [l, m, n]  = [%+.2f, %+.2f, %+.2f] lb.ft\n\n', fo(4),fo(5),fo(6));
fprintf('  Note: At trim fx≈0 (thrust balanced by drag+gravity component),\n');
fprintf('        fz≈0 (lift ≈ weight),  l=m=n≈0.\n\n');

%% ============================================================
%  SECTION 3(ii): Wind gust response
% ============================================================
fprintf('--- 3(ii): Wind gust response ---\n');

% Three gust scenarios
gusts = {
    [0;0;0;50;0;0],  'Head-on body gust  ug=+50 ft/s';
    [0;0;0;0;30;0],  'Lateral body gust  vg=+30 ft/s';
    [50;0;0;0;0;0],  'North steady wind  wn=+50 ft/s';
};

fig3ii=figure('Name','Section 3(ii): Wind Gust','Position',[30,30,1200,600]);
for k=1:3
    wnd=gusts{k,1}; lbl=gusts{k,2};
    [tw,Xw]=ode45(@(t,x) eom(x,d_trim,wnd,P),tspan,x_trim,opts);
    Va_g=arrayfun(@(i) forces_moments(Xw(i,:)',d_trim,wnd,P),1:length(tw),'Uni',false);
    Va_a=cellfun(@(f) f(7),Va_g)';
    al_a=cellfun(@(f) f(8),Va_g)';
    be_a=cellfun(@(f) f(9),Va_g)';

    subplot(3,3,k);
    plot(tw,Va_a,'b-','LineWidth',2); ylabel('Va [ft/s]'); title(lbl,'FontSize',9); grid on;
    subplot(3,3,3+k);
    plot(tw,rad2deg(al_a),'r-','LineWidth',2); ylabel('\alpha [deg]'); grid on;
    subplot(3,3,6+k);
    plot(tw,rad2deg(be_a),'g-','LineWidth',2); ylabel('\beta [deg]'); xlabel('Time [s]'); grid on;

    fprintf('  %s:\n    Va: %.1f→%.1f ft/s | alpha: %.2f→%.2f° | beta: %.2f→%.2f°\n', ...
            lbl, Va_a(1),Va_a(end), rad2deg(al_a(1)),rad2deg(al_a(end)),...
            rad2deg(be_a(1)),rad2deg(be_a(end)));
end
sgtitle('Section 3(ii): Wind Gust Response','FontSize',13,'FontWeight','bold');
saveas(fig3ii,'report_figures/Sec3_ii_GustResponse.png');
fprintf('\n  Figure saved: report_figures/Sec3_ii_GustResponse.png\n\n');

%% ============================================================
%  SECTION 3(iii): Control surface deflection tests
% ============================================================
fprintf('--- 3(iii): Control surface deflection tests ---\n\n');

cs_tests = {
    [de_trim+deg2rad(10); 0;           0;          dt_trim], [8,11], '\theta [rad]','q [rad/s]','Elevator +10°: nose up';
    [de_trim-deg2rad(10); 0;           0;          dt_trim], [8,11], '\theta [rad]','q [rad/s]','Elevator -10°: nose down';
    [de_trim;             deg2rad(15); 0;          dt_trim], [7,10], '\phi [rad]',  'p [rad/s]','Aileron +15°: right roll';
    [de_trim;             0;           deg2rad(10);dt_trim], [9,12], '\psi [rad]',  'r [rad/s]','Rudder +10°: yaw right';
    [de_trim;             0;           0;          0.95   ], [4, 3], 'u [ft/s]',   'pd [ft]',  'Throttle 0.95: accelerate/climb';
    [de_trim;             0;           0;          0.30   ], [4, 3], 'u [ft/s]',   'pd [ft]',  'Throttle 0.30: decelerate/descend';
};

fig3iii=figure('Name','Section 3(iii): Control Surfaces','Position',[30,30,1400,820]);
for k=1:6
    d=cs_tests{k,1}; si=cs_tests{k,2}; sl1=cs_tests{k,3}; sl2=cs_tests{k,4}; lbl=cs_tests{k,5};
    [t,X]=ode45(@(t,x) eom(x,d,wind0,P),tspan,x_trim,opts);
    subplot(2,3,k);
    yyaxis left;  plot(t,X(:,si(1)),'b-','LineWidth',2); ylabel(sl1,'FontSize',10);
    yyaxis right; plot(t,X(:,si(2)),'r--','LineWidth',2); ylabel(sl2,'FontSize',10);
    xlabel('Time [s]'); title(lbl,'FontSize',10,'FontWeight','bold');
    legend(sl1,sl2,'Location','northwest','FontSize',8); grid on; box on;
    fprintf('  %s\n    %s(20s)=%+.4f  |  %s(20s)=%+.4f\n', ...
            lbl, sl1,X(end,si(1)), sl2,X(end,si(2)));
end
sgtitle('Section 3(iii): Control Surface Tests','FontSize',13,'FontWeight','bold');
saveas(fig3iii,'report_figures/Sec3_iii_ControlSurface.png');
fprintf('\n  Figure saved: report_figures/Sec3_iii_ControlSurface.png\n\n');
fprintf('Section 3 complete.\n\n');
