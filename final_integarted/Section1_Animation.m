%% Section1_Animation.m
% =========================================================
% AE700 — Section 1: Coordinate Frames and Animation
% IIT Bombay | F-4 Phantom
%
% Covers:
%   (i)   Rotation matrices (theory, no code output)
%   (ii)  Static pose verification tests
%   (iii) Correct order: rotate then translate
%   (iv)  Wrong order demo: translate then rotate
%   (v)   Circular flight animation + video
%
% Run: >> Section1_Animation
% =========================================================

clear; clc; close all;
F4_params;      % loads P and workspace scalars

fprintf('=====================================================\n');
fprintf('  AE700 Section 1 — Coordinate Frames & Animation\n');
fprintf('  F-4 Phantom\n');
fprintf('=====================================================\n\n');

if ~exist('report_figures','dir'), mkdir('report_figures'); end

%% ============================================================
%  PART (ii): Static rotation axis tests
%  Verify each Euler angle independently
% ============================================================
fprintf('--- Part (ii): Static Pose Verification ---\n\n');

poses = {
    [0;0;-300; 0;0;0],           'Level flight  (phi=theta=psi=0)';
    [0;0;-300; pi/4;0;0],        'Roll 45° right';
    [0;0;-300; 0;pi/8;0],        'Pitch up 22.5°';
    [0;0;-300; 0;0;pi/4],        'Yaw 45° (heading change)';
    [0;0;-300; pi/6;pi/12;pi/3], 'Combined: roll+pitch+yaw';
    [200;100;-500; pi/8;pi/12;pi/3], 'Translated + rotated';
};

for k=1:length(poses)
    uu=poses{k,1}; lbl=poses{k,2};
    figure(k); clf;
    h=drawAircraft(uu,[],1,k);
    hold on;
    plot3(uu(1),uu(2),-uu(3),'r*','MarkerSize',14,'LineWidth',2,'DisplayName','CG');
    legend([h,gca().Children(1)],{'Aircraft','CG position'},'Location','best');
    title(sprintf('Part (ii).%d — %s',k,lbl),'FontSize',11,'FontWeight','bold');
    hold off;
    fprintf('  %d. %s\n', k, lbl);
    drawnow; pause(0.3);
end
fprintf('\n');

%% ============================================================
%  PART (iii): Circular flight animation — CORRECT order
%  Rotate about CG first, then translate
% ============================================================
fprintf('--- Part (iii): Circular Flight Animation (CORRECT order) ---\n');

fig_c=figure(10); clf;
set(fig_c,'Position',[40,40,900,700]);
h_c=[]; t=0:0.04:15; Va=850; R=3000; omega=Va/R;

% Pre-compute trajectory
traj_pn=R*sin(omega*t);
traj_pe=R*(1-cos(omega*t));

try
    vid_c=VideoWriter('report_figures/circular_flight.mp4','MPEG-4');
    vid_c.FrameRate=25; open(vid_c); do_video=true;
catch; do_video=false; end

for k=1:length(t)
    psi=omega*t(k);
    pn=traj_pn(k); pe=traj_pe(k); pd=-500;
    phi=atan(Va^2/(P.g*R));   % coordinated turn bank angle
    uu=[pn;pe;pd;phi;0;psi];
    h_c=drawAircraft(uu,h_c,1,10);
    hold on;
    if k>1
        plot3(traj_pn(1:k),traj_pe(1:k),500*ones(1,k),'c-','LineWidth',1.5);
    end
    hold off;
    title(sprintf('CORRECT: Rotate→Translate | t=%.1fs | Alt=%d ft | Bank=%.1f°',...
                  t(k),round(-pd),rad2deg(phi)),'FontSize',10);
    if do_video; writeVideo(vid_c,getframe(fig_c)); end
    drawnow limitrate;
end
if do_video; close(vid_c); fprintf('  Saved: report_figures/circular_flight.mp4\n'); end

%% ============================================================
%  PART (iv): Wrong order comparison
%  Shows aircraft sweeping arc about world origin
% ============================================================
fprintf('\n--- Part (iv): Wrong Order Demo (translate then rotate) ---\n');
fprintf('  CORRECT order: aircraft rotates about its own CG\n');
fprintf('  WRONG order:   aircraft orbits world origin — physically wrong\n\n');

fig_ok=figure(11); clf; set(fig_ok,'Position',[50, 200,750,560]);
fig_wr=figure(12); clf; set(fig_wr,'Position',[820,200,750,560]);
h_ok=[]; h_wr=[];
pn0=200; pe0=150; pd0=-300;

try
    vid_ord=VideoWriter('report_figures/rotation_order.mp4','MPEG-4');
    vid_ord.FrameRate=15; vid_ord.Quality=90; open(vid_ord); do_vid2=true;
catch; do_vid2=false; end

for psi_deg=0:5:360
    psi=psi_deg*pi/180;
    uu=[pn0;pe0;pd0;0;0;psi];

    h_ok=drawAircraft(uu,h_ok,1,11);
    figure(fig_ok);
    title(sprintf('CORRECT: Rotate→Translate  |  \\psi=%d°',psi_deg),...
          'FontSize',12,'Color',[0 0.5 0]);

    h_wr=drawAircraft(uu,h_wr,2,12);
    figure(fig_wr);
    title(sprintf('WRONG (Part iv): Translate→Rotate  |  \\psi=%d°',psi_deg),...
          'FontSize',12,'Color',[0.8 0 0]);
    drawnow;

    if do_vid2
        f11=getframe(fig_ok); f12=getframe(fig_wr);
        h_=min(size(f11.cdata,1),size(f12.cdata,1));
        img11=imresize(f11.cdata,[h_,NaN]);
        img12=imresize(f12.cdata,[h_,NaN]);
        divider=255*ones(h_,6,3,'uint8');
        writeVideo(vid_ord,[img11,divider,img12]);
    end
    pause(0.03);
end
if do_vid2; close(vid_ord); fprintf('  Saved: report_figures/rotation_order.mp4\n'); end

%% ============================================================
%  PART (i): Print rotation matrices for report
% ============================================================
fprintf('\n--- Part (i): Rotation Matrix Verification ---\n');
phi=deg2rad(10); theta=deg2rad(5); psi=deg2rad(30);

Rx=[1,0,0; 0,cos(phi),sin(phi); 0,-sin(phi),cos(phi)];
Ry=[cos(theta),0,-sin(theta); 0,1,0; sin(theta),0,cos(theta)];
Rz=[cos(psi),sin(psi),0; -sin(psi),cos(psi),0; 0,0,1];

R_bv = Rx*Ry*Rz;
fprintf('\n  R_b_to_vehicle (phi=10°, theta=5°, psi=30°):\n');
disp(R_bv);
fprintf('  det(R_bv) = %.6f  (should be 1.0)\n', det(R_bv));
fprintf('  R*R'' = I? max error = %.2e\n\n', max(max(abs(R_bv*R_bv'-eye(3)))));

saveas(figure(1),'report_figures/Sec1_Pose_LevelFlight.png');
saveas(figure(3),'report_figures/Sec1_Pose_PitchUp.png');
fprintf('Section 1 complete.\n\n');
