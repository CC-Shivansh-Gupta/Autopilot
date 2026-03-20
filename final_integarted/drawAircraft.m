function handle = drawAircraft(uu, handle, mode, fig_num)
% =========================================================
% drawAircraft.m  —  F-4 Phantom 3D Animation
% AE700 — IIT Bombay
%
% INPUTS:
%   uu      - [pn; pe; pd; phi; theta; psi]  (NED frame)
%   handle  - patch handle (pass [] for first call)
%   mode    - 1: rotate then translate (CORRECT, Part iii)
%             2: translate then rotate (WRONG,   Part iv demo)
%   fig_num - figure number (default 1)
%
% OUTPUT:
%   handle  - updated patch handle
% =========================================================

if nargin < 4 || isempty(fig_num), fig_num = 1; end
if nargin < 3 || isempty(mode),    mode    = 1; end

%% Unpack states
pn=uu(1); pe=uu(2); pd=uu(3);
phi=uu(4); theta=uu(5); psi=uu(6);

%% ============================================================
%  F-4 Phantom Body-Frame Geometry  (ft, origin at CG)
%  Convention: +x forward, +y right, +z down
% ============================================================
sc=1.0;
fl_fwd=30*sc; fl_aft=33*sc;   % fuselage forward/aft length
fw=4*sc;      fh=3*sc;        % fuselage width/height

V_fuse = [
   fl_fwd,       0,           0;        % 1  nose tip
   fl_fwd*0.5,   fw,          fh;       % 2
   fl_fwd*0.5,   fw,         -fh;       % 3
   fl_fwd*0.5,  -fw,         -fh;       % 4
   fl_fwd*0.5,  -fw,          fh;       % 5
   0,            fw,          fh;       % 6
   0,            fw,         -fh;       % 7
   0,           -fw,         -fh;       % 8
   0,           -fw,          fh;       % 9
  -fl_aft*0.5,   fw*0.8,      fh*0.8;  % 10
  -fl_aft*0.5,   fw*0.8,     -fh*0.8;  % 11
  -fl_aft*0.5,  -fw*0.8,     -fh*0.8;  % 12
  -fl_aft*0.5,  -fw*0.8,      fh*0.8;  % 13
  -fl_aft,       0,           0;       % 14 tail tip
];

F_fuse = [
   1, 2, 3, 3, 3;   1, 3, 4, 4, 4;   1, 4, 5, 5, 5;   1, 5, 2, 2, 2;
   2, 6, 7, 3, 3;   3, 7, 8, 4, 4;   4, 8, 9, 5, 5;   5, 9, 6, 2, 2;
   6,10,11, 7, 7;   7,11,12, 8, 8;   8,12,13, 9, 9;   9,13,10, 6, 6;
  10,14,14,11,11;  11,14,14,12,12;  12,14,14,13,13;  13,14,14,10,10;
];

%-- Wings --
hs=19.35*sc; rlex=8*sc; rtex=-12*sc; tlex=0*sc; ttex=-6*sc; wd=2*sc;
V_wing_r = [rlex, fw,0; rtex, fw,0; ttex, hs,wd; tlex, hs,wd];
V_wing_l = [rlex,-fw,0; rtex,-fw,0; ttex,-hs,wd; tlex,-hs,wd];

%-- Horizontal tail (attached to fuselage mid-ring) --
ht_rx=-fl_aft*0.5; ht_tex=-fl_aft; ht_sp=11*sc;
ht_tlx=ht_tex+3*sc; ht_dih=-1.0*sc;
ht_ry=fw*0.8; ht_rz=0;
V_htail_r = [ht_rx, ht_ry, ht_rz; ht_tex, ht_ry*0.3, ht_rz;
             ht_tex, ht_sp, ht_dih; ht_tlx, ht_sp, ht_dih];
V_htail_l = [ht_rx,-ht_ry, ht_rz; ht_tex,-ht_ry*0.3, ht_rz;
             ht_tex,-ht_sp, ht_dih; ht_tlx,-ht_sp, ht_dih];

%-- Vertical tail --
vt_rx=-fl_aft*0.5; vt_tex=-fl_aft; vt_ht=10*sc; vt_tipx=vt_tex+4*sc;
vt_rz=-fh*0.8;
V_vtail = [vt_rx,0,vt_rz; vt_tex,0,vt_rz*0.3; vt_tex,0,vt_rz-vt_ht*0.6;
           vt_tipx,0,vt_rz-vt_ht; vt_rx,0,vt_rz-vt_ht*0.5];

%% Assemble vertex matrix
nF=size(V_fuse,1);  oWR=nF; oWL=oWR+4; oHR=oWL+4; oHL=oHR+4; oVT=oHL+4;
V_body=[V_fuse; V_wing_r; V_wing_l; V_htail_r; V_htail_l; V_vtail];

%% Build face connectivity
pad=@(F,n)[F,repmat(F(:,end),1,n-size(F,2))];
F5  = pad(F_fuse,5);
Fw  = [oWR+(1:4); oWL+(1:4)];   Fw=[Fw,Fw(:,end)];
Fht = [oHR+(1:4); oHL+(1:4)];   Fht=[Fht,Fht(:,end)];
Fvt = oVT+(1:5);
F_all = [F5; Fw; Fht; Fvt];

%% Rotation: Body -> Inertial NED  (R_b2ned = R_bv')
cphi=cos(phi); sphi=sin(phi);
cth =cos(theta); sth=sin(theta);
cpsi=cos(psi);   spsi=sin(psi);

R_bv = [cth*cpsi,                  cth*spsi,                  -sth;
        sphi*sth*cpsi-cphi*spsi,   sphi*sth*spsi+cphi*cpsi,   sphi*cth;
        cphi*sth*cpsi+sphi*spsi,   cphi*sth*spsi-sphi*cpsi,   cphi*cth];
R_b2ned = R_bv';

%% Apply rotation and translation
if mode == 1   % CORRECT: rotate about CG, then translate
    Vr = (R_b2ned * V_body')';
    V_world = [Vr(:,1)+pn, Vr(:,2)+pe, -Vr(:,3)-pd];   % Z-up display
else            % WRONG (Part iv): translate first, then rotate about origin
    Vt = [V_body(:,1)+pn, V_body(:,2)+pe, V_body(:,3)-pd];
    Vr = (R_b2ned * Vt')';
    V_world = [Vr(:,1), Vr(:,2), -Vr(:,3)];
end

%% Face colors
nF2=size(F5,1); nW=size(Fw,1); nH=size(Fht,1); nV2=size(Fvt,1);
C_all = [repmat([0.50 0.50 0.55],nF2,1);
         repmat([0.28 0.28 0.33],nW, 1);
         repmat([0.33 0.33 0.38],nH, 1);
         repmat([0.33 0.33 0.38],nV2,1)];

%% Reset stale handle
if ~isempty(handle) && (~isgraphics(handle) || ~isvalid(handle))
    handle=[];
end

cx=mean(V_world(:,1)); cy=mean(V_world(:,2)); cz=mean(V_world(:,3));
rng=80;

if isempty(handle)
    figure(fig_num); clf;
    handle=patch('Vertices',V_world,'Faces',F_all,...
                 'FaceVertexCData',C_all,'FaceColor','flat',...
                 'EdgeColor',[0.1 0.1 0.1],'LineWidth',0.4,...
                 'FaceLighting','gouraud','AmbientStrength',0.5);
    light('Position',[1, 0,-1],'Style','infinite');
    light('Position',[-1,1, 1],'Style','infinite');
    view(3); axis equal; grid on;
    xlabel('North [ft]'); ylabel('East [ft]'); zlabel('Altitude [ft]');
    title('F-4 Phantom — 3D Animation (AE700)','FontSize',13);
    xlim([cx-rng,cx+rng]); ylim([cy-rng,cy+rng]); zlim([cz-rng,cz+rng]);
    drawnow;
else
    set(handle,'Vertices',V_world);
    ax=get(handle,'Parent');
    ax.XLim=[cx-rng,cx+rng]; ax.YLim=[cy-rng,cy+rng]; ax.ZLim=[cz-rng,cz+rng];
    drawnow limitrate;
end
end
