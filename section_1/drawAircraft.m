function handle = drawAircraft(uu, handle, mode, fig_num)
% drawAircraft - Draw and animate the F-4 Phantom in NED frame
%
% INPUTS:
%   uu     - state vector [pn; pe; pd; phi; theta; psi]
%   handle - figure handle (pass [] on first call)
%   mode   - 1: rotate then translate (CORRECT)
%            2: translate then rotate (WRONG - Part iv demo)
%   fig_num- figure number (default 1)

% -------------------------------------------------------
% Defaults
% -------------------------------------------------------
if nargin < 4 || isempty(fig_num),  fig_num = 1;  end
if nargin < 3 || isempty(mode),     mode    = 1;  end

% -------------------------------------------------------
% Unpack states
% -------------------------------------------------------
pn    = uu(1);
pe    = uu(2);
pd    = uu(3);
phi   = uu(4);
theta = uu(5);
psi   = uu(6);

% -------------------------------------------------------
% F-4 Phantom Body-Frame Vertices (ft, origin at CG)
% Body frame: +x forward, +y right, +z down
% -------------------------------------------------------
sc = 1.0;

fuse_len_fwd = 30*sc;  fuse_len_aft = 33*sc;
fuse_w = 4*sc;         fuse_h = 3*sc;

V_fuse = [...
   fuse_len_fwd,      0,           0;        % 1  nose tip
   fuse_len_fwd*0.5,  fuse_w,      fuse_h;   % 2
   fuse_len_fwd*0.5,  fuse_w,     -fuse_h;   % 3
   fuse_len_fwd*0.5, -fuse_w,     -fuse_h;   % 4
   fuse_len_fwd*0.5, -fuse_w,      fuse_h;   % 5
   0,                 fuse_w,      fuse_h;   % 6
   0,                 fuse_w,     -fuse_h;   % 7
   0,                -fuse_w,     -fuse_h;   % 8
   0,                -fuse_w,      fuse_h;   % 9
  -fuse_len_aft*0.5,  fuse_w*0.8,  fuse_h*0.8;  % 10
  -fuse_len_aft*0.5,  fuse_w*0.8, -fuse_h*0.8;  % 11
  -fuse_len_aft*0.5, -fuse_w*0.8, -fuse_h*0.8;  % 12
  -fuse_len_aft*0.5, -fuse_w*0.8,  fuse_h*0.8;  % 13
  -fuse_len_aft,      0,           0;        % 14 tail tip
];

F_fuse = [...
  1,  2,  3,  3,  3;
  1,  3,  4,  4,  4;
  1,  4,  5,  5,  5;
  1,  5,  2,  2,  2;
  2,  6,  7,  3,  3;
  3,  7,  8,  4,  4;
  4,  8,  9,  5,  5;
  5,  9,  6,  2,  2;
  6, 10, 11,  7,  7;
  7, 11, 12,  8,  8;
  8, 12, 13,  9,  9;
  9, 13, 10,  6,  6;
 10, 14, 14, 11, 11;
 11, 14, 14, 12, 12;
 12, 14, 14, 13, 13;
 13, 14, 14, 10, 10;
];

% --- Wings ---
half_span    = 19.35*sc;
root_le_x    =  8.0*sc;   root_te_x  = -12.0*sc;
tip_le_x     =  0.0*sc;   tip_te_x   =  -6.0*sc;
wing_dihedral =  2.0*sc;

% Right wing: local indices 1-4
V_wing_r = [...
   root_le_x,  fuse_w,    0;
   root_te_x,  fuse_w,    0;
   tip_te_x,   half_span, wing_dihedral;
   tip_le_x,   half_span, wing_dihedral;
];
% Left wing: local indices 1-4
V_wing_l = [...
   root_le_x, -fuse_w,    0;
   root_te_x, -fuse_w,    0;
   tip_te_x,  -half_span, wing_dihedral;
   tip_le_x,  -half_span, wing_dihedral;
];

% --- Horizontal tail ---
ht_span=11*sc; ht_root_x=-28*sc; ht_te_x=-33*sc;
ht_tip_lx=-30*sc; ht_tip_tx=-33*sc; ht_dihedral=-1.5*sc;

V_htail_r = [...
   ht_root_x,  fuse_w*0.6, -fuse_h*0.5;
   ht_te_x,    fuse_w*0.6, -fuse_h*0.5;
   ht_tip_tx,  ht_span,     ht_dihedral;
   ht_tip_lx,  ht_span,     ht_dihedral;
];
V_htail_l = [...
   ht_root_x, -fuse_w*0.6, -fuse_h*0.5;
   ht_te_x,   -fuse_w*0.6, -fuse_h*0.5;
   ht_tip_tx, -ht_span,     ht_dihedral;
   ht_tip_lx, -ht_span,     ht_dihedral;
];

% --- Vertical tail ---
vt_height=10*sc; vt_root_x=-26*sc; vt_te_x=-33*sc; vt_tip_x=-29*sc;

V_vtail = [...
   vt_root_x, 0, -fuse_h;
   vt_te_x,   0, -fuse_h;
   vt_te_x,   0, -fuse_h - vt_height*0.6;
   vt_tip_x,  0, -fuse_h - vt_height;
   vt_root_x, 0, -fuse_h - vt_height*0.7;
];

% -------------------------------------------------------
% Assemble vertices
% -------------------------------------------------------
nV_fuse  = size(V_fuse,   1);   % 14
nV_wingr = size(V_wing_r, 1);   %  4
nV_wingl = size(V_wing_l, 1);   %  4
nV_htailr= size(V_htail_r,1);   %  4
nV_htaill= size(V_htail_l,1);   %  4

off_wingr  = nV_fuse;                              % 14
off_wingl  = nV_fuse + nV_wingr;                   % 18
off_htailr = nV_fuse + nV_wingr + nV_wingl;        % 22
off_htaill = off_htailr + nV_htailr;               % 26
off_vtail  = off_htaill + nV_htaill;               % 30

V_body = [V_fuse; V_wing_r; V_wing_l; V_htail_r; V_htail_l; V_vtail];

% -------------------------------------------------------
% Build face index matrices with correct global offsets
% -------------------------------------------------------
padFaces = @(F, n) [F, repmat(F(:,end), 1, n - size(F,2))];

F_fuse_p = padFaces(F_fuse, 5);   % already 5 cols, no change

% Wing faces (each wing has 4 local verts, face = [1 2 3 4])
F_wing_faces = [...
    1+off_wingr,  2+off_wingr,  3+off_wingr,  4+off_wingr,  4+off_wingr;   % right
    1+off_wingl,  2+off_wingl,  3+off_wingl,  4+off_wingl,  4+off_wingl;   % left
];

% H-tail faces
F_htail_faces = [...
    1+off_htailr, 2+off_htailr, 3+off_htailr, 4+off_htailr, 4+off_htailr;
    1+off_htaill, 2+off_htaill, 3+off_htaill, 4+off_htaill, 4+off_htaill;
];

% V-tail face (5 vertices)
F_vtail_face = [1+off_vtail, 2+off_vtail, 3+off_vtail, 4+off_vtail, 5+off_vtail];

F_all = [F_fuse_p; F_wing_faces; F_htail_faces; F_vtail_face];

% -------------------------------------------------------
% Rotation matrix: Body -> Inertial (NED)  [ZYX / 3-2-1]
% -------------------------------------------------------
cphi=cos(phi); sphi=sin(phi);
cth =cos(theta); sth=sin(theta);
cpsi=cos(psi); spsi=sin(psi);

% -------------------------------------------------------
% Rotation matrix: Body -> Inertial (NED)   3-2-1 / ZYX
% -------------------------------------------------------
% R_bv below is the VEHICLE-TO-BODY (inertial->body) matrix.
% For animation we need body->inertial, so use R_bv' (transpose).
% R_b2ned = R_bv' rotates body-frame vertices INTO inertial NED.
R_bv = [...
  cth*cpsi,                   cth*spsi,                  -sth;
  sphi*sth*cpsi - cphi*spsi,  sphi*sth*spsi + cphi*cpsi,  sphi*cth;
  cphi*sth*cpsi + sphi*spsi,  cphi*sth*spsi - sphi*cpsi,  cphi*cth;
];
R_b2ned = R_bv';   % body -> inertial NED (correct direction for animation)

% -------------------------------------------------------
% Apply rotation and translation
% -------------------------------------------------------
if mode == 1
    % CORRECT (Section 1-iii): rotate about own CG, THEN translate
    V_rotated    = (R_b2ned * V_body')';
    V_world(:,1) = V_rotated(:,1) + pn;
    V_world(:,2) = V_rotated(:,2) + pe;
    V_world(:,3) = -V_rotated(:,3) - pd;   % NED down -> altitude up
else
    % WRONG (Section 1-iv): translate first, then rotate about WORLD ORIGIN
    % Aircraft sweeps an arc instead of rotating about its own CG
    V_trans      = V_body;
    V_trans(:,1) = V_trans(:,1) + pn;
    V_trans(:,2) = V_trans(:,2) + pe;
    V_trans(:,3) = V_trans(:,3) - pd;
    V_rot2       = (R_b2ned * V_trans')';
    V_world      = V_rot2;
    V_world(:,3) = -V_world(:,3);
end

% -------------------------------------------------------
% Colors
% -------------------------------------------------------
nFuse  = size(F_fuse_p,    1);
nWings = size(F_wing_faces, 1);
nHtail = size(F_htail_faces,1);
nVtail = size(F_vtail_face, 1);

C_all = [repmat([0.50 0.50 0.55], nFuse,  1);
         repmat([0.30 0.30 0.35], nWings, 1);
         repmat([0.35 0.35 0.40], nHtail, 1);
         repmat([0.35 0.35 0.40], nVtail, 1)];

% -------------------------------------------------------
% Draw or update
% -------------------------------------------------------
% Reset handle if it's been deleted (e.g. figure was closed between runs)
if ~isempty(handle) && ~isvalid(handle)
    handle = [];
end

% Axis centre from actual vertex positions (correct for both mode 1 and 2)
cx = mean(V_world(:,1));
cy = mean(V_world(:,2));
cz = mean(V_world(:,3));
ax_range = 80;

if isempty(handle)
    figure(fig_num); clf;
    handle = patch('Vertices',         V_world, ...
                   'Faces',            F_all, ...
                   'FaceVertexCData',  C_all, ...
                   'FaceColor',        'flat', ...
                   'EdgeColor',        [0.1 0.1 0.1], ...
                   'LineWidth',        0.5, ...
                   'FaceLighting',     'gouraud', ...
                   'AmbientStrength',  0.5);
    light('Position', [1,  0, -1], 'Style', 'infinite');
    light('Position', [-1, 1,  1], 'Style', 'infinite');
    view(3); axis equal; grid on;
    xlabel('North (ft)','FontSize',12);
    ylabel('East (ft)', 'FontSize',12);
    zlabel('Altitude (ft)','FontSize',12);
    title('F-4 Phantom - 3D Animation (AE700)','FontSize',14);
    set(gca,'ZDir','normal');
    xlim([cx-ax_range, cx+ax_range]);
    ylim([cy-ax_range, cy+ax_range]);
    zlim([cz-ax_range, cz+ax_range]);
    drawnow;
else
    set(handle, 'Vertices', V_world);
    ax = get(handle, 'Parent');
    ax.XLim = [cx-ax_range, cx+ax_range];
    ax.YLim = [cy-ax_range, cy+ax_range];
    ax.ZLim = [cz-ax_range, cz+ax_range];
    drawnow limitrate;
end

end
