function dp = dubins_parameters(start_node, end_node, R)
% =========================================================
% dubins_parameters.m
% Compute Dubins path geometry between two configurations.
% Beard & McLain, "Small Unmanned Aircraft," Chapter 11.
% Implements Algorithm 10.
%
% INPUTS:
%   start_node - 1x4 [pn, pe, h, chi_s]  start pose
%   end_node   - 1x4 [pn, pe, h, chi_e]  end pose
%   R          - minimum turn radius [ft]
%
% OUTPUT:
%   dp - struct with fields:
%     .L        total path length [ft]
%     .c_s      3x1 start circle centre
%     .lambda_s start turn direction (+1=CW, -1=CCW)
%     .c_e      3x1 end circle centre
%     .lambda_e end turn direction
%     .z1       3x1 start of straight segment
%     .q1       3x1 direction of straight segment (unit)
%     .z2       3x1 end of straight segment
%     .z3       3x1 start of end-circle tangent
%     .q3       3x1 direction at end of path
%     .rho      turn radius (= R)
%
% Four Dubins cases: RSR, RSL, LSR, LSL (Algorithm 10).
% =========================================================

%% Unpack start and end configurations
ps  = [start_node(1); start_node(2); 0];
chi_s = start_node(4);
pe_ = [end_node(1);   end_node(2);   0];
chi_e = end_node(4);

%% Circle centres for each turn direction (Eq 11.5)
% Right-turn centre: c = p + R*[sin(chi); -cos(chi); 0]
% Left-turn  centre: c = p + R*[-sin(chi); cos(chi); 0]

% TODO: compute four circle-centre combinations (Eq 11.5)
% c_rs = ps  + R*[ sin(chi_s); -cos(chi_s); 0];
% c_ls = ps  + R*[-sin(chi_s);  cos(chi_s); 0];
% c_re = pe_ + R*[ sin(chi_e); -cos(chi_e); 0];
% c_le = pe_ + R*[-sin(chi_e);  cos(chi_e); 0];

c_rs = zeros(3,1);  c_ls = zeros(3,1);   % TODO Eq 11.5
c_re = zeros(3,1);  c_le = zeros(3,1);   % TODO Eq 11.5

%% Compute path lengths for all 4 cases (Algorithm 10)
% TODO: implement L_RSR, L_RSL, L_LSR, L_LSL  (Eqs 11.6-11.13)
L_RSR = Inf;  L_RSL = Inf;  L_LSR = Inf;  L_LSL = Inf;

% Find minimum-length case
[L_min, idx] = min([L_RSR, L_RSL, L_LSR, L_LSL]);

%% Set output fields for the winning case
% TODO: fill in z1, q1, z2, z3, q3, c_s, c_e, lambda_s, lambda_e
% for whichever case (idx) gives minimum length.
% switch idx
%   case 1  % RSR
%     ...
%   case 2  % RSL
%     ...
% end

dp.L        = L_min;
dp.c_s      = zeros(3,1);   % TODO: start circle centre
dp.lambda_s = +1;           % TODO: +1=CW, -1=CCW
dp.c_e      = zeros(3,1);   % TODO: end circle centre
dp.lambda_e = +1;           % TODO
dp.z1       = zeros(3,1);   % TODO: straight-segment start
dp.q1       = [1;0;0];      % TODO: straight-segment direction (unit)
dp.z2       = zeros(3,1);   % TODO: straight-segment end
dp.z3       = zeros(3,1);   % TODO: end-circle tangent point
dp.q3       = [1;0;0];      % TODO: direction at end of path
dp.rho      = R;

end
