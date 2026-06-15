function path = path_manager(waypoints, state, P)
% =========================================================
% path_manager.m
% Sequences waypoints into path segments for the path follower.
% Beard & McLain, "Small Unmanned Aircraft," Chapter 11.
%
% INPUTS:
%   waypoints - Nx4 matrix: each row = [pn, pe, h, chi]
%               chi is heading at waypoint (used by Dubins manager)
%   state     - struct: pn, pe, h, Va, chi  (from estimator)
%   P         - param struct (needs P.manager_type, P.R_min)
%
% OUTPUT:
%   path  - struct for path_follower.m:
%             .flag, .Va_d, .r, .q   (line)
%             .c, .rho, .lambda      (orbit)
%
% Dispatch: P.manager_type = 'line' | 'fillet' | 'dubins'
%
% Sub-functions:
%   line_manager    — Algorithm 7
%   fillet_manager  — Algorithm 8
%   dubins_manager  — Algorithm 9  (calls dubins_parameters)
%
% Uses persistent pointer to track current active waypoint.
% =========================================================

persistent ptr    % index of current "from" waypoint

if isempty(ptr), ptr = 1; end

switch P.manager_type
    case 'line'
        path = line_manager(waypoints, state, P, ptr);
    case 'fillet'
        path = fillet_manager(waypoints, state, P, ptr);
    case 'dubins'
        path = dubins_manager(waypoints, state, P, ptr);
    otherwise
        error('path_manager: unknown P.manager_type = %s', P.manager_type);
end

end

%% ============================================================
%  Algorithm 7 — Straight-Line Manager
% ============================================================
function path = line_manager(wp, state, P, ptr)
% Follows straight-line segments between consecutive waypoints.
% Switches to next waypoint when MAV enters half-plane at wp(ptr+1).
% Reference: Algorithm 7, Fig 11.2

N = size(wp, 1);

% Current and next waypoint (wrap around)
i   = ptr;
ip1 = mod(ptr, N) + 1;

r = [wp(i,1);   wp(i,2);   -wp(i,3)];    % from-waypoint (NED, pd=-h)
r_next = [wp(ip1,1); wp(ip1,2); -wp(ip1,3)];

q = r_next - r;
if norm(q) > 1e-6, q = q / norm(q); end

% Check if MAV has crossed the half-plane at r_next (Eq 11.1)
p = [state.pn; state.pe; -state.h];
% TODO: if (p-r_next)'*q >= 0, advance ptr

% Build path struct
path.flag = 1;
path.Va_d = P.Va_trim;
path.r    = r;
path.q    = q;

end

%% ============================================================
%  Algorithm 8 — Fillet Manager
% ============================================================
function path = fillet_manager(wp, state, P, ptr)
% Connects waypoints with circular fillets to smooth corners.
% Reference: Algorithm 8, Eqs 11.17-11.23

N   = size(wp, 1);
i   = ptr;
ip1 = mod(ptr,   N) + 1;
ip2 = mod(ptr+1, N) + 1;

r1 = [wp(i,1);   wp(i,2);   -wp(i,3)];
r2 = [wp(ip1,1); wp(ip1,2); -wp(ip1,3)];
r3 = [wp(ip2,1); wp(ip2,2); -wp(ip2,3)];

q1 = r2 - r1;  if norm(q1)>1e-6, q1 = q1/norm(q1); end
q2 = r3 - r2;  if norm(q2)>1e-6, q2 = q2/norm(q2); end

% Fillet geometry (Eq 11.17-11.18)
% rho   = P.R_min / sin(acos(-q1'*q2)/2)
% TODO: compute fillet circle centre c, radius rho, direction lambda
% TODO: half-plane switching logic (Eq 11.20-11.23)

% Placeholder: straight line until student fills in fillet geometry
path.flag = 1;
path.Va_d = P.Va_trim;
path.r    = r1;
path.q    = q1;

end

%% ============================================================
%  Algorithm 9 — Dubins Manager
% ============================================================
function path = dubins_manager(wp, state, P, ptr)
% Uses Dubins paths between consecutive waypoints.
% Reference: Algorithm 9, calls dubins_parameters (Algorithm 10).

N   = size(wp, 1);
i   = ptr;
ip1 = mod(ptr, N) + 1;

start_node = wp(i,   :);    % [pn, pe, h, chi]
end_node   = wp(ip1, :);    % [pn, pe, h, chi]

dp = dubins_parameters(start_node, end_node, P.R_min);

% TODO: determine which Dubins sub-segment is active (start circle /
%       straight / end circle) and build the appropriate path struct.
% Sub-segments in order:
%   1. Start circle  (orbit, lambda_s, c_s, rho)
%   2. Straight line (z1 -> z2)
%   3. End circle    (orbit, lambda_e, c_e, rho)
% Switch between sub-segments using half-plane checks at z1, z2, z3
% (Algorithm 9, Fig 11.6).

% Placeholder: straight line
path.flag = 1;
path.Va_d = P.Va_trim;
path.r    = [start_node(1); start_node(2); -start_node(3)];
q         = [end_node(1)-start_node(1); end_node(2)-start_node(2); 0];
if norm(q) > 1e-6, q = q / norm(q); end
path.q    = q;

end
