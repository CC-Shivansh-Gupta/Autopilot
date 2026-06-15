function waypoints = rrt(map, start, goal, P, use_dubins)
% =========================================================
% rrt.m
% Rapidly-exploring Random Tree path planning.
% Beard & McLain, "Small Unmanned Aircraft," Chapter 12.
%
% INPUTS:
%   map        - struct:
%                  .buildings  : Nx5 [pn, pe, h, width, height] (ft)
%                  .world_width  : world size in north [ft]
%                  .world_height : world size in east  [ft]
%   start      - 1x4 [pn, pe, h, chi]  start configuration
%   goal       - 1x4 [pn, pe, h, chi]  goal configuration
%   P          - param struct (needs P.R_min, P.rrt_step)
%   use_dubins - false = straight-line RRT (Alg 11)
%                true  = Dubins RRT        (Alg 12)
%
% OUTPUT:
%   waypoints  - Mx4 list of waypoints [pn, pe, h, chi]
%                smoothed / pruned path from start to goal.
%
% Algorithm references:
%   Straight-line RRT : Algorithm 11  (Sec 12.3)
%   Dubins RRT        : Algorithm 12  (Sec 12.4, calls dubins_parameters)
%
% Units: ft throughout.
% =========================================================

%% RRT parameters (add to F4_params.m or override here)
max_iter  = 1000;                     % maximum tree expansions
step_size = P.rrt_step;               % straight-line step length [ft]
R_min     = P.R_min;                  % Dubins turn radius [ft]

%% Initialise tree
% Tree nodes stored as rows of [pn, pe, h, chi]
% tree.nodes(k,:) = node k
% tree.parent(k)  = parent index of node k
tree.nodes  = start;
tree.parent = 0;    % root has no parent

%% ---- Main RRT Loop ----
found = false;

for iter = 1:max_iter

    %% 1. Sample random configuration (Eq 12.1 — biased toward goal)
    if rand < 0.05
        x_rand = goal;
    else
        % TODO: sample uniformly in [0, world_width] x [0, world_height]
        x_rand = [rand*map.world_width, rand*map.world_height, ...
                  start(3), rand*2*pi - pi];  % random pn,pe,h,chi
    end

    %% 2. Find nearest node in tree
    % TODO: compute distances from x_rand to all tree.nodes
    % [~, idx_near] = min( distances );
    idx_near = 1;   % placeholder: always root
    x_near   = tree.nodes(idx_near, :);

    %% 3. Steer toward x_rand
    if ~use_dubins
        % Straight-line RRT (Algorithm 11)
        % x_new = x_near + step_size * (x_rand - x_near) / norm(...)
        % TODO: compute x_new (Eq 12.2)
        x_new = x_near;   % placeholder
    else
        % Dubins RRT (Algorithm 12)
        % dp = dubins_parameters(x_near, x_rand, R_min)
        % move step_size along the Dubins path from x_near
        % TODO: compute x_new along Dubins arc
        x_new = x_near;   % placeholder
    end

    %% 4. Collision check (Algorithm 11, Step 4)
    % TODO: check if path from x_near to x_new collides with any building
    %   col = collision_check(x_near, x_new, map, use_dubins, R_min);
    collision = false;   % placeholder

    if ~collision
        % Add x_new to tree
        tree.nodes  = [tree.nodes;  x_new];
        tree.parent = [tree.parent; idx_near];

        %% 5. Check if goal reached
        % TODO: if norm(x_new(1:2) - goal(1:2)) < step_size, found = true
        if norm(x_new(1:2) - goal(1:2)) < step_size
            tree.nodes  = [tree.nodes;  goal];
            tree.parent = [tree.parent; size(tree.nodes,1)-1];
            found = true;
            break;
        end
    end
end

if ~found
    warning('rrt: goal not reached in %d iterations', max_iter);
    waypoints = [start; goal];
    return;
end

%% ---- Extract and smooth path ----
% Trace parent pointers from goal back to start
path_idx = size(tree.nodes, 1);
raw_path = [];
while path_idx ~= 0
    raw_path = [tree.nodes(path_idx,:); raw_path];
    path_idx = tree.parent(path_idx);
end

% TODO: path smoothing — try to skip intermediate nodes if direct path
%       is collision-free (Algorithm 11, Step 6)
waypoints = raw_path;

end

%% ---- Helper: collision check ----
% (student implements this)
% function col = collision_check(x1, x2, map, use_dubins, R_min)
%   col = false;
%   % For straight-line: check if segment x1->x2 intersects any building.
%   % For Dubins: discretise the arc and check each sample point.
%   for b = 1:size(map.buildings, 1)
%     % TODO: AABB or cylinder collision test
%   end
% end
