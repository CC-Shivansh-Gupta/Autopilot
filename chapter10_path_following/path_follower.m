function cmd = path_follower(path, state, P)
% =========================================================
% path_follower.m
% Computes autopilot commands to follow a straight-line or
% orbit path segment.
% Beard & McLain, "Small Unmanned Aircraft," Chapter 10.
%
% INPUTS:
%   path  - struct defining current path segment:
%             .flag   : 1 = straight line, 2 = orbit
%             .Va_d   : desired airspeed [ft/s]
%             -- Straight-line fields (flag=1) --
%             .r      : 3x1 point on line [pn;pe;pd] [ft]
%             .q      : 3x1 unit direction vector
%             -- Orbit fields (flag=2) --
%             .c      : 3x1 orbit centre [pn;pe;pd] [ft]
%             .rho    : orbit radius [ft]
%             .lambda : direction (+1=CW, -1=CCW)
%   state - struct with fields pn,pe,h,Va,chi (from estimator or true)
%   P     - param struct from F4_params.m
%
% OUTPUT:
%   cmd   - struct:
%             .phi_c  : roll angle command [rad]
%             .chi_c  : course angle command [rad]
%             .h_c    : altitude command [ft]
%             .Va_c   : airspeed command [ft/s]
%
% Units: ft, slug, lb, rad throughout.
% =========================================================

cmd.Va_c = path.Va_d;   % always command desired airspeed

switch path.flag

    case 1   %% ---- Straight-Line Following (Algorithm 5) ----
        % Reference: Eqs 10.1-10.8

        % Unpack
        r  = path.r;       % 3x1 point on line
        q  = path.q;       % 3x1 unit direction (must be unit vector)
        p  = [state.pn; state.pe; -state.h];   % MAV position in NED

        % Altitude command: follow line altitude (Eq 10.5)
        % TODO: cmd.h_c = -r(3) + ...  (handle non-level segments via Eq 10.5)
        cmd.h_c = -r(3);   % placeholder: level flight at line altitude

        % Course command via cross-track error (Eqs 10.1-10.4)
        % e_py = cross-track error in path frame
        % chi_q = atan2(q(2), q(1))           % line heading
        % e_py  = -sin(chi_q)*(p(1)-r(1)) + cos(chi_q)*(p(2)-r(2))
        % chi_c = chi_q - chi_inf * (2/pi) * atan(k_path * e_py)  (Eq 10.4)

        chi_q   = atan2(q(2), q(1));           % heading of the line
        e_py    = 0;   % TODO: compute cross-track error (Eq 10.3)
        chi_inf = P.chi_inf;    % approach angle (add to P, ~70 deg)
        k_path  = P.k_path;    % gain (add to P)

        cmd.chi_c  = chi_q - chi_inf*(2/pi)*atan(k_path*e_py);  % Eq 10.4
        cmd.phi_c  = 0;         % TODO: optional feed-forward roll command

    case 2   %% ---- Orbit Following (Algorithm 6) ----
        % Reference: Eqs 10.9-10.15

        % Unpack
        c      = path.c;        % orbit centre [pn;pe;pd]
        rho    = path.rho;      % radius [ft]
        lambda = path.lambda;   % +1=CW, -1=CCW

        p  = [state.pn; state.pe; -state.h];

        % Altitude command: fly at orbit centre altitude (Eq 10.13)
        cmd.h_c = -c(3);        % TODO Eq 10.13

        % Radial error (Eq 10.9)
        % d = norm(p(1:2) - c(1:2))           % horizontal distance to centre
        % e_rho = d - rho                      % radial error

        d       = 0;   % TODO: compute distance to orbit centre
        e_rho   = 0;   % TODO: d - rho  (Eq 10.9)

        % Course command (Eq 10.12-10.15)
        % phi_orbit = atan2(p(2)-c(2), p(1)-c(1))   % azimuth to MAV
        % chi_c = phi_orbit + lambda*(pi/2 + atan(k_orbit * e_rho))
        phi_orb    = 0;   % TODO: atan2 (Eq 10.11)
        k_orbit    = P.k_orbit;    % gain (add to P)

        cmd.chi_c  = phi_orb + lambda*(pi/2 + atan(k_orbit*e_rho));  % Eq 10.12
        cmd.phi_c  = lambda * atan(state.Va^2 / (P.g * rho));         % Eq 10.15

    otherwise
        error('path_follower: unknown path.flag = %d', path.flag);
end

end
