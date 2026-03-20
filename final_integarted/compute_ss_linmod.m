function [SS_lon, SS_lat, A_full, B_full, C_full, D_full] = ...
         compute_ss_linmod(x_trim, u_trim, P)
% =========================================================
% compute_ss_linmod.m
% Extracts lateral and longitudinal state-space models by
% linearising mavsim_trim.slx at the trim condition using
% Simulink's linmod command (Appendix F.3 of Beard & McLain).
%
% Complements compute_ss_models.m (analytical, Tables 5.1/5.2).
% Both should give the same eigenvalues — use this to verify.
%
% INPUTS:
%   x_trim  - 12x1 trim state  [pn..r]
%   u_trim  - 4x1  trim input  [de,da,dr,dt]
%   P       - parameter struct
%
% OUTPUTS:
%   SS_lon  - longitudinal ss object (states: u,w,q,theta,h)
%   SS_lat  - lateral      ss object (states: v,p,r,phi,psi)
%   A_full, B_full, C_full, D_full — full 12-state linearisation
%
% REQUIRES: mavsim_trim.slx to be on MATLAB path or in current dir.
%           Build it first with build_mavsim_trim.m
% =========================================================

model = 'mavsim_trim';

%% ---- Load model if not already loaded ----
if ~bdIsLoaded(model)
    if exist([model '.slx'],'file')
        load_system(model);
    else
        error('compute_ss_linmod: %s.slx not found. Run build_mavsim_trim.m first.', model);
    end
end

%% ---- Set initial conditions in EOM block to trim state ----
% Format parameter string for chap3_eom dialog
pstr = sprintf('%g,%g,%g,%g,%g, %g,%g,%g, %g,%g,%g, %g,%g,%g, %g,%g,%g', ...
    P.mass, P.Jx, P.Jy, P.Jz, P.Jxz, ...
    x_trim(1), x_trim(2), x_trim(3), ...
    x_trim(4), x_trim(5), x_trim(6), ...
    x_trim(7), x_trim(8), x_trim(9), ...
    x_trim(10),x_trim(11),x_trim(12));
set_param([model '/EOM'], 'Parameters', pstr);

% Set Memory block initial condition
mem_ic = sprintf('[%s]', num2str(x_trim','%g,'));
set_param([model '/StateMem'], 'X0', mem_ic);

%% ---- Run linmod (Appendix F.3) ----
fprintf('Running linmod on %s at trim condition...\n', model);
[A_full, B_full, C_full, D_full] = linmod(model, x_trim, u_trim);

fprintf('  Full system: A(%dx%d)  B(%dx%d)  C(%dx%d)  D(%dx%d)\n', ...
        size(A_full,1),size(A_full,2), size(B_full,1),size(B_full,2), ...
        size(C_full,1),size(C_full,2), size(D_full,1),size(D_full,2));

%% ============================================================
%  EXTRACT LONGITUDINAL STATE-SPACE  (eq 5.50)
%  States: [u, w, q, theta, h]  ->  indices [4,6,11,8,3*]
%  *h = -pd, so we use pd index=3 but flip sign in C matrix
%  Inputs: [delta_e, delta_t]   ->  indices [1,4]
% ============================================================

% State indices in full 12-state vector
%   pn=1,pe=2,pd=3,u=4,v=5,w=6,phi=7,theta=8,psi=9,p=10,q=11,r=12
lon_states = [4, 6, 11, 8, 3];   % u, w, q, theta, pd
lat_states  = [5, 10, 12, 7, 9]; % v, p, r, phi, psi
lon_inputs  = [1, 4];             % delta_e, delta_t
lat_inputs  = [1, 2];             % delta_a, delta_r — NOTE: col 1 of B is de

% For lateral we want inputs [delta_a, delta_r] = columns [2,3]
lat_inputs_B = [2, 3];

% Extract submatrices
A_lon = A_full(lon_states, lon_states);
B_lon = B_full(lon_states, lon_inputs);

A_lat = A_full(lat_states, lat_states);
B_lat = B_full(lat_states, lat_inputs_B);

% Flip pd row/col sign so h = -pd convention is consistent
% Row 5 of A_lon corresponds to pd_dot; h_dot = -pd_dot
% We negate the 5th row and 5th column
A_lon(5,:) = -A_lon(5,:);
A_lon(:,5) = -A_lon(:,5);
B_lon(5,:) = -B_lon(5,:);

%% ---- Build ss objects ----
SS_lon = ss(A_lon, B_lon, eye(5), zeros(5,2));
SS_lon.StateName  = {'u','w','q','theta','h'};
SS_lon.InputName  = {'delta_e','delta_t'};
SS_lon.OutputName = {'u','w','q','theta','h'};

SS_lat = ss(A_lat, B_lat, eye(5), zeros(5,2));
SS_lat.StateName  = {'v','p','r','phi','psi'};
SS_lat.InputName  = {'delta_a','delta_r'};
SS_lat.OutputName = {'v','p','r','phi','psi'};

%% ============================================================
%  PRINT AND COMPARE EIGENVALUES
% ============================================================
fprintf('\n=== linmod Longitudinal State-Space (eq 5.50) ===\n');
fprintf('States: u, w, q, theta, h\n\n');
fprintf('A_lon =\n'); disp(A_lon);
fprintf('B_lon =\n'); disp(B_lon);

lons = eig(A_lon);
fprintf('Eigenvalues:\n');
for lam = lons.'
    if abs(imag(lam)) > 1e-4
        wn=abs(lam); z=-real(lam)/wn;
        if real(lam) > 0
            tag = 'UNSTABLE';
        else
            tag = 'stable';
        end
        fprintf('  %+.4f%+.4fi   wn=%.4f rad/s  zeta=%.4f  [%s]\n', ...
                real(lam),imag(lam),wn,z,tag);
    else
        if abs(real(lam)) < 1e-4
            fprintf('  %+.6f  [integrator]\n', real(lam));
        elseif real(lam) > 0
            fprintf('  %+.6f  [UNSTABLE]\n', real(lam));
        else
            fprintf('  %+.6f  [stable]\n', real(lam));
        end
    end
end

fprintf('\n=== linmod Lateral State-Space (eq 5.43) ===\n');
fprintf('States: v, p, r, phi, psi\n\n');
fprintf('A_lat =\n'); disp(A_lat);
fprintf('B_lat =\n'); disp(B_lat);

lats = eig(A_lat);
fprintf('Eigenvalues:\n');
for lam = lats.'
    if abs(imag(lam)) > 1e-4
        wn=abs(lam); z=-real(lam)/wn;
        if real(lam) > 0
            tag = 'UNSTABLE';
        else
            tag = 'stable  Dutch roll';
        end
        fprintf('  %+.4f%+.4fi   wn=%.4f rad/s  zeta=%.4f  [%s]\n', ...
                real(lam),imag(lam),wn,z,tag);
    else
        if abs(real(lam)) < 1e-4
            fprintf('  %+.6f  [heading integrator]\n', real(lam));
        elseif real(lam) > 1e-4
            fprintf('  %+.6f  [spiral — unstable]\n', real(lam));
        else
            fprintf('  %+.6f  [roll subsidence]\n', real(lam));
        end
    end
end

fprintf('\n=== Cross-check: compare with analytical (compute_ss_models) ===\n');
fprintf('Eigenvalues should match to within numerical precision.\n\n');

end
