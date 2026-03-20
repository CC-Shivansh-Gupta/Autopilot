%% F4_chap4_params.m  (FIXED — consistent with F4_Phantom_params.m)
% Aircraft parameters for F-4 Phantom — used in Sections 4 and 5
% AE700 - IIT Bombay
%
% FIX NOTES vs previous version:
%   1. Va_trim corrected to 845 ft/s (Mach 0.85, from F4_Phantom_params)
%   2. kmotor recomputed to match required thrust at Va=845 ft/s
%   3. Cm_de sign corrected to match Roskam source (-0.580, not +0.580)
%   4. CY_da, Cl_da, Cn_da signs corrected to match F4_Phantom_params
%   5. alpha_trim recomputed for level flight at Va=845 ft/s
%
% Units: ft, slug, lb, rad

clear P;

%% ===== Physical constants =====
P.g   = 32.174;          % gravity [ft/s^2]
P.rho = 0.002377;        % air density at sea level [slug/ft^3]

%% ===== Mass & Inertia =====
Weight   = 39000;        % [lb]
P.mass   = Weight / P.g; % [slug] = 1211.6
P.Jx     = 25000;        % [slug.ft^2]
P.Jy     = 122200;
P.Jz     = 139800;
P.Jxz    = 2200;

%% ===== Geometry =====
P.bw    = 38.7;           % wingspan [ft]
P.cbar  = 16.0;           % mean chord [ft]
P.Sw    = 530.0;          % wing area [ft^2]

%% ===== Aerodynamic parameters =====
P.M      = 50;            % sigmoid stall transition rate
P.alpha0 = 0.4712;        % stall angle [rad] ~27 deg
P.e      = 0.9;           % Oswald efficiency
P.CDp    = 0.0205;        % parasite drag (CD0)

% -- Lift --
P.CLo    = 0.100;
P.CL_a   = 3.750;         % [/rad]
P.CL_q   = 1.8;           % [/rad]
P.CL_de  = 0.40;          % [/rad]

% -- Drag --
P.CD_q   = 0;
P.CD_de  = -0.10;         % [/rad]

% -- Pitching moment --
P.Cmo    = 0.025;
P.Cm_a   = -0.400;        % [/rad]
P.Cm_adot= -1.3;
P.Cm_q   = -2.7;          % [/rad]
P.Cm_de  = -0.580;        % [/rad]  FIXED: negative (Roskam sign, nose-up de < 0)

% -- Side force --
P.CY0    = 0;
P.CY_beta= -0.68;         % [/rad]
P.CY_p   =  0.0;
P.CY_r   =  0.0;
P.CY_da  =  0.016;        % [/rad]  FIXED: positive (matches F4_Phantom_params)
P.CY_dr  =  0.095;

% -- Rolling moment --
P.Cl0    = 0;
P.Cl_beta= -0.080;
P.Cl_p   = -0.240;
P.Cl_r   =  0.070;
P.Cl_da  = -0.042;        % [/rad]  FIXED: negative (matches F4_Phantom_params)
P.Cl_dr  =  0.0060;

% -- Yawing moment --
P.Cn0    = 0;
P.Cn_beta=  0.125;
P.Cn_p   = -0.036;
P.Cn_r   = -0.270;
P.Cn_da  =  0.0010;       % [/rad]  FIXED: positive (matches F4_Phantom_params)
P.Cn_dr  = -0.066;

%% ===== Propulsion =====
% kmotor sized so that at Va=845 ft/s, dt=0.5 gives approx T=24000 lb (max thrust/2)
% Thrust model: T = rho*Sprop*Cprop*((kmotor*dt)^2 - Va^2)
% At trim: T_trim = Weight * sin(alpha_trim) + D_trim  (approx)
% Working value: kmotor = sqrt(Va^2 + T_trim/(rho*Sprop*Cprop)) / dt_trim
P.Sprop  = pi*(2.0)^2;    % propeller disk area [ft^2]
P.Cprop  = 1.0;
P.kmotor = 1200;   % was 650 — FIXED: gives T=9750 lb at dt=0.85, need 9505 lb
P.kTp    = 0;
P.kOmega = 0;

%% ===== Control Surface Limits =====
P.de_max =  40*pi/180;
P.de_min = -40*pi/180;
P.da_max =  40*pi/180;
P.da_min = -40*pi/180;
P.dr_max =  20*pi/180;
P.dr_min = -20*pi/180;
P.dt_max =  1.0;
P.dt_min =  0.0;

%% ===== Trim / Initial Conditions =====
% FIXED: Use Va_trim=845 ft/s consistent with F4_Phantom_params
P.Va_trim    = 845;              % [ft/s]

% Compute alpha_trim from level flight lift balance:
%   L = W  =>  0.5*rho*Va^2*Sw*CL = W
%   CL_trim = W/(0.5*rho*Va^2*Sw)
CL_trim = Weight / (0.5*P.rho*P.Va_trim^2*P.Sw);
fprintf('CL_trim = %.4f\n', CL_trim);
% CL = CLo + CL_a * alpha  =>  alpha = (CL_trim - CLo)/CL_a
alpha_trim_computed = (CL_trim - P.CLo) / P.CL_a;
if alpha_trim_computed < 0 || alpha_trim_computed > 0.3
    % If Va is high enough that CL_trim < CLo, clamp to small positive value
    alpha_trim_computed = max(0.02, min(0.15, alpha_trim_computed));
    fprintf('  Note: alpha_trim clamped to %.3f rad (%.1f deg)\n', ...
            alpha_trim_computed, rad2deg(alpha_trim_computed));
end

P.alpha_trim = -0.0035;   % rad = -0.2 deg (computed from lift balance)
P.theta_trim = P.alpha_trim;
P.de_trim = -(P.Cmo + P.Cm_a*P.alpha_trim) / P.Cm_de;

% Trim throttle: initial guess (will be refined by compute_trim)
P.dt_trim = 0.85;

%% ===== Gamma constants (eq 3.13) =====
Gamma    = P.Jx*P.Jz - P.Jxz^2;
P.Gamma1 = P.Jxz*(P.Jx - P.Jy + P.Jz) / Gamma;
P.Gamma2 = (P.Jz*(P.Jz - P.Jy) + P.Jxz^2) / Gamma;
P.Gamma3 = P.Jz  / Gamma;
P.Gamma4 = P.Jxz / Gamma;
P.Gamma5 = (P.Jz - P.Jx) / P.Jy;
P.Gamma6 = P.Jxz / P.Jy;
P.Gamma7 = ((P.Jx - P.Jy)*P.Jx + P.Jxz^2) / Gamma;
P.Gamma8 = P.Jx  / Gamma;

fprintf('F-4 Phantom params loaded (FIXED for Va=%.0f ft/s).\n', P.Va_trim);
fprintf('  mass=%.1f slug, alpha_trim=%.2f deg, de_trim=%.3f rad, dt_trim=%.2f\n', ...
        P.mass, rad2deg(P.alpha_trim), P.de_trim, P.dt_trim);
