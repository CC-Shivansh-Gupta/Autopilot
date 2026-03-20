%% F4_chap4_params.m  (FIXED)
% Aircraft parameters for F-4 Phantom — used in Sections 4 and 5
% AE700 - IIT Bombay
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
P.Cm_de  = -0.580;        % [/rad]

% -- Side force --
P.CY0    = 0;             % FIXED: explicitly initialised (symmetric aircraft)
P.CY_beta= -0.68;         % [/rad]
P.CY_p   =  0.0;
P.CY_r   =  0.0;
P.CY_da  =  0.016;        % [/rad]
P.CY_dr  =  0.095;

% -- Rolling moment --
P.Cl0    = 0;
P.Cl_beta= -0.080;
P.Cl_p   = -0.240;
P.Cl_r   =  0.070;
P.Cl_da  = -0.042;        % [/rad]
P.Cl_dr  =  0.0060;

% -- Yawing moment --
P.Cn0    = 0;
P.Cn_beta=  0.125;
P.Cn_p   = -0.036;
P.Cn_r   = -0.270;
P.Cn_da  =  0.0010;       % [/rad]
P.Cn_dr  = -0.066;

%% ===== Propulsion =====
P.Sprop  = pi*(2.0)^2;    % propeller disk area [ft^2]
P.Cprop  = 1.0;
P.kmotor = 1200;
P.kTp    = 0;             % jet engine: no propeller reaction torque
P.kOmega = 0;
% NOTE: kTp=0 and kOmega=0 because the F-4 is a jet aircraft.
% Propeller reaction torque (eq 4.20) is identically zero.

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
% The F-4 at Va=845 ft/s (Mach ~0.85 at sea level) generates sufficient
% lift at a very small AoA because dynamic pressure is high.
% CL_trim = W / (0.5*rho*Va^2*Sw) = 39000/(0.5*0.002377*845^2*530) = 0.0869
% alpha_trim = (CL_trim - CLo)/CL_a = (0.0869-0.1)/3.75 = -0.0035 rad = -0.20 deg
%
% The project asks for alpha=5-7 deg. At 845 ft/s the aircraft is
% aerodynamically over-powered. Using Va=350 ft/s gives alpha~5 deg
% which is physically consistent with low-speed subsonic cruise.
%
% CHOICE: We use Va_trim=350 ft/s so that alpha falls in the 5-7 deg
%         range as specified, consistent with subsonic manoeuvring.

P.Va_trim = 350;   % [ft/s]  -- gives alpha~5 deg at sea level

CL_trim = Weight / (0.5*P.rho*P.Va_trim^2*P.Sw);
P.alpha_trim = (CL_trim - P.CLo) / P.CL_a;   % [rad]
P.theta_trim = P.alpha_trim;                   % level flight: gamma=0
P.de_trim    = -(P.Cmo + P.Cm_a*P.alpha_trim) / P.Cm_de;
P.dt_trim    = 0.5;   % initial guess; refined by compute_trim

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

fprintf('F-4 Phantom params loaded.\n');
fprintf('  Va_trim = %.0f ft/s\n', P.Va_trim);
fprintf('  CL_trim = %.4f\n', CL_trim);
fprintf('  alpha_trim = %.3f deg  (target: 5-7 deg)\n', rad2deg(P.alpha_trim));
fprintf('  de_trim = %.4f rad\n', P.de_trim);
