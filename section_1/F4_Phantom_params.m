%% F4_Phantom_params.m
% Aircraft Parameters for F-4 Phantom (Subsonic Cruise Configuration)
% AE700 - IIT Bombay
%
% Source: aircraft_dat.txt (Roskam, Airplane Flight Dynamics, Part I, pg 536-538)
%
% Units: ft, slug, lb, rad unless noted

%% ===== Geometry =====
bw   = 38.7;      % Wingspan [ft]
cbar = 16.0;      % Mean aerodynamic chord [ft]
Sw   = 530.0;     % Wing area [ft^2]

%% ===== Mass & Inertia =====
Weight = 39000;   % Weight [lb]
g      = 32.174;  % Gravity [ft/s^2]
mass   = Weight / g;   % Mass [slug]

I_xx   = 25000;   % Roll  moment of inertia [slug-ft^2]
I_yy   = 122200;  % Pitch moment of inertia [slug-ft^2]
I_zz   = 139800;  % Yaw   moment of inertia [slug-ft^2]
I_xz   =   2200;  % Product of inertia      [slug-ft^2]
I_xy   = 0;
I_yz   = 0;

% Inertia matrix (body frame)
J = [I_xx,  0,    -I_xz;
     0,     I_yy,  0;
    -I_xz,  0,     I_zz];

%% ===== Engine =====
T_max  = 24000;   % Max thrust [lb] (both engines combined)

%% ===== Aerodynamic Coefficients =====
% Lift
CLo    = 0.100;   % Zero-alpha lift coefficient
CL_a   = 3.750;   % Lift curve slope [/rad]
CL_adot= 0.86;    % Alpha-rate effect [/rad]
CL_q   = 1.8;     % Pitch rate effect [/rad]
CL_de  = 0.40;    % Elevator effectiveness [/rad]

% Drag
CDo    = 0.0205;  % Parasite drag
CD_a   = 0.300;   % [/rad]
CD_de  = -0.10;   % [/rad]

% Pitching Moment
Cmo    = 0.025;   % Zero-alpha pitching moment
Cm_a   = -0.400;  % Pitch stiffness [/rad]
Cm_adot= -1.3;    % [/rad]
Cm_q   = -2.7;    % Pitch damping [/rad]
Cm_de  = -0.580;  % Elevator effectiveness [/rad]

% Side Force
CY_beta= -0.68;   % [/rad]
CY_p   =  0.0;
CY_r   =  0.0;
CY_da  =  0.016;  % [/rad]
CY_dr  =  0.095;  % [/rad]

% Rolling Moment
Cl_beta= -0.080;  % Dihedral effect [/rad]
Cl_p   = -0.240;  % Roll damping [/rad]
Cl_r   =  0.070;  % [/rad]
Cl_da  = -0.042;  % Aileron effectiveness [/rad]
Cl_dr  =  0.0060; % Rudder-roll coupling [/rad]

% Yawing Moment
Cn_beta=  0.125;  % Weathercock stability [/rad]
Cn_p   = -0.036;  % [/rad]
Cn_r   = -0.270;  % Yaw damping [/rad]
Cn_da  =  0.0010; % Adverse yaw [/rad]
Cn_dr  = -0.066;  % Rudder effectiveness [/rad]

%% ===== Control Surface Limits =====
de_max =  40 * pi/180;  % Elevator  +/- [rad]
de_min = -40 * pi/180;
da_max =  40 * pi/180;  % Aileron   +/- [rad]
da_min = -40 * pi/180;
dr_max =  20 * pi/180;  % Rudder    +/- [rad]
dr_min = -20 * pi/180;

%% ===== Trim / Cruise Conditions (typical subsonic) =====
% Used for initial conditions in simulation
Va_trim    = 845;         % Trim airspeed [ft/s] (~Mach 0.85 @ sea level)
alpha_trim = 5 * pi/180;  % Trim AoA [rad]
theta_trim = alpha_trim;  % Level flight: gamma=0 -> theta=alpha
h_trim     = 0;           % Altitude [ft] (sea level for initial test)

fprintf('F-4 Phantom parameters loaded.\n');
fprintf('  Weight: %.0f lb | Wingspan: %.1f ft | Sw: %.0f ft^2\n', Weight, bw, Sw);
fprintf('  I_xx=%.0f  I_yy=%.0f  I_zz=%.0f  I_xz=%.0f [slug-ft^2]\n', ...
        I_xx, I_yy, I_zz, I_xz);
