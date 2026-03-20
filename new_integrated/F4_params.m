%% F4_params.m
% =========================================================
% Master Parameter File — F-4 Phantom (Subsonic Cruise)
% AE700 — IIT Bombay
%
% Returns BOTH:
%   (a) struct P  — for all function-based code
%   (b) workspace scalars — for Simulink S-Function dialog parameters
%
% Source: Roskam, Airplane Flight Dynamics, Part I, pg 536-538
% Units: ft, slug, lb, rad throughout
% =========================================================

clear P;

%% ===== Physical Constants =====
P.g   = 32.174;          % gravity [ft/s^2]
P.rho = 0.002377;        % air density at sea level [slug/ft^3]

%% ===== Mass & Inertia =====
Weight   = 39000;        % [lb]
P.mass   = Weight / P.g; % [slug]  ~= 1211.6
P.Jx     = 25000;        % [slug.ft^2]
P.Jy     = 122200;
P.Jz     = 139800;
P.Jxz    = 2200;

% Inertia matrix (for reference / verification)
P.J = [P.Jx,  0,     -P.Jxz;
       0,      P.Jy,  0;
      -P.Jxz,  0,     P.Jz];

%% ===== Geometry =====
P.bw    = 38.7;           % wingspan [ft]
P.cbar  = 16.0;           % mean aerodynamic chord [ft]
P.Sw    = 530.0;          % wing area [ft^2]

%% ===== Stall & Drag Model Parameters =====
P.M      = 50;            % sigmoid transition steepness
P.alpha0 = 0.4712;        % stall onset angle [rad] (~27 deg)
P.e      = 0.9;           % Oswald efficiency factor
P.CDp    = 0.0205;        % parasitic (zero-lift) drag

%% ===== Aerodynamic Coefficients — Lift =====
P.CLo    = 0.100;         % CL at zero alpha
P.CL_a   = 3.750;         % dCL/dalpha [/rad]
P.CL_q   = 1.8;           % dCL/d(qc/2Va) [/rad]
P.CL_de  = 0.40;          % dCL/d(de)   [/rad]

%% ===== Aerodynamic Coefficients — Drag =====
P.CD_q   = 0.0;           % dCD/d(qc/2Va)
P.CD_de  = -0.10;         % dCD/d(de) [/rad]

%% ===== Aerodynamic Coefficients — Pitching Moment =====
P.Cmo    = 0.025;         % Cm at zero alpha / trim
P.Cm_a   = -0.400;        % dCm/dalpha  [/rad] — negative = pitch stable
P.Cm_adot= -1.3;          % dCm/d(alpha_dot) [/rad]
P.Cm_q   = -2.7;          % pitch damping [/rad]
P.Cm_de  = -0.580;        % dCm/d(de) [/rad] — negative = nose-down de>0

%% ===== Aerodynamic Coefficients — Side Force =====
P.CY0    = 0.0;
P.CY_beta= -0.68;         % [/rad]
P.CY_p   =  0.0;
P.CY_r   =  0.0;
P.CY_da  =  -0.016;        % [/rad]
P.CY_dr  =  0.095;        % [/rad]

%% ===== Aerodynamic Coefficients — Roll Moment =====
P.Cl0    = 0.0;
P.Cl_beta= -0.080;        % dihedral stability [/rad]
P.Cl_p   = -0.240;        % roll damping [/rad]
P.Cl_r   =  0.070;        % [/rad]
P.Cl_da  = +0.042;        % aileron control power [/rad]
P.Cl_dr  =  0.0060;       % rudder cross-coupling [/rad]

%% ===== Aerodynamic Coefficients — Yaw Moment =====
P.Cn0    = 0.0;
P.Cn_beta=  0.125;        % weathercock stability [/rad]
P.Cn_p   = -0.036;        % [/rad]
P.Cn_r   = -0.270;        % yaw damping [/rad]
P.Cn_da  =  -0.0010;       % adverse yaw [/rad]
P.Cn_dr  = -0.066;        % rudder control power [/rad]

%% ===== Propulsion =====
% Simplified model: T = rho*Sprop*Cprop*((kmotor*dt)^2 - Va^2)
% kmotor sized so Va=845, dt=0.913 gives required trim thrust
P.Sprop  = pi*(2.0)^2;    % propeller disk area [ft^2]
P.Cprop  = 1.0;           % propeller efficiency coefficient
P.kmotor = 1200;          % motor constant [ft/s per unit throttle]
P.kTp    = 0.0;           % propeller reaction torque constant
P.kOmega = 0.0;           % propeller angular velocity gain

%% ===== Control Surface Limits =====
P.de_max =  40*pi/180;    % elevator  +/- [rad]
P.de_min = -40*pi/180;
P.da_max =  40*pi/180;    % aileron   +/- [rad]
P.da_min = -40*pi/180;
P.dr_max =  20*pi/180;    % rudder    +/- [rad]
P.dr_min = -20*pi/180;
P.dt_max =  1.0;
P.dt_min =  0.0;

%% ===== Trim / Cruise Conditions =====
P.Va_trim = 845;          % trim airspeed [ft/s]  (~Mach 0.85 SL)

% Compute alpha_trim from level-flight lift balance: L = W
CL_trim = Weight / (0.5 * P.rho * P.Va_trim^2 * P.Sw);
alpha_trim_calc = (CL_trim - P.CLo) / P.CL_a;
% At Va=845, CL_trim is very small; clamp to physical range
P.alpha_trim = max(-0.01, min(0.10, alpha_trim_calc));
P.theta_trim = P.alpha_trim;   % level flight: gamma=0 => theta=alpha

% Elevator for trim pitching moment balance
P.de_trim = -(P.Cmo + P.Cm_a * P.alpha_trim) / P.Cm_de;
P.dt_trim = 0.913;         % throttle at trim (sustains Va=845 ft/s)

%% ===== Gamma (Inertia) Constants  [eq 3.13] =====
Gamma    = P.Jx*P.Jz - P.Jxz^2;
P.Gamma  = Gamma;
P.Gamma1 = P.Jxz*(P.Jx - P.Jy + P.Jz) / Gamma;
P.Gamma2 = (P.Jz*(P.Jz - P.Jy) + P.Jxz^2) / Gamma;
P.Gamma3 = P.Jz  / Gamma;
P.Gamma4 = P.Jxz / Gamma;
P.Gamma5 = (P.Jz - P.Jx) / P.Jy;
P.Gamma6 = P.Jxz / P.Jy;
P.Gamma7 = ((P.Jx - P.Jy)*P.Jx + P.Jxz^2) / Gamma;
P.Gamma8 = P.Jx  / Gamma;

%% ===== Workspace Scalars for Simulink S-Function =====
% chap3_eom.m reads these as dialog parameters
mass = P.mass;
I_xx = P.Jx;   I_yy = P.Jy;   I_zz = P.Jz;   I_xz = P.Jxz;
Va_trim    = P.Va_trim;
alpha_trim = P.alpha_trim;
theta_trim = P.theta_trim;
g          = P.g;

%% ===== Print Summary =====
fprintf('===== F-4 Phantom Parameters Loaded =====\n');
fprintf('  Weight  : %.0f lb    |  mass : %.1f slug\n', Weight, P.mass);
fprintf('  Wingspan: %.1f ft   |  chord: %.1f ft   |  Sw: %.0f ft^2\n', P.bw, P.cbar, P.Sw);
fprintf('  Jx=%-6.0f  Jy=%-7.0f  Jz=%-7.0f  Jxz=%.0f  [slug-ft^2]\n', P.Jx,P.Jy,P.Jz,P.Jxz);
fprintf('  Va_trim=%.0f ft/s  |  alpha_trim=%.3f deg\n', P.Va_trim, rad2deg(P.alpha_trim));
fprintf('  de_trim=%.4f rad  |  dt_trim=%.3f\n', P.de_trim, P.dt_trim);
fprintf('==========================================\n\n');
