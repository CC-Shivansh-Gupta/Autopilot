function out = forces_moments(x, delta, wind, P)
% =========================================================
% forces_moments.m
% Computes total forces and moments on F-4 Phantom in body frame.
% Implements gravity (eq 4.18), aerodynamics (eq 4.18-4.19),
% and propulsion (eq 4.18, 4.20) from Beard & McLain Ch. 4.
%
% INPUTS:
%   x     - 12x1 state vector [pn,pe,pd,u,v,w,phi,theta,psi,p,q,r]
%   delta - 4x1 control inputs [de, da, dr, dt]
%             de = elevator deflection  [rad]
%             da = aileron  deflection  [rad]
%             dr = rudder   deflection  [rad]
%             dt = throttle             [0..1]
%   wind  - 6x1 wind vector
%             [wn,we,wd]' steady wind in inertial frame [ft/s]
%             [ug,vg,wg]' gust in body frame [ft/s]
%   P     - struct of aircraft parameters (from F4_Phantom_params)
%
% OUTPUTS (13x1):
%   out(1)  = fx    body-x force  [lb]
%   out(2)  = fy    body-y force  [lb]
%   out(3)  = fz    body-z force  [lb]
%   out(4)  = l     roll  moment  [lb.ft]
%   out(5)  = m     pitch moment  [lb.ft]
%   out(6)  = n     yaw   moment  [lb.ft]
%   out(7)  = Va    airspeed      [ft/s]
%   out(8)  = alpha angle of attack [rad]
%   out(9)  = beta  sideslip angle  [rad]
%   out(10) = wn    north wind (inertial) [ft/s]
%   out(11) = we    east  wind (inertial) [ft/s]
%   out(12) = wd    down  wind (inertial) [ft/s]
% =========================================================

%% ---- Unpack states ----
% pn = x(1); pe = x(2); pd = x(3);
u     = x(4);  v  = x(5);  w  = x(6);
phi   = x(7);  theta = x(8);  psi = x(9);
p     = x(10); q  = x(11); r  = x(12);

%% ---- Unpack controls ----
de = delta(1);   % elevator  [rad]
da = delta(2);   % aileron   [rad]
dr = delta(3);   % rudder    [rad]
dt = delta(4);   % throttle  [0..1]

%% ---- Unpack wind ----
wn_s = wind(1);  we_s = wind(2);  wd_s = wind(3);  % steady, inertial
ug   = wind(4);  vg   = wind(5);  wg   = wind(6);  % gust, body frame

%% ---- Rotation matrix: inertial -> body (R_bv) ----
cphi=cos(phi); sphi=sin(phi);
cth =cos(theta); sth=sin(theta);
cpsi=cos(psi); spsi=sin(psi);

R_bv = [...
    cth*cpsi,                   cth*spsi,                  -sth;
    sphi*sth*cpsi-cphi*spsi,    sphi*sth*spsi+cphi*cpsi,    sphi*cth;
    cphi*sth*cpsi+sphi*spsi,    cphi*sth*spsi-sphi*cpsi,    cphi*cth];

%% ---- Total wind in body frame (eq 4.17 + steady+gust) ----
% Rotate steady inertial wind into body frame, add gust
wind_steady_inertial = [wn_s; we_s; wd_s];
wind_body = R_bv * wind_steady_inertial + [ug; vg; wg];

uw = wind_body(1);
vw = wind_body(2);
ww = wind_body(3);

%% ---- Relative airspeed in body frame ----
ur = u - uw;
vr = v - vw;
wr = w - ww;

%% ---- Va, alpha, beta (eq 2.8 / section 4.4) ----
Va = sqrt(ur^2 + vr^2 + wr^2);
if Va < 1e-3
    Va    = 1e-3;   % avoid divide-by-zero
    alpha = 0;
    beta  = 0;
else
    alpha = atan2(wr, ur);
    beta  = asin(vr / Va);
end

%% ---- Dynamic pressure ----
rho = P.rho;
qbar = 0.5 * rho * Va^2;   % dynamic pressure [lb/ft^2]

%% ============================================================
%  GRAVITY FORCES (body frame)  -- eq 4.18 top block
% ============================================================
fg_x = -P.mass * P.g * sth;
fg_y =  P.mass * P.g * cth * sphi;
fg_z =  P.mass * P.g * cth * cphi;

%% ============================================================
%  AERODYNAMIC COEFFICIENTS (Sections 4.2.2 and 4.2.3)
% ============================================================

%-- Nonlinear CL(alpha) with stall blending (eq 4.9) --%
sigma = (1 + exp(-P.M*(alpha - P.alpha0)) + exp(P.M*(alpha + P.alpha0))) / ...
        ((1 + exp(-P.M*(alpha - P.alpha0))) * (1 + exp(P.M*(alpha + P.alpha0))));
CL_alpha = (1 - sigma) * (P.CLo + P.CL_a * alpha) + ...
            sigma * (2 * sign(alpha) * sin(alpha)^2 * cos(alpha));

%-- Nonlinear CD(alpha) with quadratic induced drag (eq 4.11) --%
AR = P.bw^2 / P.Sw;
CD_alpha = P.CDp + (P.CLo + P.CL_a * alpha)^2 / (pi * P.e * AR);

%-- Pitching moment coefficient (linear) --%
Cm_alpha = P.Cmo + P.Cm_a * alpha;

%-- Longitudinal: CX, CZ in body frame (eq 4.19) --%
CX      = -CD_alpha * cos(alpha) + CL_alpha * sin(alpha);
CX_q    = -P.CD_q  * cos(alpha) + P.CL_q   * sin(alpha);
CX_de   = -P.CD_de * cos(alpha) + P.CL_de  * sin(alpha);

CZ      = -CD_alpha * sin(alpha) - CL_alpha * cos(alpha);
CZ_q    = -P.CD_q  * sin(alpha) - P.CL_q   * cos(alpha);
CZ_de   = -P.CD_de * sin(alpha) - P.CL_de  * cos(alpha);

%% ============================================================
%  AERODYNAMIC FORCES (eq 4.18)
% ============================================================
c2Va = P.cbar / (2 * Va);
b2Va = P.bw   / (2 * Va);

% Body-x force (longitudinal)
fa_x = qbar * P.Sw * (CX + CX_q * c2Va * q + CX_de * de);

% Body-y force (lateral, eq 4.14)
fa_y = qbar * P.Sw * (P.CY0 + P.CY_beta*beta + ...
                       P.CY_p*b2Va*p + P.CY_r*b2Va*r + ...
                       P.CY_da*da + P.CY_dr*dr);

% Body-z force (longitudinal)
fa_z = qbar * P.Sw * (CZ + CZ_q * c2Va * q + CZ_de * de);

%% ============================================================
%  PROPULSION FORCES (eq 4.18 bottom, Section 4.3.1)
%  Thrust along body-x axis only
% ============================================================
fp_x = 0.5 * rho * P.Sprop * P.Cprop * ((P.kmotor * dt)^2 - Va^2);
fp_y = 0;
fp_z = 0;

%% ============================================================
%  TOTAL FORCES
% ============================================================
fx = fg_x + fa_x + fp_x;
fy = fg_y + fa_y + fp_y;
fz = fg_z + fa_z + fp_z;

%% ============================================================
%  AERODYNAMIC MOMENTS (eq 4.20, Sections 4.2.2-4.2.3)
% ============================================================
% Roll moment l (eq 4.15)
l_aero = qbar * P.Sw * P.bw * (P.Cl0 + P.Cl_beta*beta + ...
          P.Cl_p*b2Va*p + P.Cl_r*b2Va*r + ...
          P.Cl_da*da + P.Cl_dr*dr);

% Pitch moment m (eq 4.5 extended)
m_aero = qbar * P.Sw * P.cbar * (Cm_alpha + P.Cm_q*c2Va*q + P.Cm_de*de);

% Yaw moment n (eq 4.16)
n_aero = qbar * P.Sw * P.bw * (P.Cn0 + P.Cn_beta*beta + ...
          P.Cn_p*b2Va*p + P.Cn_r*b2Va*r + ...
          P.Cn_da*da + P.Cn_dr*dr);

%% ============================================================
%  PROPULSION MOMENTS (eq 4.20, Section 4.3.2)
%  Propeller torque opposes rotation, acts about body-x
% ============================================================
Omega_p = P.kOmega * dt;          % propeller angular speed [rad/s]
l_prop  = -P.kTp * Omega_p^2;    % reaction torque
m_prop  = 0;
n_prop  = 0;

%% ---- Total moments ----
l_total = l_aero + l_prop;
m_total = m_aero + m_prop;
n_total = n_aero + n_prop;

%% ---- Pack output ----
out = [fx; fy; fz; l_total; m_total; n_total; ...
       Va; alpha; beta; ...
       wn_s; we_s; wd_s];
end
