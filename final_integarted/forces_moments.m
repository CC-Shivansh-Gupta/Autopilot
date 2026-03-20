function out = forces_moments(x, delta, wind, P)
% =========================================================
% forces_moments.m
% Total forces and moments on the F-4 Phantom in body frame.
% Implements eq (4.18)-(4.20) from Beard & McLain Ch. 4.
%
% INPUTS:
%   x     - 12x1  [pn,pe,pd,u,v,w,phi,theta,psi,p,q,r]'
%   delta - 4x1   [de, da, dr, dt]'
%   wind  - 6x1   [wn,we,wd]' inertial steady + [ug,vg,wg]' body gust
%   P     - param struct from F4_params.m
%
% OUTPUTS (13x1):
%   [fx, fy, fz, l, m, n, Va, alpha, beta, wn, we, wd]'
% =========================================================

%% Unpack states
u_b=x(4); v_b=x(5); w_b=x(6);
phi=x(7); theta=x(8); psi=x(9);
p=x(10);  q=x(11);   r=x(12);

%% Unpack controls
de=delta(1); da=delta(2); dr=delta(3); dt=delta(4);

%% Unpack wind
wn_s=wind(1); we_s=wind(2); wd_s=wind(3);
ug  =wind(4); vg  =wind(5); wg  =wind(6);

%% Body -> Inertial rotation matrix  R_bv  (eq 2.4)
cphi=cos(phi); sphi=sin(phi);
cth =cos(theta); sth=sin(theta);
cpsi=cos(psi);   spsi=sin(psi);

R_bv = [cth*cpsi,                  cth*spsi,                 -sth;
        sphi*sth*cpsi-cphi*spsi,   sphi*sth*spsi+cphi*cpsi,   sphi*cth;
        cphi*sth*cpsi+sphi*spsi,   cphi*sth*spsi-sphi*cpsi,   cphi*cth];

%% Total wind in body frame (steady rotated + gust)
wind_body = R_bv * [wn_s; we_s; wd_s] + [ug; vg; wg];
uw=wind_body(1); vw=wind_body(2); ww=wind_body(3);

%% Relative airspeed components
ur=u_b-uw; vr=v_b-vw; wr=w_b-ww;

%% Va, alpha, beta  (eq 2.8)
Va = sqrt(ur^2 + vr^2 + wr^2);
if Va < 1e-3
    Va=1e-3; alpha=0; beta=0;
else
    alpha = atan2(wr, ur);
    beta  = asin(max(-1, min(1, vr/Va)));   % clamp to avoid asin domain error
end

%% Dynamic pressure
qbar = 0.5 * P.rho * Va^2;
c2Va = P.cbar / (2*Va);
b2Va = P.bw   / (2*Va);

%% ============================================================
%  GRAVITY  (body-frame components)
% ============================================================
fg_x = -P.mass * P.g * sth;
fg_y =  P.mass * P.g * cth * sphi;
fg_z =  P.mass * P.g * cth * cphi;

%% ============================================================
%  NONLINEAR LIFT  CL(alpha)  with stall blending  (eq 4.9)
% ============================================================
sigma = (1 + exp(-P.M*(alpha-P.alpha0)) + exp(P.M*(alpha+P.alpha0))) / ...
        ((1+exp(-P.M*(alpha-P.alpha0))) * (1+exp(P.M*(alpha+P.alpha0))));
CL_nl = (1-sigma)*(P.CLo + P.CL_a*alpha) + ...
         sigma*(2*sign(alpha)*sin(alpha)^2*cos(alpha));

%% NONLINEAR DRAG  CD(alpha)  polar model  (eq 4.11)
AR   = P.bw^2 / P.Sw;
CD_nl = P.CDp + (P.CLo + P.CL_a*alpha)^2 / (pi * P.e * AR);

%% Pitching moment coefficient (linear)
Cm_al = P.Cmo + P.Cm_a * alpha;

%% Body-frame aero force coefficients  (eq 4.19)
CX    = -CD_nl*cos(alpha) + CL_nl*sin(alpha);
CX_q  = -P.CD_q *cos(alpha) + P.CL_q *sin(alpha);
CX_de = -P.CD_de*cos(alpha) + P.CL_de*sin(alpha);

CZ    = -CD_nl*sin(alpha) - CL_nl*cos(alpha);
CZ_q  = -P.CD_q *sin(alpha) - P.CL_q *cos(alpha);
CZ_de = -P.CD_de*sin(alpha) - P.CL_de*cos(alpha);

%% ============================================================
%  AERODYNAMIC FORCES  (eq 4.18)
% ============================================================
fa_x = qbar*P.Sw*(CX + CX_q*c2Va*q + CX_de*de);

fa_y = qbar*P.Sw*(P.CY0 + P.CY_beta*beta + ...
                  P.CY_p*b2Va*p + P.CY_r*b2Va*r + ...
                  P.CY_da*da + P.CY_dr*dr);

fa_z = qbar*P.Sw*(CZ + CZ_q*c2Va*q + CZ_de*de);

%% ============================================================
%  PROPULSION FORCES  (eq 4.18, Section 4.3.1)
% ============================================================
fp_x = 0.5*P.rho*P.Sprop*P.Cprop*((P.kmotor*dt)^2 - Va^2);

%% Total forces
fx = fg_x + fa_x + fp_x;
fy = fg_y + fa_y;
fz = fg_z + fa_z;

%% ============================================================
%  AERODYNAMIC MOMENTS  (eq 4.20)
% ============================================================
l_aero = qbar*P.Sw*P.bw*(P.Cl0 + P.Cl_beta*beta + ...
          P.Cl_p*b2Va*p + P.Cl_r*b2Va*r + P.Cl_da*da + P.Cl_dr*dr);

m_aero = qbar*P.Sw*P.cbar*(Cm_al + P.Cm_q*c2Va*q + P.Cm_de*de);

n_aero = qbar*P.Sw*P.bw*(P.Cn0 + P.Cn_beta*beta + ...
          P.Cn_p*b2Va*p + P.Cn_r*b2Va*r + P.Cn_da*da + P.Cn_dr*dr);

%% Propulsion moments  (propeller reaction torque about body-x)
Omega_p = P.kOmega * dt;
l_prop  = -P.kTp * Omega_p^2;

%% Total moments
l_tot = l_aero + l_prop;
m_tot = m_aero;
n_tot = n_aero;

%% Output
out = [fx; fy; fz; l_tot; m_tot; n_tot; Va; alpha; beta; wn_s; we_s; wd_s];
end
