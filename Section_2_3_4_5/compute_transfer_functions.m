function TF = compute_transfer_functions(x_trim, u_trim, P)
% =========================================================
% compute_transfer_functions.m
% Computes the transfer functions listed in Section 5.4
% Beard & McLain, "Small Unmanned Aircraft"
%
% INPUTS:
%   x_trim - 12x1 trim state vector
%   u_trim - 4x1  trim control vector [de,da,dr,dt]
%   P      - aircraft parameter struct
%
% OUTPUT:
%   TF - struct with transfer function objects:
%     TF.phi_da    : phi(s)/da(s)    eq (5.26)
%     TF.chi_phi   : chi(s)/phi(s)   eq (5.27)
%     TF.beta_dr   : beta(s)/dr(s)   eq (5.28)
%     TF.theta_de  : theta(s)/de(s)  eq (5.29)
%     TF.h_theta   : h(s)/theta(s)   eq (5.31)
%     TF.h_Va      : h(s)/Va(s)      eq (5.32)
%     TF.Va_dt     : Va(s)/dt(s)     eq (5.36)
%     TF.Va_theta  : Va(s)/theta(s)  eq (5.36)
% =========================================================

%% ---- Unpack trim state ----
u_v   = x_trim(4);  v_v = x_trim(5);  w_v  = x_trim(6);
phi_s = x_trim(7);  theta_s = x_trim(8);
p_s   = x_trim(10); q_s = x_trim(11); r_s = x_trim(12);

de_s = u_trim(1); dt_s = u_trim(4);

%% ---- Compute Va, alpha, beta at trim ----
Va_s    = sqrt(u_v^2 + v_v^2 + w_v^2);
alpha_s = atan2(w_v, u_v);
beta_s  = asin(v_v / Va_s);

%% ---- Common terms ----
rho   = P.rho;
qbar  = 0.5*rho*Va_s^2;
c2Va  = P.cbar / (2*Va_s);
b2Va  = P.bw   / (2*Va_s);

% CL, CD at trim
sigma   = (1+exp(-P.M*(alpha_s-P.alpha0))+exp(P.M*(alpha_s+P.alpha0))) / ...
          ((1+exp(-P.M*(alpha_s-P.alpha0)))*(1+exp(P.M*(alpha_s+P.alpha0))));
CL_a    = (1-sigma)*(P.CLo + P.CL_a*alpha_s) + sigma*(2*sign(alpha_s)*sin(alpha_s)^2*cos(alpha_s));
AR      = P.bw^2/P.Sw;
CD_a    = P.CDp + (P.CLo + P.CL_a*alpha_s)^2/(pi*P.e*AR);

% Gamma constants
G  = P.Jx*P.Jz - P.Jxz^2;
G1 = P.Jxz*(P.Jx-P.Jy+P.Jz)/G;
G3 = P.Jz/G;  G4 = P.Jxz/G;
G7 = ((P.Jx-P.Jy)*P.Jx+P.Jxz^2)/G;  G8 = P.Jx/G;

% Combined Cp, Cr coefficients
Cp_da = (G3*P.Cl_da + G4*P.Cn_da) * qbar*P.Sw*P.bw;
Cp_p  = G3*P.Cl_p  + G4*P.Cn_p;

%% ============================================================
%  LATERAL TRANSFER FUNCTIONS
% ============================================================

%-- Roll angle / aileron  eq (5.26) --%
% phi(s) = a_phi2 / (s*(s + a_phi1)) * da(s)
a_phi1 = -qbar*P.Sw*P.bw^2 / (2*Va_s) * (G3*P.Cl_p + G4*P.Cn_p);
a_phi2 = qbar*P.Sw*P.bw * (G3*P.Cl_da + G4*P.Cn_da);

TF.phi_da = tf([a_phi2], [1, a_phi1, 0]);
fprintf('a_phi1=%.4f, a_phi2=%.4f\n', a_phi1, a_phi2);

%-- Course angle / roll angle  eq (5.27) --%
% chi(s) = (g/Va) / s * phi(s)
Vg_s = Va_s;   % assume no wind
TF.chi_phi = tf([P.g/Vg_s], [1, 0]);

%-- Sideslip / rudder  eq (5.28) --%
a_beta1 = -rho*Va_s*P.Sw / (2*P.mass) * P.CY_beta;
a_beta2 =  rho*Va_s*P.Sw / (2*P.mass) * P.CY_dr;

TF.beta_dr = tf([a_beta2], [1, a_beta1]);
fprintf('a_beta1=%.4f, a_beta2=%.4f\n', a_beta1, a_beta2);

%% ============================================================
%  LONGITUDINAL TRANSFER FUNCTIONS
% ============================================================

%-- Pitch angle / elevator  eq (5.29) --%
% theta(s) = a_theta3 / (s^2 + a_theta1*s + a_theta2) * de(s)
a_theta1 = -qbar*P.Sw*P.cbar^2 / (2*P.Jy*Va_s) * P.Cm_q;
a_theta2 = -qbar*P.Sw*P.cbar   / P.Jy * P.Cm_a;
a_theta3 =  qbar*P.Sw*P.cbar   / P.Jy * P.Cm_de;

TF.theta_de = tf([a_theta3], [1, a_theta1, a_theta2]);
fprintf('a_theta1=%.4f, a_theta2=%.4f, a_theta3=%.4f\n', a_theta1,a_theta2,a_theta3);

%-- Altitude / pitch angle  eq (5.31) --%
% h(s) = Va_s/s * theta(s)
TF.h_theta = tf([Va_s], [1, 0]);

%-- Altitude / airspeed  eq (5.32) --%
% h(s) = theta_trim/s * Va(s)
theta_trim_val = x_trim(8);
TF.h_Va = tf([theta_trim_val], [1, 0]);

%-- Airspeed / throttle and pitch  eq (5.36) --%
% Va(s) = 1/(s + a_V1) * (a_V2*dt - a_V3*theta)
a_V1 = rho*Va_s*P.Sw/P.mass * (CD_a + P.CD_de*de_s) ...
       + rho*P.Sprop*P.Cprop*Va_s/P.mass;
a_V2 = rho*P.Sprop*P.Cprop*P.kmotor^2*dt_s / P.mass;
a_V3 = P.g * cos(theta_s - alpha_s);

TF.Va_dt    = tf([a_V2], [1, a_V1]);
TF.Va_theta = tf([-a_V3], [1, a_V1]);
fprintf('a_V1=%.4f, a_V2=%.4f, a_V3=%.4f\n', a_V1, a_V2, a_V3);

%% ---- Print summary ----
fprintf('\n=== Section 5.4 Transfer Functions ===\n');
fprintf('\nLATERAL:\n');
fprintf('phi/da   (roll/aileron):   '); disp(TF.phi_da);
fprintf('chi/phi  (course/roll):    '); disp(TF.chi_phi);
fprintf('beta/dr  (sideslip/rudder):'); disp(TF.beta_dr);
fprintf('\nLONGITUDINAL:\n');
fprintf('theta/de (pitch/elevator): '); disp(TF.theta_de);
fprintf('h/theta  (alt/pitch):      '); disp(TF.h_theta);
fprintf('Va/dt    (speed/throttle): '); disp(TF.Va_dt);
fprintf('Va/theta (speed/pitch):    '); disp(TF.Va_theta);
end
