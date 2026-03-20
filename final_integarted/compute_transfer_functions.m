function TF = compute_transfer_functions(x_trim, u_trim, P)
% =========================================================
% compute_transfer_functions.m
% Section 5.4 transfer functions — Beard & McLain
%
% OUTPUTS (struct TF):
%   .phi_da    phi/da     eq (5.26)
%   .chi_phi   chi/phi    eq (5.27)
%   .beta_dr   beta/dr    eq (5.28)
%   .theta_de  theta/de   eq (5.29)
%   .h_theta   h/theta    eq (5.31)
%   .h_Va      h/Va       eq (5.32)
%   .Va_dt     Va/dt      eq (5.36)
%   .Va_theta  Va/theta   eq (5.36)
% =========================================================

u_v=x_trim(4); v_v=x_trim(5); w_v=x_trim(6);
theta_s=x_trim(8); p_s=x_trim(10); q_s=x_trim(11); r_s=x_trim(12);
de_s=u_trim(1); dt_s=u_trim(4);

Va_s    = sqrt(u_v^2+v_v^2+w_v^2);
alpha_s = atan2(w_v,u_v);

rho=P.rho; qbar=0.5*rho*Va_s^2;
c2Va=P.cbar/(2*Va_s); b2Va=P.bw/(2*Va_s);

% CL, CD at trim
sigma=(1+exp(-P.M*(alpha_s-P.alpha0))+exp(P.M*(alpha_s+P.alpha0)))/ ...
      ((1+exp(-P.M*(alpha_s-P.alpha0)))*(1+exp(P.M*(alpha_s+P.alpha0))));
CL_s=(1-sigma)*(P.CLo+P.CL_a*alpha_s)+sigma*(2*sign(alpha_s)*sin(alpha_s)^2*cos(alpha_s));
AR=P.bw^2/P.Sw;
CD_s=P.CDp+(P.CLo+P.CL_a*alpha_s)^2/(pi*P.e*AR);

% Gamma constants
G=P.Jx*P.Jz-P.Jxz^2; G3=P.Jz/G; G4=P.Jxz/G;

%% ---- LATERAL ----

% a_phi coefficients  (eq 5.23-5.24)
a_phi1 = -qbar*P.Sw*P.bw^2/(2*Va_s)*(G3*P.Cl_p+G4*P.Cn_p);
a_phi2 =  qbar*P.Sw*P.bw*(G3*P.Cl_da+G4*P.Cn_da);

% phi/da  (eq 5.26)
TF.phi_da = tf([a_phi2],[1, a_phi1, 0]);

% chi/phi  (eq 5.27) — zero wind => Vg = Va
TF.chi_phi = tf([P.g/Va_s],[1, 0]);

% beta/dr  (eq 5.28)
a_beta1 = -rho*Va_s*P.Sw/(2*P.mass)*P.CY_beta;
a_beta2 =  rho*Va_s*P.Sw/(2*P.mass)*P.CY_dr;
TF.beta_dr = tf([a_beta2],[1, a_beta1]);

%% ---- LONGITUDINAL ----

% a_theta coefficients  (eq 5.29)
a_theta1 = -qbar*P.Sw*P.cbar^2/(2*P.Jy*Va_s)*P.Cm_q;
a_theta2 = -qbar*P.Sw*P.cbar/P.Jy*P.Cm_a;
a_theta3 =  qbar*P.Sw*P.cbar/P.Jy*P.Cm_de;

% theta/de  (eq 5.29)
TF.theta_de = tf([a_theta3],[1, a_theta1, a_theta2]);

% h/theta  (eq 5.31)
TF.h_theta = tf([Va_s],[1, 0]);

% h/Va  (eq 5.32)
TF.h_Va = tf([x_trim(8)],[1, 0]);   % theta_trim

% Va/(dt and theta)  (eq 5.36) — linearised about trim
% dCD/dalpha for Xu
CD_a_lin = 2*(P.CLo+P.CL_a*alpha_s)*P.CL_a/(pi*P.e*AR);

a_V1 = rho*Va_s*P.Sw/P.mass*(CD_s+CD_a_lin*alpha_s+P.CD_de*de_s) ...
       + rho*P.Sprop*P.Cprop*Va_s/P.mass;
a_V2 = rho*P.Sprop*P.Cprop*P.kmotor^2*dt_s/P.mass;
a_V3 = P.g*cos(theta_s-alpha_s);   % note: -a_V3 in Va/theta TF

TF.Va_dt    = tf([a_V2],   [1, a_V1]);
TF.Va_theta = tf([-a_V3],  [1, a_V1]);

%% Print
fprintf('\n  === Transfer Function Coefficients ===\n');
fprintf('  Lateral : a_phi1=%.4f  a_phi2=%.5f\n', a_phi1,a_phi2);
fprintf('  Lateral : a_beta1=%.4f  a_beta2=%.5f\n', a_beta1,a_beta2);
fprintf('  Longit  : a_theta1=%.4f  a_theta2=%.4f  a_theta3=%.5f\n', a_theta1,a_theta2,a_theta3);
fprintf('  Longit  : a_V1=%.5f  a_V2=%.5f  a_V3=%.4f\n\n', a_V1,a_V2,a_V3);
end
