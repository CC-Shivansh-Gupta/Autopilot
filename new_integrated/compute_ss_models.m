function [SS_lon, SS_lat] = compute_ss_models(x_trim, u_trim, P)
% =========================================================
% compute_ss_models.m
% Lateral (eq 5.43) and longitudinal (eq 5.50) state-space models.
% Analytical Jacobians from Tables 5.1 and 5.2, Beard & McLain.
%
% OUTPUTS:
%   SS_lon  states: [u, w, q, theta, h]
%   SS_lat  states: [v, p, r, phi, psi]
% =========================================================

u_s=x_trim(4); v_s=x_trim(5); w_s=x_trim(6);
phi_s=x_trim(7); theta_s=x_trim(8);
p_s=x_trim(10); q_s=x_trim(11); r_s=x_trim(12);
de_s=u_trim(1); dt_s=u_trim(4);

Va_s    = sqrt(u_s^2+v_s^2+w_s^2);
alpha_s = atan2(w_s,u_s);
beta_s  = asin(max(-1,min(1,v_s/Va_s)));

rho=P.rho; qbar=0.5*rho*Va_s^2;
Sw=P.Sw; bw=P.bw; cbar=P.cbar;

%% Gamma constants
G =P.Jx*P.Jz-P.Jxz^2;
G1=P.Jxz*(P.Jx-P.Jy+P.Jz)/G;
G2=(P.Jz*(P.Jz-P.Jy)+P.Jxz^2)/G;
G3=P.Jz/G; G4=P.Jxz/G;
G7=((P.Jx-P.Jy)*P.Jx+P.Jxz^2)/G; G8=P.Jx/G;

%% Aero at trim
sigma=(1+exp(-P.M*(alpha_s-P.alpha0))+exp(P.M*(alpha_s+P.alpha0)))/ ...
      ((1+exp(-P.M*(alpha_s-P.alpha0)))*(1+exp(P.M*(alpha_s+P.alpha0))));
CL_s=(1-sigma)*(P.CLo+P.CL_a*alpha_s)+sigma*(2*sign(alpha_s)*sin(alpha_s)^2*cos(alpha_s));
AR=bw^2/Sw;
CD_s=P.CDp+(P.CLo+P.CL_a*alpha_s)^2/(pi*P.e*AR);
CD_a=2*(P.CLo+P.CL_a*alpha_s)*P.CL_a/(pi*P.e*AR);   % dCD/dalpha

CX_s =-CD_s*cos(alpha_s)+CL_s*sin(alpha_s);
CZ_s =-CD_s*sin(alpha_s)-CL_s*cos(alpha_s);
CX_q =-P.CD_q*cos(alpha_s)+P.CL_q*sin(alpha_s);
CZ_q =-P.CD_q*sin(alpha_s)-P.CL_q*cos(alpha_s);
CX_de=-P.CD_de*cos(alpha_s)+P.CL_de*sin(alpha_s);
CZ_de=-P.CD_de*sin(alpha_s)-P.CL_de*cos(alpha_s);

%% ============================================================
%  LONGITUDINAL  (eq 5.50)  states: [u, w, q, theta, h]
% ============================================================

Xu = rho*u_s*Sw/P.mass*(CX_s-(CD_a*(-w_s/Va_s^2))*Va_s) ...
     + rho*Sw*u_s/(2*P.mass*Va_s)*CX_q*cbar*q_s ...
     - rho*P.Sprop*P.Cprop*u_s/P.mass;
Xu = -rho*Va_s*Sw/P.mass*(2*CD_s+CD_a*alpha_s) - rho*P.Sprop*P.Cprop*Va_s/P.mass;

Xw =  rho*Va_s*Sw/(2*P.mass)*(CL_s - CD_a);

Zu = -rho*Va_s*Sw/P.mass*(2*CL_s);       % Table 5.2 leading term
Zw = -rho*Va_s*Sw/(2*P.mass)*(P.CL_a+CD_s);

Xq =  rho*Va_s*Sw*cbar/(4*P.mass)*CX_q;
Zq =  rho*Va_s*Sw*cbar/(4*P.mass)*CZ_q;

Mu = 0;
Mw =  rho*Va_s*Sw*cbar/(2*P.Jy)*P.Cm_a;
Mq =  rho*Va_s*Sw*cbar^2/(4*P.Jy)*P.Cm_q;

X_de = qbar*Sw/P.mass*CX_de;
Z_de = qbar*Sw/P.mass*CZ_de;
M_de = qbar*Sw*cbar/P.Jy*P.Cm_de;
X_dt = rho*P.Sprop*P.Cprop*P.kmotor^2*dt_s/P.mass;

A_lon = [Xu,           Xw,          Xq,          -P.g*cos(theta_s), 0;
         Zu,           Zw,          Va_s+Zq,     -P.g*sin(theta_s), 0;
         Mu,           Mw,          Mq,           0,                0;
         0,            0,           1,            0,                0;
         sin(theta_s), -cos(theta_s), 0, Va_s*cos(theta_s)+w_s*sin(theta_s), 0];

B_lon = [X_de, X_dt;
         Z_de, 0;
         M_de, 0;
         0,    0;
         0,    0];

SS_lon = ss(A_lon, B_lon, eye(5), zeros(5,2));
SS_lon.StateName  = {'u','w','q','theta','h'};
SS_lon.InputName  = {'delta_e','delta_t'};
SS_lon.OutputName = {'u','w','q','theta','h'};

%% ============================================================
%  LATERAL  (eq 5.43)  states: [v, p, r, phi, psi]
% ============================================================

Yv =  rho*Va_s*Sw/(2*P.mass)*P.CY_beta;
Yp =  w_s + rho*Va_s*Sw*bw/(4*P.mass)*P.CY_p;
Yr = -u_s + rho*Va_s*Sw*bw/(4*P.mass)*P.CY_r;

Lv =  rho*Va_s*Sw*bw/2*(G3*P.Cl_beta+G4*P.Cn_beta);
Lp =  rho*Va_s*Sw*bw^2/4*(G3*P.Cl_p+G4*P.Cn_p);
Lr =  rho*Va_s*Sw*bw^2/4*(G3*P.Cl_r+G4*P.Cn_r);

Nv =  rho*Va_s*Sw*bw/2*(G4*P.Cl_beta+G8*P.Cn_beta);
Np =  rho*Va_s*Sw*bw^2/4*(G4*P.Cl_p+G8*P.Cn_p);
Nr =  rho*Va_s*Sw*bw^2/4*(G4*P.Cl_r+G8*P.Cn_r);

Yda = qbar*Sw/P.mass*P.CY_da;
Ydr = qbar*Sw/P.mass*P.CY_dr;
Lda = qbar*Sw*bw*(G3*P.Cl_da+G4*P.Cn_da);
Ldr = qbar*Sw*bw*(G3*P.Cl_dr+G4*P.Cn_dr);
Nda = qbar*Sw*bw*(G4*P.Cl_da+G8*P.Cn_da);
Ndr = qbar*Sw*bw*(G4*P.Cl_dr+G8*P.Cn_dr);

A_lat = [Yv,  Yp,  (Yr-Va_s),  P.g*cos(theta_s)*cos(phi_s), 0;
         Lv,  Lp,   Lr,         0,                            0;
         Nv,  Np,   Nr,         0,                            0;
         0,   1,    tan(theta_s), 0,                          0;
         0,   0,    1/cos(theta_s), 0,                        0];

B_lat = [Yda,Ydr; Lda,Ldr; Nda,Ndr; 0,0; 0,0];

SS_lat = ss(A_lat, B_lat, eye(5), zeros(5,2));
SS_lat.StateName  = {'v','p','r','phi','psi'};
SS_lat.InputName  = {'delta_a','delta_r'};
SS_lat.OutputName = {'v','p','r','phi','psi'};

%% Print eigenvalues
fprintf('\n  === Longitudinal State-Space Eigenvalues ===\n');
lons=eig(A_lon);
for lam=lons.'
    if abs(imag(lam))>1e-4
        wn=abs(lam); z=-real(lam)/wn;
        stab=ternary(real(lam)<=0,'STABLE','UNSTABLE');
        fprintf('    %+.4f%+.4fi  wn=%.3f rad/s  zeta=%.3f  [%s]\n', ...
                real(lam),imag(lam),wn,z,stab);
    else
        stab=ternary(real(lam)<=1e-4,'STABLE','UNSTABLE');
        fprintf('    %+.6f  [%s]\n',real(lam),stab);
    end
end

fprintf('\n  === Lateral State-Space Eigenvalues ===\n');
lats=eig(A_lat);
for lam=lats.'
    if abs(imag(lam))>1e-4
        stab=ternary(real(lam)<=0,'STABLE','UNSTABLE');
        fprintf('    %+.4f%+.4fi  Dutch-roll  [%s]\n',real(lam),imag(lam),stab);
    elseif real(lam)>1e-4
        fprintf('    %+.4f  Spiral mode (unstable)\n',real(lam));
    elseif abs(real(lam))<1e-4
        fprintf('    %+.6f  Heading integrator\n',real(lam));
    else
        fprintf('    %+.4f  Roll subsidence (stable)\n',real(lam));
    end
end
fprintf('\n');
end

%% Local helper
function s=ternary(cond,a,b); if cond; s=a; else; s=b; end; end
