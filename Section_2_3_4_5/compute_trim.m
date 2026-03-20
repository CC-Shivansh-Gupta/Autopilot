function [x_trim, u_trim] = compute_trim(Va_star, gamma_star, R_star, P)
% =========================================================
% compute_trim.m  (v3 - fminsearch, robust)
% Computes trim states and inputs for fixed-wing MAV.
% Uses MATLAB fminsearch (Nelder-Mead) — much more robust
% than hand-rolled gradient descent for large-scale aircraft.
%
% INPUTS:
%   Va_star    - desired airspeed [ft/s]
%   gamma_star - desired flight path angle [rad]
%   R_star     - turn radius [ft]  (Inf = wings-level)
%   P          - aircraft parameter struct
%
% OUTPUTS:
%   x_trim (12x1), u_trim (4x1) = [de, da, dr, dt]
% =========================================================

fprintf('Computing trim: Va=%.1f ft/s, gamma=%.2f deg, R=', ...
        Va_star, rad2deg(gamma_star));
if isinf(R_star), fprintf('Inf\n'); else, fprintf('%.0f ft\n', R_star); end

%% ---- Analytical initial guess ----
qbar = 0.5 * P.rho * Va_star^2;

if isinf(R_star)
    n_load = 1.0;
    phi0   = 0;
else
    phi0   = atan(Va_star^2 / (P.g * abs(R_star)));
    phi0   = sign(R_star) * phi0;
    n_load = 1 / cos(phi0);
end

CL_need = P.mass * P.g * n_load / (qbar * P.Sw);
alpha0  = (CL_need - P.CLo) / P.CL_a;
alpha0  = max(-0.15, min(0.30, alpha0));

xi0 = [alpha0; 0; phi0];   % [alpha, beta, phi]

%% ---- fminsearch ----
opts_nm = optimset('TolX',1e-10,'TolFun',1e-12, ...
                   'MaxFunEvals',5000,'MaxIter',5000,'Display','off');

cost_fn = @(xi) trim_cost(xi(1),xi(2),xi(3),Va_star,gamma_star,R_star,P);

% Multi-start
starts = [xi0, xi0+[0.01;0;0], xi0+[-0.01;0;0], xi0+[0;0.005;0]];
best_J  = inf;
best_xi = xi0;
for s = 1:size(starts,2)
    [xi_s, J_s] = fminsearch(cost_fn, starts(:,s), opts_nm);
    if J_s < best_J
        best_J  = J_s;
        best_xi = xi_s;
    end
    if best_J < 1e-10, break; end
end

fprintf('  Converged: J = %.3e\n', best_J);
if best_J > 1e-4
    warning('compute_trim: J=%.2e — trim may be inaccurate.', best_J);
end

alpha = best_xi(1);
beta  = best_xi(2);
phi   = best_xi(3);

%% ---- Build trim state ----
u_v   = Va_star * cos(alpha) * cos(beta);
v_v   = Va_star * sin(beta);
w_v   = Va_star * sin(alpha) * cos(beta);
theta = alpha + gamma_star;

if isinf(R_star)
    p_v=0; q_v=0; r_v=0;
else
    p_v = -(Va_star/R_star)*sin(theta);
    q_v =  (Va_star/R_star)*sin(phi)*cos(theta);
    r_v =  (Va_star/R_star)*cos(phi)*cos(theta);
end

x_trim = [0;0;0; u_v;v_v;w_v; phi;theta;0; p_v;q_v;r_v];
u_trim = trim_inputs(alpha,beta,phi,Va_star,gamma_star,R_star, ...
                     p_v,q_v,r_v,theta,P);

fprintf('  alpha=%+.3f deg  beta=%+.3f deg  phi=%+.2f deg  theta=%+.3f deg\n', ...
        rad2deg(alpha),rad2deg(beta),rad2deg(phi),rad2deg(theta));
fprintf('  u=%.2f  v=%.3f  w=%.3f ft/s\n', u_v,v_v,w_v);
fprintf('  de=%+.4f rad  da=%+.4f rad  dr=%+.4f rad  dt=%.4f\n', ...
        u_trim(1),u_trim(2),u_trim(3),u_trim(4));
end

%% ============================================================
function J = trim_cost(alpha,beta,phi,Va,gamma,R,P)
    u_v   = Va*cos(alpha)*cos(beta);
    v_v   = Va*sin(beta);
    w_v   = Va*sin(alpha)*cos(beta);
    theta = alpha + gamma;

    if isinf(R)
        p_v=0; q_v=0; r_v=0;
        xdot_star=[0;0;-Va*sin(gamma); 0;0;0; 0;0;0; 0;0;0];
    else
        p_v=-(Va/R)*sin(theta);
        q_v= (Va/R)*sin(phi)*cos(theta);
        r_v= (Va/R)*cos(phi)*cos(theta);
        xdot_star=[0;0;-Va*sin(gamma); 0;0;0; 0;0;Va/R; 0;0;0];
    end

    x = [0;0;0; u_v;v_v;w_v; phi;theta;0; p_v;q_v;r_v];
    u = trim_inputs(alpha,beta,phi,Va,gamma,R,p_v,q_v,r_v,theta,P);

    xdot = eom_rhs(x,u,zeros(6,1),P);
    diff = xdot - xdot_star;
    diff([1,2,9]) = 0;   % ignore pn, pe, psi
    J = diff'*diff;
end

%% ============================================================
function u = trim_inputs(alpha,beta,phi,Va,gamma,R,p,q,r,theta,P)
    qbar = 0.5*P.rho*Va^2;
    c2Va = P.cbar/(2*Va);
    b2Va = P.bw/(2*Va);

    % Nonlinear CL, CD
    sigma = (1+exp(-P.M*(alpha-P.alpha0))+exp(P.M*(alpha+P.alpha0))) / ...
            ((1+exp(-P.M*(alpha-P.alpha0)))*(1+exp(P.M*(alpha+P.alpha0))));
    CL  = (1-sigma)*(P.CLo+P.CL_a*alpha) + ...
           sigma*(2*sign(alpha)*sin(alpha)^2*cos(alpha));
    AR  = P.bw^2/P.Sw;
    CD  = P.CDp + (P.CLo+P.CL_a*alpha)^2/(pi*P.e*AR);
    CX    = -CD*cos(alpha)    + CL*sin(alpha);
    CX_q  = -P.CD_q*cos(alpha)  + P.CL_q*sin(alpha);
    CX_de = -P.CD_de*cos(alpha) + P.CL_de*sin(alpha);

    % Elevator from Cm=0
    G  = P.Jx*P.Jz-P.Jxz^2;
    G1 = P.Jxz*(P.Jx-P.Jy+P.Jz)/G;
    G2 = (P.Jz*(P.Jz-P.Jy)+P.Jxz^2)/G;
    G3 = P.Jz/G;  G4=P.Jxz/G;
    G7 = ((P.Jx-P.Jy)*P.Jx+P.Jxz^2)/G;  G8=P.Jx/G;

    de = (-(P.Jxz*(p^2-r^2)+(P.Jx-P.Jz)*p*r)/(qbar*P.Sw*P.cbar) ...
          - P.Cmo - P.Cm_a*alpha - P.Cm_q*c2Va*q) / P.Cm_de;
    de = max(P.de_min, min(P.de_max, de));

    % Throttle from u_dot=0
    u_v=Va*cos(alpha)*cos(beta);
    v_v=Va*sin(beta);
    w_v=Va*sin(alpha)*cos(beta);

    rhs = 2*P.mass*(-r*v_v+q*w_v+P.g*sin(theta)) ...
          - P.rho*Va^2*P.Sw*(CX+CX_q*c2Va*q+CX_de*de) ...
          + P.rho*P.Sprop*P.Cprop*Va^2;
    dt2 = rhs/(P.rho*P.Sprop*P.Cprop*P.kmotor^2);
    if dt2 <= 0, dt=0; else, dt=sqrt(dt2); end
    dt = max(0,min(1,dt));

    % Aileron & rudder from p_dot=r_dot=0
    Cp_0    = G3*P.Cl0    +G4*P.Cn0;
    Cp_beta = G3*P.Cl_beta+G4*P.Cn_beta;
    Cp_p    = G3*P.Cl_p   +G4*P.Cn_p;
    Cp_r    = G3*P.Cl_r   +G4*P.Cn_r;
    Cp_da   = G3*P.Cl_da  +G4*P.Cn_da;
    Cp_dr   = G3*P.Cl_dr  +G4*P.Cn_dr;
    Cr_0    = G4*P.Cl0    +G8*P.Cn0;
    Cr_beta = G4*P.Cl_beta+G8*P.Cn_beta;
    Cr_p    = G4*P.Cl_p   +G8*P.Cn_p;
    Cr_r    = G4*P.Cl_r   +G8*P.Cn_r;
    Cr_da   = G4*P.Cl_da  +G8*P.Cn_da;
    Cr_dr   = G4*P.Cl_dr  +G8*P.Cn_dr;

    rhs_p = -G1*p*q+G2*q*r - qbar*P.Sw*P.bw*(Cp_0+Cp_beta*beta+Cp_p*b2Va*p+Cp_r*b2Va*r);
    rhs_r = -G7*p*q+G1*q*r - qbar*P.Sw*P.bw*(Cr_0+Cr_beta*beta+Cr_p*b2Va*p+Cr_r*b2Va*r);

    M_ar = qbar*P.Sw*P.bw*[Cp_da,Cp_dr; Cr_da,Cr_dr];
    if abs(det(M_ar)) > 1e-10
        sol = M_ar\[rhs_p;rhs_r];
    else
        sol = [0;0];
    end
    da = max(P.da_min,min(P.da_max,sol(1)));
    dr = max(P.dr_min,min(P.dr_max,sol(2)));

    u = [de;da;dr;dt];
end

%% ============================================================
function xdot = eom_rhs(x,u,wind,P)
    fm=forces_moments(x,u,wind,P);
    fx=fm(1);fy=fm(2);fz=fm(3);
    l=fm(4);mm=fm(5);n=fm(6);

    u_v=x(4);v_v=x(5);w_v=x(6);
    phi=x(7);theta=x(8);psi=x(9);
    p=x(10);q=x(11);r=x(12);

    cp=cos(phi);sp=sin(phi);
    ct=cos(theta);st=sin(theta);tt=tan(theta);
    cs=cos(psi);ss=sin(psi);

    G  = P.Jx*P.Jz-P.Jxz^2;
    G1 = P.Jxz*(P.Jx-P.Jy+P.Jz)/G;
    G2 = (P.Jz*(P.Jz-P.Jy)+P.Jxz^2)/G;
    G3 = P.Jz/G;  G4=P.Jxz/G;
    G5 = (P.Jz-P.Jx)/P.Jy;
    G6 = P.Jxz/P.Jy;
    G7 = ((P.Jx-P.Jy)*P.Jx+P.Jxz^2)/G;
    G8 = P.Jx/G;

    pnd=(ct*cs)*u_v+(sp*st*cs-cp*ss)*v_v+(cp*st*cs+sp*ss)*w_v;
    ped=(ct*ss)*u_v+(sp*st*ss+cp*cs)*v_v+(cp*st*ss-sp*cs)*w_v;
    pdd=(-st)*u_v+(sp*ct)*v_v+(cp*ct)*w_v;

    ud=r*v_v-q*w_v+fx/P.mass;
    vd=p*w_v-r*u_v+fy/P.mass;
    wd=q*u_v-p*v_v+fz/P.mass;

    phd=p+(sp*tt)*q+(cp*tt)*r;
    thd=cp*q-sp*r;
    psd=(sp/ct)*q+(cp/ct)*r;

    pd_=G1*p*q-G2*q*r+G3*l+G4*n;
    qd_=G5*p*r-G6*(p^2-r^2)+mm/P.Jy;
    rd_=G7*p*q-G1*q*r+G4*l+G8*n;

    xdot=[pnd;ped;pdd;ud;vd;wd;phd;thd;psd;pd_;qd_;rd_];
end
