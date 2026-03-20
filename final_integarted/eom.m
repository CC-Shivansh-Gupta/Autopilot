function xdot = eom(x, delta, wind, P)
% =========================================================
% eom.m  —  6-DOF equations of motion for F-4 Phantom
% Implements eq (3.14)-(3.17) + forces_moments  (Beard & McLain)
%
% Called by ode45 in all Section scripts.
% Use chap3_eom.m (Level-2 S-Function) for Simulink.
%
% INPUTS:
%   x      - 12x1 state  [pn,pe,pd,u,v,w,phi,theta,psi,p,q,r]
%   delta  - 4x1  inputs [de,da,dr,dt]
%   wind   - 6x1  wind   [wn,we,wd,ug,vg,wg]
%   P      - param struct from F4_params.m
%
% OUTPUT:
%   xdot   - 12x1 state derivative
% =========================================================

%% Compute forces and moments
fm = forces_moments(x, delta, wind, P);
fx=fm(1); fy=fm(2); fz=fm(3);
l =fm(4); m =fm(5); n =fm(6);

%% Unpack states
u=x(4); v=x(5); w=x(6);
phi=x(7); theta=x(8); psi=x(9);
p=x(10); q=x(11); r=x(12);

%% Trig shortcuts
cp=cos(phi); sp=sin(phi);
ct=cos(theta); st=sin(theta); tt=tan(theta);
cs=cos(psi); ss=sin(psi);

%% Inertia constants  (eq 3.13)
G  = P.Jx*P.Jz - P.Jxz^2;
G1 = P.Jxz*(P.Jx-P.Jy+P.Jz)/G;
G2 = (P.Jz*(P.Jz-P.Jy)+P.Jxz^2)/G;
G3 = P.Jz/G;   G4=P.Jxz/G;
G5 = (P.Jz-P.Jx)/P.Jy;
G6 = P.Jxz/P.Jy;
G7 = ((P.Jx-P.Jy)*P.Jx+P.Jxz^2)/G;
G8 = P.Jx/G;

%% Position kinematics  (eq 3.14)
pnd = (ct*cs)*u + (sp*st*cs-cp*ss)*v + (cp*st*cs+sp*ss)*w;
ped = (ct*ss)*u + (sp*st*ss+cp*cs)*v + (cp*st*ss-sp*cs)*w;
pdd = (-st)*u  + (sp*ct)*v           + (cp*ct)*w;

%% Translational dynamics  (eq 3.15)
ud = r*v - q*w + fx/P.mass;
vd = p*w - r*u + fy/P.mass;
wd = q*u - p*v + fz/P.mass;

%% Attitude kinematics  (eq 3.16)
phd = p + (sp*tt)*q + (cp*tt)*r;
thd = cp*q - sp*r;
psd = (sp/ct)*q + (cp/ct)*r;

%% Rotational dynamics  (eq 3.17)
pd_ = G1*p*q - G2*q*r + G3*l + G4*n;
qd_ = G5*p*r - G6*(p^2-r^2) + m/P.Jy;
rd_ = G7*p*q - G1*q*r + G4*l + G8*n;

xdot = [pnd; ped; pdd; ud; vd; wd; phd; thd; psd; pd_; qd_; rd_];
end
