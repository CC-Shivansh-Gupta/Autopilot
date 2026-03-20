function chap3_eom(block)
% =========================================================
% chap3_eom.m  —  Level-2 MATLAB S-Function
% 6-DOF Equations of Motion for F-4 Phantom (Simulink)
% Implements eq (3.14)-(3.17), Beard & McLain Ch. 3
%
% INPUT (6x1):  [fx; fy; fz; l; m; n]  (forces & moments)
% OUTPUT(12x1): [pn,pe,pd,u,v,w,phi,theta,psi,p,q,r]
%
% DIALOG PARAMETERS (17):
%   1: mass [slug]
%   2-5: Jx, Jy, Jz, Jxz [slug.ft^2]
%   6-8:  pn0, pe0, pd0  initial position [ft]
%   9-11: u0, v0, w0     initial velocity [ft/s]
%  12-14: phi0, theta0, psi0  initial Euler angles [rad]
%  15-17: p0, q0, r0     initial angular rates [rad/s]
% =========================================================

setup(block);

%% ---- Setup ----
function setup(block)
    block.NumInputPorts  = 1;
    block.NumOutputPorts = 1;
    block.InputPort(1).Dimensions        = 6;
    block.InputPort(1).DirectFeedthrough = false;
    block.OutputPort(1).Dimensions = 12;
    block.NumContStates  = 12;
    block.NumDialogPrms  = 17;
    block.SampleTimes    = [0, 0];
    block.RegBlockMethod('InitializeConditions', @mdlInitCond);
    block.RegBlockMethod('Outputs',              @mdlOutputs);
    block.RegBlockMethod('Derivatives',          @mdlDerivatives);
    block.RegBlockMethod('Terminate',            @mdlTerminate);
end

%% ---- Initial Conditions ----
function mdlInitCond(block)
    block.ContStates.Data = [ ...
        block.DialogPrm(6).Data;   % pn0
        block.DialogPrm(7).Data;   % pe0
        block.DialogPrm(8).Data;   % pd0
        block.DialogPrm(9).Data;   % u0
        block.DialogPrm(10).Data;  % v0
        block.DialogPrm(11).Data;  % w0
        block.DialogPrm(12).Data;  % phi0
        block.DialogPrm(13).Data;  % theta0
        block.DialogPrm(14).Data;  % psi0
        block.DialogPrm(15).Data;  % p0
        block.DialogPrm(16).Data;  % q0
        block.DialogPrm(17).Data]; % r0
end

%% ---- Outputs ----
function mdlOutputs(block)
    block.OutputPort(1).Data = block.ContStates.Data;
end

%% ---- Derivatives ----
function mdlDerivatives(block)
    mass= block.DialogPrm(1).Data;
    Jx  = block.DialogPrm(2).Data;
    Jy  = block.DialogPrm(3).Data;
    Jz  = block.DialogPrm(4).Data;
    Jxz = block.DialogPrm(5).Data;

    x   = block.ContStates.Data;
    u_v=x(4); v_v=x(5); w_v=x(6);
    phi=x(7); theta=x(8); psi=x(9);
    p=x(10); q=x(11); r=x(12);

    uu   = block.InputPort(1).Data;
    fx=uu(1); fy=uu(2); fz=uu(3);
    l=uu(4); m_m=uu(5); n=uu(6);

    cp=cos(phi); sp=sin(phi);
    ct=cos(theta); st=sin(theta); tt=tan(theta);
    cpsi=cos(psi); spsi=sin(psi);

    G  = Jx*Jz-Jxz^2;
    G1 = Jxz*(Jx-Jy+Jz)/G;
    G2 = (Jz*(Jz-Jy)+Jxz^2)/G;
    G3 = Jz/G;  G4=Jxz/G;
    G5 = (Jz-Jx)/Jy;
    G6 = Jxz/Jy;
    G7 = ((Jx-Jy)*Jx+Jxz^2)/G;
    G8 = Jx/G;

    % eq (3.14)
    pnd=(ct*cpsi)*u_v+(sp*st*cpsi-cp*spsi)*v_v+(cp*st*cpsi+sp*spsi)*w_v;
    ped=(ct*spsi)*u_v+(sp*st*spsi+cp*cpsi)*v_v+(cp*st*spsi-sp*cpsi)*w_v;
    pdd=(-st)*u_v+(sp*ct)*v_v+(cp*ct)*w_v;

    % eq (3.15)
    ud=r*v_v-q*w_v+fx/mass;
    vd=p*w_v-r*u_v+fy/mass;
    wd=q*u_v-p*v_v+fz/mass;

    % eq (3.16)
    phd=p+(sp*tt)*q+(cp*tt)*r;
    thd=cp*q-sp*r;
    psd=(sp/ct)*q+(cp/ct)*r;

    % eq (3.17)
    pd_=G1*p*q-G2*q*r+G3*l+G4*n;
    qd_=G5*p*r-G6*(p^2-r^2)+m_m/Jy;
    rd_=G7*p*q-G1*q*r+G4*l+G8*n;

    block.Derivatives.Data = ...
        [pnd;ped;pdd; ud;vd;wd; phd;thd;psd; pd_;qd_;rd_];
end

%% ---- Terminate ----
function mdlTerminate(block) %#ok<INUSD>
end

end
