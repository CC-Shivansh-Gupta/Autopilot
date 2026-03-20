function chap3_eom(block)
% =========================================================
% chap3_eom.m  -  Level-2 MATLAB S-Function
% 6-DOF Equations of Motion for the F-4 Phantom
% AE 700 - IIT Bombay | Section 2: Kinematics and Dynamics
%
% Implements equations (3.14) through (3.17) from
% Beard & McLain, "Small Unmanned Aircraft", Ch. 3
%
% ----------------------------------------------------------
% INPUT PORT (6x1):
%   u(1) = fx   [lb]      Force along body x-axis
%   u(2) = fy   [lb]      Force along body y-axis
%   u(3) = fz   [lb]      Force along body z-axis
%   u(4) = l    [lb.ft]   Moment about body x-axis (roll)
%   u(5) = m    [lb.ft]   Moment about body y-axis (pitch)
%   u(6) = n    [lb.ft]   Moment about body z-axis (yaw)
%
% OUTPUT PORT (12x1):
%   [pn, pe, pd, u, v, w, phi, theta, psi, p, q, r]'
%
% STATES (12 continuous):
%   x(1)  = pn     North position      [ft]
%   x(2)  = pe     East  position      [ft]
%   x(3)  = pd     Down  position      [ft]
%   x(4)  = u      Body-x velocity     [ft/s]
%   x(5)  = v      Body-y velocity     [ft/s]
%   x(6)  = w      Body-z velocity     [ft/s]
%   x(7)  = phi    Roll  angle         [rad]
%   x(8)  = theta  Pitch angle         [rad]
%   x(9)  = psi    Yaw   angle         [rad]
%   x(10) = p      Roll  rate          [rad/s]
%   x(11) = q      Pitch rate          [rad/s]
%   x(12) = r      Yaw   rate          [rad/s]
%
% DIALOG PARAMETERS (13 total):
%   P(1)  = mass   [slug]
%   P(2)  = Jx     [slug.ft^2]
%   P(3)  = Jy     [slug.ft^2]
%   P(4)  = Jz     [slug.ft^2]
%   P(5)  = Jxz    [slug.ft^2]
%   P(6)  = pn0    initial North   [ft]
%   P(7)  = pe0    initial East    [ft]
%   P(8)  = pd0    initial Down    [ft]
%   P(9)  = u0     initial u       [ft/s]
%   P(10) = v0     initial v       [ft/s]
%   P(11) = w0     initial w       [ft/s]
%   P(12) = phi0   initial phi     [rad]
%   P(13) = theta0 initial theta   [rad]
%   P(14) = psi0   initial psi     [rad]
%   P(15) = p0     initial p       [rad/s]
%   P(16) = q0     initial q       [rad/s]
%   P(17) = r0     initial r       [rad/s]
% =========================================================

setup(block);

%% ============================================================
function setup(block)
    % --- Ports ---
    block.NumInputPorts  = 1;   % [fx; fy; fz; l; m_mom; n]
    block.NumOutputPorts = 1;   % full 12-state vector

    block.InputPort(1).Dimensions        = 6;
    block.InputPort(1).DirectFeedthrough = false;  % states only, no feedthrough

    block.OutputPort(1).Dimensions = 12;

    % --- Continuous states (12) ---
    block.NumContStates  = 12;

    % --- Dialog parameters (17) ---
    block.NumDialogPrms  = 17;

    % --- Sample time: continuous ---
    block.SampleTimes = [0, 0];

    % --- Register methods ---
    block.RegBlockMethod('InitializeConditions', @mdlInitCond);
    block.RegBlockMethod('Outputs',              @mdlOutputs);
    block.RegBlockMethod('Derivatives',          @mdlDerivatives);
    block.RegBlockMethod('Terminate',            @mdlTerminate);
end

%% ============================================================
function mdlInitCond(block)
    % Set initial conditions from dialog parameters P(6..17)
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
        block.DialogPrm(17).Data;  % r0
    ];
end

%% ============================================================
function mdlOutputs(block)
    % Output the full 12-state vector
    block.OutputPort(1).Data = block.ContStates.Data;
end

%% ============================================================
function mdlDerivatives(block)
    % ---- Unpack parameters ----
    mass = block.DialogPrm(1).Data;
    Jx   = block.DialogPrm(2).Data;
    Jy   = block.DialogPrm(3).Data;
    Jz   = block.DialogPrm(4).Data;
    Jxz  = block.DialogPrm(5).Data;

    % ---- Unpack states ----
    x     = block.ContStates.Data;
    % pn  = x(1);
    % pe  = x(2);
    % pd  = x(3);
    u_vel = x(4);
    v_vel = x(5);
    w_vel = x(6);
    phi   = x(7);
    theta = x(8);
    psi   = x(9);
    p     = x(10);
    q     = x(11);
    r     = x(12);

    % ---- Unpack forces & moments ----
    uu   = block.InputPort(1).Data;
    fx   = uu(1);
    fy   = uu(2);
    fz   = uu(3);
    l    = uu(4);
    m_mom= uu(5);
    n    = uu(6);

    % ---- Trig shorthands ----
    cphi = cos(phi);   sphi = sin(phi);
    cth  = cos(theta); sth  = sin(theta); tth = tan(theta);
    cpsi = cos(psi);   spsi = sin(psi);

    % ---- Gamma constants (eq 3.13) ----
    Gamma  = Jx*Jz - Jxz^2;
    Gamma1 = Jxz*(Jx - Jy + Jz) / Gamma;
    Gamma2 = (Jz*(Jz - Jy) + Jxz^2) / Gamma;
    Gamma3 = Jz / Gamma;
    Gamma4 = Jxz / Gamma;
    Gamma5 = (Jz - Jx) / Jy;
    Gamma6 = Jxz / Jy;
    Gamma7 = ((Jx - Jy)*Jx + Jxz^2) / Gamma;
    Gamma8 = Jx / Gamma;

    % ===========================================================
    % EQUATIONS OF MOTION  (Beard & McLain eq 3.14 - 3.17)
    % ===========================================================

    % --- eq (3.14): Position kinematics (body vel -> NED pos rates) ---
    % [pn_dot; pe_dot; pd_dot] = R_b2ned * [u; v; w]
    pn_dot = (cth*cpsi)*u_vel + (sphi*sth*cpsi - cphi*spsi)*v_vel + (cphi*sth*cpsi + sphi*spsi)*w_vel;
    pe_dot = (cth*spsi)*u_vel + (sphi*sth*spsi + cphi*cpsi)*v_vel + (cphi*sth*spsi - sphi*cpsi)*w_vel;
    pd_dot = (-sth)*u_vel     + (sphi*cth)*v_vel                  + (cphi*cth)*w_vel;

    % --- eq (3.15): Translational dynamics (Newton F=ma in body frame) ---
    u_dot = r*v_vel - q*w_vel + (1/mass)*fx;
    v_dot = p*w_vel - r*u_vel + (1/mass)*fy;
    w_dot = q*u_vel - p*v_vel + (1/mass)*fz;

    % --- eq (3.16): Attitude kinematics (body rates -> Euler angle rates) ---
    phi_dot = p + (sphi*tth)*q + (cphi*tth)*r;
    the_dot = cphi*q            - sphi*r;
    psi_dot = (sphi/cth)*q     + (cphi/cth)*r;

    % --- eq (3.17): Rotational dynamics (Euler equations with Jxz) ---
    p_dot = Gamma1*p*q - Gamma2*q*r + Gamma3*l + Gamma4*n;
    q_dot = Gamma5*p*r - Gamma6*(p^2 - r^2) + m_mom/Jy;
    r_dot = Gamma7*p*q - Gamma1*q*r + Gamma4*l + Gamma8*n;

    % ---- Pack derivatives ----
    block.Derivatives.Data = [ ...
        pn_dot; pe_dot; pd_dot; ...
        u_dot;  v_dot;  w_dot;  ...
        phi_dot; the_dot; psi_dot; ...
        p_dot;  q_dot;  r_dot];
end

%% ============================================================
function mdlTerminate(block) %#ok<INUSD>
end

end
