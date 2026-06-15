function xhat = estimate_states(y, delta, P)
% =========================================================
% estimate_states.m
% Sequential state estimation pipeline.
% Beard & McLain, "Small Unmanned Aircraft," Chapter 8.
%
% INPUTS:
%   y     - sensor struct from sensors.m (Ch 7)
%   delta - 4x1 control inputs [de,da,dr,dt]
%   P     - param struct from F4_params.m
%
% OUTPUT:
%   xhat  - struct of estimated states matching true state x
%
% Pipeline (called every P.Ts seconds):
%   1. Low-pass filter  -> Va_hat, alpha_hat, beta_hat
%   2. Attitude EKF     -> phi_hat, theta_hat          (Sec 8.9)
%   3. GPS EKF          -> pn,pe,Vg,chi,wn,we,psi_hat (Sec 8.10)
%
% Units: ft, slug, lb, rad throughout.
% =========================================================

%% Persistent filter states
persistent Va_hat alpha_hat beta_hat   % LP filter outputs

% Attitude EKF  (state: [phi; theta])
persistent x_att P_att                 % Eq 8.19

% GPS EKF  (state: [pn; pe; Vg; chi; wn; we; psi])
persistent x_gps P_gps                 % Eq 8.34
persistent gps_old                     % last GPS reading (to detect update)

%% Initialise on first call
if isempty(Va_hat)
    Va_hat    = P.Va_trim;
    alpha_hat = P.alpha_trim;
    beta_hat  = 0;

    x_att = zeros(2,1);               % [phi; theta]
    P_att = eye(2) * 0.1;

    x_gps = zeros(7,1);               % [pn;pe;Vg;chi;wn;we;psi]
    P_gps = diag([1,1,1,1,1,1,1]);
    gps_old = [0;0;0;0;0];
end

Ts = P.Ts;

%% ============================================================
%  BLOCK 1 — Low-Pass Filter for Va, alpha, beta  (Sec 8.4)
% ============================================================
% Va from dynamic pressure: Va = sqrt(2*q_dyn / rho)   (Eq 7.10 inverted)
% TODO: implement first-order LP  xhat_k = (1-a)*xhat_{k-1} + a*meas
%   where a = Ts / (tau + Ts)
tau_LP  = 0.5;                        % LP time constant [s]  (tune)
a_LP    = Ts / (tau_LP + Ts);

Va_meas    = sqrt(max(0, 2*y.dynamic_pressure / P.rho));  % Eq 7.10 inverted
Va_hat     = (1 - a_LP)*Va_hat + a_LP*Va_meas;    % TODO Eq LP

% alpha, beta from Va and body accelerations (Sec 8.4)
% TODO: derive alpha_hat from accel measurements (Eq 8.x)
alpha_hat  = alpha_hat;   % TODO LP filter
beta_hat   = beta_hat;    % TODO LP filter

%% ============================================================
%  BLOCK 2 — Attitude EKF  (Sec 8.9, Eqs 8.19-8.29)
% ============================================================
% State: x_att = [phi; theta]
% Input: gyro measurements as p,q,r
% Propagation (Eq 8.19):
%   A = df/dx evaluated at x_att, u=[p,q,r]'
%   x_att = x_att + Ts * f(x_att, u)
% Measurement update with accelerometers (Eq 8.25):
%   h(x) = specific force model
%   K = P*H' * (H*P*H' + R)^{-1}

p = y.gyro_x;  q = y.gyro_y;  r = y.gyro_z;
phi   = x_att(1);
theta = x_att(2);

% TODO: propagate attitude (Eq 8.19)
%   phi_dot   = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r
%   theta_dot = cos(phi)*q - sin(phi)*r
% f_att = [phi_dot; theta_dot];
% A_att = jacobian df/dx;       % Eq 8.20
% Q_att = diag([sigma_gyro^2, sigma_gyro^2]);
% P_att = P_att + Ts*(A_att*P_att + P_att*A_att' + Q_att);  % Eq 8.21
% x_att = x_att + Ts * f_att;

x_att = x_att;    % TODO: propagate (Eq 8.19)
P_att = P_att;    % TODO: covariance propagate (Eq 8.21)

% TODO: measurement update from accelerometers (Eqs 8.25-8.29)
% z_att = [y.accel_x; y.accel_y; y.accel_z];
% h_att = accel_model(x_att, Va_hat, P);
% H_att = jacobian dh/dx;
% R_att = diag([sigma_accel^2, ...]);
% S = H_att*P_att*H_att' + R_att;
% K = P_att * H_att' / S;
% x_att = x_att + K*(z_att - h_att);
% P_att = (eye(2) - K*H_att)*P_att;

phi_hat   = x_att(1);
theta_hat = x_att(2);

%% ============================================================
%  BLOCK 3 — GPS EKF  (Sec 8.10, Eqs 8.34-8.50)
% ============================================================
% State: x_gps = [pn; pe; Vg; chi; wn; we; psi]

% TODO: propagate GPS EKF state (Eq 8.34)
%   pn_dot  = Vg*cos(chi)
%   pe_dot  = Vg*sin(chi)
%   Vg_dot  = ((Va*cos(psi)+wn)*(-Va*psi_dot*sin(psi)) + ...
%              (Va*sin(psi)+we)*( Va*psi_dot*cos(psi))) / Vg
%   chi_dot = (P.g/Vg)*tan(phi_hat)          % coordinated turn
%   wn_dot  = 0;  we_dot = 0;  psi_dot = q*sin(phi)/cos(theta)+r*cos(phi)/cos(theta)
% A_gps = jacobian df/dx;       % Eq 8.35
% x_gps = x_gps + Ts*f_gps;
% P_gps = P_gps + Ts*(A_gps*P_gps + P_gps*A_gps' + Q_gps);

x_gps = x_gps;   % TODO: propagate (Eq 8.34)
P_gps = P_gps;   % TODO: covariance propagate

% GPS measurement update (triggered when new GPS reading arrives, Eq 8.44)
gps_now = [y.gps_n; y.gps_e; y.gps_h; y.gps_Vg; y.gps_course];
if norm(gps_now - gps_old) > 1e-6
    % TODO: measurement update (Eqs 8.44-8.50)
    % z_gps = [y.gps_n; y.gps_e; y.gps_Vg; y.gps_course];
    % h_gps = [x_gps(1); x_gps(2); x_gps(3); x_gps(4)];
    % H_gps = [eye(4), zeros(4,3)];
    % R_gps = diag([sigma_GPS^2, sigma_GPS^2, sigma_Vg^2, sigma_chi^2]);
    % K = P_gps * H_gps' / (H_gps*P_gps*H_gps' + R_gps);
    % x_gps = x_gps + K*(z_gps - h_gps);
    % P_gps = (eye(7) - K*H_gps)*P_gps;
    gps_old = gps_now;
end

%% ---- Pack output struct ----
xhat.pn      = x_gps(1);
xhat.pe      = x_gps(2);
xhat.h       = -x_gps(2);   % altitude (positive up)
xhat.Va      = Va_hat;
xhat.alpha   = alpha_hat;
xhat.beta    = beta_hat;
xhat.phi     = phi_hat;
xhat.theta   = theta_hat;
xhat.chi     = x_gps(4);
xhat.psi     = x_gps(7);
xhat.Vg      = x_gps(3);
xhat.wn      = x_gps(5);
xhat.we      = x_gps(6);
xhat.p       = p;            % gyro (not estimated, just passed through)
xhat.q       = q;
xhat.r       = r;

end
