function y = sensors(x, delta, wind, P)
% =========================================================
% sensors.m
% Simulated sensor suite for the F-4 Phantom MAV.
% Beard & McLain, "Small Unmanned Aircraft," Chapter 7.
%
% INPUTS:
%   x     - 12x1 state [pn,pe,pd,u,v,w,phi,theta,psi,p,q,r]
%   delta - 4x1  controls [de,da,dr,dt]
%   wind  - 6x1  wind [wn,we,wd,ug,vg,wg]
%   P     - param struct from F4_params.m
%
% OUTPUT:
%   y     - struct with sensor measurements
%
% Units: ft, slug, lb, rad throughout.
% =========================================================

%% Gauss-Markov bias states (persist between calls)
persistent bias_p bias_q bias_r  % gyro biases
persistent eta_n eta_e eta_h     % GPS position biases (Eq 7.18)

if isempty(bias_p)
    bias_p = 0;  bias_q = 0;  bias_r = 0;
    eta_n  = 0;  eta_e  = 0;  eta_h  = 0;
end

%% Unpack relevant states
pn    = x(1);  pe    = x(2);  pd    = x(3);
phi   = x(7);  theta = x(8);  psi   = x(9);
p     = x(10); q     = x(11); r     = x(12);

%% Aerodynamic quantities (from forces_moments)
fm = forces_moments(x, delta, wind, P);
Va    = fm(7);
alpha = fm(8);
beta  = fm(9);
fx    = fm(1);  fy = fm(2);  fz = fm(3);

%% ---- Accelerometers (Eq 7.3) ----
% Specific force = (total body force - gravity) / mass
% TODO: replace placeholder with proper specific-force calculation
% y.accel_x = (fx + P.mass*P.g*sin(theta)) / P.mass + sigma_ax*randn;
y.accel_x = 0;   % TODO Eq 7.3
y.accel_y = 0;   % TODO Eq 7.3
y.accel_z = 0;   % TODO Eq 7.3

%% ---- Rate Gyros (Eq 7.5) ----
% Measurement = true body rate + bias + white noise
% TODO: evolve bias via first-order Markov: bias_dot = -1/tau_g * bias + noise
% y.gyro_x = p + bias_p + sigma_g*randn;
sigma_g = P.sigma_gyro;   % rad/s noise std (add to P in params)
y.gyro_x = p + bias_p + sigma_g * randn;   % Eq 7.5
y.gyro_y = q + bias_q + sigma_g * randn;   % Eq 7.5
y.gyro_z = r + bias_r + sigma_g * randn;   % Eq 7.5

% TODO: update biases (Gauss-Markov, Eq 7.6)
% bias_p = (1 - P.Ts/P.tau_g)*bias_p + P.Ts/P.tau_g * sigma_gb*randn;

%% ---- Pressure Sensors ----
% Static pressure from altitude (Eq 7.9)
% y.static_pressure = P.rho * P.g * (-pd) + sigma_baro*randn;
y.static_pressure  = 0;   % TODO Eq 7.9

% Dynamic pressure from airspeed (Eq 7.10)
% y.dynamic_pressure = 0.5 * P.rho * Va^2 + sigma_pitot*randn;
y.dynamic_pressure = 0;   % TODO Eq 7.10

%% ---- GPS (Eqs 7.18-7.20, 7.25-7.26) ----
% GPS position with Gauss-Markov bias
% eta_n_dot = -1/tau_GPS * eta_n + sigma_GPS_n * randn;  (Eq 7.18)
% y.gps_n = pn + eta_n;
y.gps_n   = 0;   % TODO Eq 7.18
y.gps_e   = 0;   % TODO Eq 7.19
y.gps_h   = 0;   % TODO Eq 7.20

% Ground speed and course angle (Eqs 7.25-7.26)
% Vg = norm([pndot, pedot]) + noise
% chi = atan2(pedot, pndot) + noise
y.gps_Vg     = 0;   % TODO Eq 7.25
y.gps_course = 0;   % TODO Eq 7.26

end
