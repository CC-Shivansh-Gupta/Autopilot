function [az, el, img_pt] = gimbal_model(state, target_pos, P)
% =========================================================
% gimbal_model.m
% Gimbal pointing model and image-plane projection.
% Beard & McLain, "Small Unmanned Aircraft," Chapter 13.
%
% INPUTS:
%   state      - 12x1 MAV state [pn,pe,pd,u,v,w,phi,theta,psi,p,q,r]
%   target_pos - 3x1 target position in NED [pn; pe; pd]  [ft]
%   P          - param struct (needs P.f_cam, P.img_width, P.img_height)
%
% OUTPUTS:
%   az     - gimbal azimuth angle [rad]  (rotation about body-z)
%   el     - gimbal elevation angle [rad] (rotation about gimbal-y, positive = nose-down)
%   img_pt - 2x1 image-plane point [x_img; y_img] in pixels
%            origin at image centre; x=right, y=down
%
% Key rotation chain (Eq 13.x):
%   NED -> body: R_bv (from phi,theta,psi)
%   body -> gimbal: R_gimbal (from az, el)
%   camera: pinhole projection
%
% Units: ft, rad.
% =========================================================

%% Unpack MAV state
pn    = state(1);  pe    = state(2);  pd    = state(3);
phi   = state(7);  theta = state(8);  psi   = state(9);

%% ---- Rotation: NED -> Body (Eq 2.4) ----
cphi=cos(phi); sphi=sin(phi);
cth =cos(theta); sth=sin(theta);
cpsi=cos(psi);   spsi=sin(psi);

R_bv = [cth*cpsi,                  cth*spsi,                -sth;
        sphi*sth*cpsi-cphi*spsi,   sphi*sth*spsi+cphi*cpsi,  sphi*cth;
        cphi*sth*cpsi+sphi*spsi,   cphi*sth*spsi-sphi*cpsi,  cphi*cth];

%% ---- Vector from MAV to target in NED ----
mav_pos  = [pn; pe; pd];
l_ned    = target_pos - mav_pos;          % pointing from MAV to target

%% ---- Express target direction in body frame ----
l_body   = R_bv * l_ned;                  % Eq 13.x

%% ---- Gimbal angles (az, el) from body-frame target vector ----
% Azimuth:   rotation about body-z to align gimbal-x with target
% Elevation: rotation about gimbal-y (tilt toward target)
% TODO: implement gimbal angle computation
%   az = atan2(l_body(2), l_body(1));       % azimuth in body xy-plane
%   el = atan2(-l_body(3), norm(l_body(1:2)));  % elevation (positive = up)

az = 0;   % TODO Eq 13.x
el = 0;   % TODO Eq 13.x

%% ---- Rotation: body -> gimbal (az then el) ----
% R_az  = Rz(az)   rotation about body-z
% R_el  = Ry(-el)  rotation about gimbal-y (sign depends on convention)
% R_gimbal = R_el * R_az

% TODO: build R_gimbal from az and el
R_gimbal = eye(3);   % placeholder

%% ---- Target vector in camera frame ----
l_cam = R_gimbal * l_body;   % Eq 13.x

%% ---- Pinhole projection onto image plane ----
% Pinhole model: x_img = f * (l_cam(2)/l_cam(1))  (right = +y_cam)
%                y_img = f * (l_cam(3)/l_cam(1))  (down  = +z_cam)
% f = P.f_cam [pixels], assuming l_cam(1) > 0 (target in front of camera)

if l_cam(1) < 1e-3
    img_pt = [0; 0];   % target behind camera
    return;
end

% TODO: pinhole projection (Eq 13.x)
%   x_img = P.f_cam * l_cam(2) / l_cam(1);
%   y_img = P.f_cam * l_cam(3) / l_cam(1);

img_pt = [0; 0];   % TODO: [x_img; y_img] in pixels

end
