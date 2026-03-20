function chap4_forces_moments_sfcn(block)
% =========================================================
% chap4_forces_moments_sfcn.m  —  Level-2 MATLAB S-Function
% Wrapper around forces_moments.m for use in Simulink.
%
% INPUT PORT  (22x1):
%   u(1:12)  = state vector  [pn,pe,pd,u,v,w,phi,theta,psi,p,q,r]
%   u(13:16) = controls      [de,da,dr,dt]
%   u(17:22) = wind          [wn,we,wd,ug,vg,wg]
%
% OUTPUT PORT (12x1):
%   [fx,fy,fz, l,m,n, Va,alpha,beta, wn,we,wd]
%
% DIALOG PARAMETERS: none required (P loaded from workspace)
%
% Usage in Simulink:
%   Block type : Level-2 MATLAB S-Function
%   FunctionName: chap4_forces_moments_sfcn
%   Parameters  : (leave blank)
% =========================================================

setup(block);

%% ============================================================
function setup(block)
    block.NumInputPorts  = 1;
    block.NumOutputPorts = 1;

    block.InputPort(1).Dimensions        = 22;   % state(12)+ctrl(4)+wind(6)
    block.InputPort(1).DirectFeedthrough = true;

    block.OutputPort(1).Dimensions = 12;          % fm output

    block.NumContStates  = 0;
    block.NumDialogPrms  = 0;
    block.SampleTimes    = [0, 0];   % continuous

    block.RegBlockMethod('Outputs',   @mdlOutputs);
    block.RegBlockMethod('Terminate', @mdlTerminate);
end

%% ============================================================
function mdlOutputs(block)
    u_in = block.InputPort(1).Data;

    x     = u_in(1:12);
    delta = u_in(13:16);
    wind  = u_in(17:22);

    % Load P from base workspace
    P = evalin('base','P');

    out = forces_moments(x, delta, wind, P);

    block.OutputPort(1).Data = out;
end

%% ============================================================
function mdlTerminate(block) %#ok<INUSD>
end

end
