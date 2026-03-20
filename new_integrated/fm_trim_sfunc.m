function fm_trim_sfunc(block)
% fm_trim_sfunc.m  -  Level-2 MATLAB S-Function
% Wrapper for forces_moments.m for use inside mavsim_trim.slx
%
% INPUT PORT (22x1):  [state(12); controls(4); wind(6)]
%   state(1:12)   = [pn,pe,pd,u,v,w,phi,theta,psi,p,q,r]
%   controls(1:4) = [de,da,dr,dt]
%   wind(1:6)     = [wn,we,wd,ug,vg,wg]
%
% OUTPUT PORT (12x1): [fx,fy,fz,l,m,n,Va,alpha,beta,wn,we,wd]
%
% DIALOG PARAMETER: P  (aircraft parameter struct from F4_chap4_params)

setup(block);

    function setup(block)
        block.NumInputPorts  = 1;
        block.NumOutputPorts = 1;
        block.InputPort(1).Dimensions        = 22;
        block.InputPort(1).DirectFeedthrough = true;
        block.OutputPort(1).Dimensions       = 12;
        block.NumContStates  = 0;
        block.NumDialogPrms  = 1;   % P struct
        block.SampleTimes    = [0, 0];
        block.RegBlockMethod('Outputs',   @mdlOutputs);
        block.RegBlockMethod('Terminate', @mdlTerminate);
    end

    function mdlOutputs(block)
        P  = block.DialogPrm(1).Data;
        uu = block.InputPort(1).Data;
        x      = uu(1:12);
        delta  = uu(13:16);
        wind   = uu(17:22);
        out = forces_moments(x, delta, wind, P);
        block.OutputPort(1).Data = out(1:12);
    end

    function mdlTerminate(~)
    end

end
