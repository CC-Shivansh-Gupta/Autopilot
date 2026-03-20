function chap2_animation(block)
% chap2_animation - Level-2 S-Function for F-4 Phantom animation
%
% INPUT PORT:  uu (6x1) = [pn; pe; pd; phi; theta; psi]
% PARAMETERS:  mode (1 = correct order, 2 = wrong order for Part iv)
%
% In Simulink: use "Level-2 MATLAB S-Function" block
%   FunctionName : chap2_animation
%   Parameters   : 1

setup(block);

%% ---------------------------------------------------------------
function setup(block)
    block.NumInputPorts  = 1;
    block.NumOutputPorts = 0;
    block.InputPort(1).Dimensions        = 6;
    block.InputPort(1).DirectFeedthrough = true;
    block.NumDialogPrms  = 1;   % mode
    block.SampleTimes    = [-1, 0];
    block.RegBlockMethod('Start',     @mdlStart);
    block.RegBlockMethod('Outputs',   @mdlOutputs);
    block.RegBlockMethod('Terminate', @mdlTerminate);
end

%% ---------------------------------------------------------------
function mdlStart(block)  %#ok<INUSD>
    % Store an empty handle in global so mdlOutputs starts fresh each run
    global f4_patch_handle;
    f4_patch_handle = [];
end

%% ---------------------------------------------------------------
function mdlOutputs(block)
    global f4_patch_handle;
    uu   = block.InputPort(1).Data;
    mode = block.DialogPrm(1).Data;
    f4_patch_handle = drawAircraft(uu, f4_patch_handle, mode, 1);
end

%% ---------------------------------------------------------------
function mdlTerminate(block)  %#ok<INUSD>
    global f4_patch_handle;
    f4_patch_handle = [];
end

end
