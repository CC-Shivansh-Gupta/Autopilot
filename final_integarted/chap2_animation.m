function chap2_animation(block)
% =========================================================
% chap2_animation.m  —  Level-2 S-Function: Aircraft Animation
% AE700 — IIT Bombay
%
% INPUT:  6x1 = [pn; pe; pd; phi; theta; psi]
% PARAM:  mode  (1=correct rotate-then-translate,
%                2=wrong translate-then-rotate for Part iv)
% =========================================================

setup(block);

function setup(block)
    block.NumInputPorts  = 1;
    block.NumOutputPorts = 0;
    block.InputPort(1).Dimensions        = 6;
    block.InputPort(1).DirectFeedthrough = true;
    block.NumDialogPrms  = 1;
    block.SampleTimes    = [-1, 0];
    block.RegBlockMethod('Start',     @mdlStart);
    block.RegBlockMethod('Outputs',   @mdlOutputs);
    block.RegBlockMethod('Terminate', @mdlTerminate);
end

function mdlStart(block)  %#ok<INUSD>
    global f4_patch_handle;
    f4_patch_handle = [];
end

function mdlOutputs(block)
    global f4_patch_handle;
    uu   = block.InputPort(1).Data;
    mode = block.DialogPrm(1).Data;
    f4_patch_handle = drawAircraft(uu, f4_patch_handle, mode, 1);
end

function mdlTerminate(block)  %#ok<INUSD>
    global f4_patch_handle;
    f4_patch_handle = [];
end

end
