function fixedwing_sfun(block)
% Level-2 S-Function wrapper for rigid_body.m
% No code generation — calls your existing files directly.
setup(block);

% -------------------------------------------------------
function setup(block)
    block.NumInputPorts  = 2;   % port 1: X (16x1), port 2: U (4x1)
    block.NumOutputPorts = 1;   % port 1: dX (16x1)

    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;

    block.InputPort(1).Dimensions  = 16;
    block.InputPort(2).Dimensions  = 4;
    block.OutputPort(1).Dimensions = 16;

    block.InputPort(1).DirectFeedthrough = true;
    block.InputPort(2).DirectFeedthrough = true;

    block.SampleTimes = [0 0];   % continuous block

    block.RegBlockMethod('Outputs', @Outputs);

% -------------------------------------------------------
function Outputs(block)
    X = block.InputPort(1).Data;
    U = block.InputPort(2).Data;
    P = evalin('base', 'P');     % reads P from MATLAB workspace
    block.OutputPort(1).Data = rigid_body([], X, U, P);