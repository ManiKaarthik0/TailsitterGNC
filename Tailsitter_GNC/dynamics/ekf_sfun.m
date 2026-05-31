function ekf_sfun(block)
setup(block);

function setup(block)
    block.NumInputPorts  = 2;
    block.NumOutputPorts = 1;

    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;

    block.InputPort(1).Dimensions  = 3;
    block.InputPort(2).Dimensions  = 3;
    block.OutputPort(1).Dimensions = 6;

    block.InputPort(1).DirectFeedthrough = true;
    block.InputPort(2).DirectFeedthrough = true;
    block.SampleTimes = [0 0];

    block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
    block.RegBlockMethod('Start',   @Start);
    block.RegBlockMethod('Outputs', @Outputs);

function DoPostPropSetup(block)
    block.NumDworks = 2;

    block.Dwork(1).Name       = 'X_est';
    block.Dwork(1).Dimensions = 6;
    block.Dwork(1).DatatypeID = 0;
    block.Dwork(1).Complexity = 'Real';
    block.Dwork(1).UsedAsDiscState = true;

    block.Dwork(2).Name       = 'P_est';
    block.Dwork(2).Dimensions = 36;
    block.Dwork(2).DatatypeID = 0;
    block.Dwork(2).Complexity = 'Real';
    block.Dwork(2).UsedAsDiscState = true;

function Start(block)
    alpha0 = evalin('base', 'alpha0');
    X0 = [0; 0; alpha0; 0; 0; 0];
    block.Dwork(1).Data = X0;

    P0 = diag([0.1, 0.1, 0.1, 0.01, 0.01, 0.01]);
    block.Dwork(2).Data = P0(:);

function Outputs(block)
    z_gyro  = block.InputPort(1).Data;
    z_accel = block.InputPort(2).Data;

    X_est = block.Dwork(1).Data;
    P_est = reshape(block.Dwork(2).Data, 6, 6);

    Q       = diag([1e-4, 1e-4, 1e-4, 1e-5, 1e-5, 1e-5]);
    R_gyro  = diag([0.005^2, 0.005^2, 0.005^2]);
    R_accel = diag([0.02^2,  0.02^2,  0.02^2]);

    [X_est, P_est] = ekf_attitude(X_est, P_est, z_gyro, z_accel, ...
                                   0.01, Q, R_gyro, R_accel);

    block.Dwork(1).Data = X_est;
    block.Dwork(2).Data = P_est(:);
    block.OutputPort(1).Data = X_est;