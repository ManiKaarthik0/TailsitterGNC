function imu_sfun(block)
setup(block);

function setup(block)
    block.NumInputPorts  = 1;   % X_true (16x1)
    block.NumOutputPorts = 3;   % z_gyro(3), z_accel(3), z_mag(1)
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;
    block.InputPort(1).Dimensions  = 16;
    block.OutputPort(1).Dimensions = 3;
    block.OutputPort(2).Dimensions = 3;
    block.OutputPort(3).Dimensions = 1;   % <-- new: magnetometer heading
    block.InputPort(1).DirectFeedthrough = true;
    block.SampleTimes = [0.01 0];         % <-- DISCRETE 100 Hz (was [0 0])
    block.RegBlockMethod('Outputs', @Outputs);

function Outputs(block)
    X_true = block.InputPort(1).Data;
    [z_gyro, z_accel, z_mag] = imu_simulate(X_true, 0.01);
    block.OutputPort(1).Data = z_gyro;
    block.OutputPort(2).Data = z_accel;
    block.OutputPort(3).Data = z_mag;