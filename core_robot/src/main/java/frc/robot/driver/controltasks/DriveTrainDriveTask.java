package frc.robot.driver.controltasks;

import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;

public class DriveTrainDriveTask extends TimedTask
{
    private final boolean useRobotOrientatation;

    private final double xVelocity;
    private final double yVelocity;

    public DriveTrainDriveTask(double duration, boolean useRobotOrientatation, double xVelocity, double yVelocity)
    {
        super(duration);

        this.useRobotOrientatation = useRobotOrientatation;

        this.xVelocity = xVelocity;
        this.yVelocity = yVelocity;
    }

    @Override
    public void update()
    {
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, this.xVelocity);
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveRight, -this.yVelocity);

        this.setDigitalOperationState(DigitalOperation.DriveTrainUseRobotOrientation, this.useRobotOrientatation);
    }

    @Override
    public void end()
    {
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveRight, 0.0);

        this.setDigitalOperationState(DigitalOperation.DriveTrainUseRobotOrientation, false);
    }
}
