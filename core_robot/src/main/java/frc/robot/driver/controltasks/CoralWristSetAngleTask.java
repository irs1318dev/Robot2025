package frc.robot.driver.controltasks;

import frc.lib.helpers.Helpers;
import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.mechanisms.CoralEndEffectorMechanism;

public class CoralWristSetAngleTask extends UpdateCycleTask
{
    private final double angle;
    private final boolean waitUntilReached;

    private final double minAngle;
    private final double maxAngle;

    private CoralEndEffectorMechanism coralMechanism;

    public CoralWristSetAngleTask(double angle)
    {
        this(angle, true);
    }

    public CoralWristSetAngleTask(double angle, boolean waitUntilReached)
    {
        super(1);

        this.angle = angle;
        this.minAngle = Helpers.updateAngleRange360(angle - TuningConstants.CORAL_WRIST_POSITION_TOLERANCE);
        this.maxAngle = Helpers.updateAngleRange360(angle + TuningConstants.CORAL_WRIST_POSITION_TOLERANCE);

        this.waitUntilReached = waitUntilReached;
    }

    @Override
    public void begin()
    {
        super.begin();
        this.coralMechanism = this.getInjector().getInstance(CoralEndEffectorMechanism.class);

        this.setAnalogOperationState(AnalogOperation.CoralWristDesiredAngle, this.angle);
    }

    @Override
    public void update()
    {
        super.update();

        this.setAnalogOperationState(AnalogOperation.CoralWristDesiredAngle, this.angle);
    }

    @Override
    public void end()
    {
        super.end();

        this.setAnalogOperationState(AnalogOperation.CoralWristDesiredAngle, TuningConstants.MAGIC_NULL_VALUE);
    }
    
    @Override
    public boolean hasCompleted()
    {
        if (this.waitUntilReached)
        {
            return Helpers.withinAbsoluteAngleRange(this.coralMechanism.getAngle(), this.minAngle, this.maxAngle);
        }

        return super.hasCompleted();
    }
}
