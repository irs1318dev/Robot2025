package frc.robot.driver.controltasks;

import frc.lib.helpers.Helpers;
import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.mechanisms.AlgaeIntakeMechanism;

public class AlgaeWristSetAngleTask extends UpdateCycleTask
{
    private final double angle;
    private final boolean waitUntilReached;

    private AlgaeIntakeMechanism algaeMechanism;

    public AlgaeWristSetAngleTask(double angle)
    {
        this(angle, true);
    }

    public AlgaeWristSetAngleTask(double angle, boolean waitUntilReached)
    {
        super(1);

        this.angle = angle;
        this.waitUntilReached = waitUntilReached;
    }

    @Override
    public void begin()
    {
        super.begin();
        this.algaeMechanism = this.getInjector().getInstance(AlgaeIntakeMechanism.class);

        this.setAnalogOperationState(AnalogOperation.AlgaeWristDesiredAngle, this.angle);
    }

    @Override
    public void update()
    {
        super.update();

        this.setAnalogOperationState(AnalogOperation.AlgaeWristDesiredAngle, this.angle);
    }

    @Override
    public void end()
    {
        super.end();

        this.setAnalogOperationState(AnalogOperation.AlgaeWristDesiredAngle, TuningConstants.MAGIC_NULL_VALUE);
    }
    
    @Override
    public boolean hasCompleted()
    {
        if (this.waitUntilReached)
        {
            return Helpers.withinDelta(this.algaeMechanism.getAngle(), this.angle, TuningConstants.ALGAE_WRIST_POSITION_TOLERANCE);
        }

        return super.hasCompleted();
    }
}
