package frc.robot.driver.controltasks;

import frc.lib.helpers.Helpers;
import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.ClimberMechanism;

public class ClimberElbowSetPositionTask extends UpdateCycleTask
{
    private final double angle;
    private final boolean waitUntilReached;

    private ClimberMechanism climberMechanism;

    public ClimberElbowSetPositionTask(double angle)
    {
        this(angle, true);
    }

    public ClimberElbowSetPositionTask(double angle, boolean waitUntilReached)
    {
        super(1);

        this.angle = angle;
        this.waitUntilReached = waitUntilReached;
    }

    @Override
    public void begin()
    {
        super.begin();
        this.climberMechanism = this.getInjector().getInstance(ClimberMechanism.class);

        this.setDigitalOperationState(DigitalOperation.ClimberElbowDisableSimpleMode, true);
        this.setAnalogOperationState(AnalogOperation.ClimberElbowDesiredAngle, this.angle);
    }

    @Override
    public void update()
    {
        super.update();

        this.setAnalogOperationState(AnalogOperation.ClimberElbowDesiredAngle, this.angle);
    }

    @Override
    public void end()
    {
        super.end();

        this.setDigitalOperationState(DigitalOperation.ClimberElbowDisableSimpleMode, false);
        this.setAnalogOperationState(AnalogOperation.ClimberElbowDesiredAngle, TuningConstants.MAGIC_NULL_VALUE);
    }

    @Override
    public boolean hasCompleted()
    {
        if (this.waitUntilReached)
        {
            Double elbowAngle = this.climberMechanism.getAngle();
            if (elbowAngle == null)
            {
                return true;
            }

            return Helpers.withinDelta(elbowAngle, this.angle, TuningConstants.CLIMBER_ELBOW_POSITION_TOLERANCE);
        }

        return super.hasCompleted();
    }
}