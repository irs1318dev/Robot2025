package frc.robot.driver.controltasks;

import frc.lib.helpers.Helpers;
import frc.robot.TuningConstants;
import frc.robot.mechanisms.ElevatorMechanism;
import frc.robot.driver.AnalogOperation;

public class ElevatorPositionTask extends ControlTaskBase
{
    private final double lifterPosition;

    private ElevatorMechanism elevator;

    public ElevatorPositionTask(double lifterPosition)
    {
        this.lifterPosition = lifterPosition;
    }

    @Override
    public void begin()
    {
        this.elevator = this.getInjector().getInstance(ElevatorMechanism.class);

        this.setAnalogOperationState(AnalogOperation.ElevatorDesiredPosition, this.lifterPosition);
    }

    @Override
    public void update()
    {
        this.setAnalogOperationState(AnalogOperation.ElevatorDesiredPosition, this.lifterPosition);
    }

    @Override
    public void end()
    {
        this.setAnalogOperationState(AnalogOperation.ElevatorDesiredPosition, TuningConstants.MAGIC_NULL_VALUE);
    }

    @Override
    public boolean hasCompleted()
    {
        return Helpers.withinDelta(this.elevator.getPosition(), lifterPosition, TuningConstants.ELEVATOR_POSITION_TOLERANCE);
    }
}
