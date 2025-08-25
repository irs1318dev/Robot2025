package frc.robot.driver.controltasks;

import frc.robot.mechanisms.ElevatorMechanism;

public class ElevatorPositionWaitTask extends ControlTaskBase
{
    private final double threshold;
    private final boolean greater;

    private ElevatorMechanism elevator;

    public ElevatorPositionWaitTask(
        double threshold,
        boolean greater)
    {
        this.threshold = threshold;
        this.greater = greater;
    }

    @Override
    public void begin()
    {
        this.elevator = this.getInjector().getInstance(ElevatorMechanism.class);
    }

    @Override
    public void update()
    {
    }

    @Override
    public void end()
    {
    }

    @Override
    public boolean hasCompleted()
    {
        // return true if greater is true and elevator position is above threshold,
        // or if greater is false and elevator position is below threshold
        return this.greater == (this.elevator.getPosition() > this.threshold);
    }
}
