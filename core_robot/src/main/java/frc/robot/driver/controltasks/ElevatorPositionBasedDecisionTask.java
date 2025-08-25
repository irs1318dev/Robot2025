package frc.robot.driver.controltasks;

import frc.lib.driver.IControlTask;
import frc.robot.mechanisms.ElevatorMechanism;

public class ElevatorPositionBasedDecisionTask extends DecisionSequentialTask
{
    private final double threshold;

    private final IControlTask lowTask;
    private final IControlTask highTask;

    public ElevatorPositionBasedDecisionTask(
        double threshold,
        IControlTask lowTask,
        IControlTask highTask)
    {
        this.threshold = threshold;

        this.lowTask = lowTask;
        this.highTask = highTask;
    }

    @Override
    public void begin()
    {
        super.begin();

        if (this.getInjector().getInstance(ElevatorMechanism.class).getPosition() > this.threshold)
        {
            this.AppendTask(this.highTask);
        }
        else
        {
            this.AppendTask(this.lowTask);
        }
    }
}
