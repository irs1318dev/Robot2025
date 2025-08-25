package frc.robot.driver.controltasks;

import frc.lib.driver.IControlTask;
import frc.robot.mechanisms.AlgaeIntakeMechanism;

public class AlgaeDecisionTask extends DecisionSequentialTask
{
    private final IControlTask withTask;
    private final IControlTask withoutTask;

    public AlgaeDecisionTask(
        IControlTask withTask,
        IControlTask withoutTask)
    {
        this.withTask = withTask;
        this.withoutTask = withoutTask;
    }

    public AlgaeDecisionTask(
        boolean hasAlgae,
        IControlTask task)
    {
        this(hasAlgae ? task : null, hasAlgae ? null : task);
    }

    @Override
    public void begin()
    {
        super.begin();

        if (this.getInjector().getInstance(AlgaeIntakeMechanism.class).hasGamePiece())
        {
            if (this.withTask != null)
            {
                this.AppendTask(this.withTask);
            }
        }
        else
        {
            if (this.withoutTask != null)
            {
                this.AppendTask(this.withoutTask);
            }
        }
    }
}
