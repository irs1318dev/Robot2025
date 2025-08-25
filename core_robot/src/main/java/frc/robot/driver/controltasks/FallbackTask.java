package frc.robot.driver.controltasks;

import frc.lib.driver.IControlTask;

public class FallbackTask extends DecisionSequentialTask
{
    private final IControlTask initialTask;
    private final IControlTask fallbackTask;

    /**
     * Executes an intial task, and if that task aborts (indicates that it has failed and should be canceled), executes the fallback
     * @param initialTask to execute by default
     * @param fallbackTask to execute only if the initial task indicates that it should be canceled
     */
    public FallbackTask(
        IControlTask initialTask,
        IControlTask fallbackTask)
    {
        this.initialTask = initialTask;
        this.fallbackTask = fallbackTask;

        this.AppendTask(initialTask);
    }

    @Override
    protected boolean onTaskCanceled(IControlTask task)
    {
        if (task == this.initialTask)
        {
            this.AppendTask(this.fallbackTask);
            return false;
        }

        return super.onTaskCanceled(task);
    }
}
