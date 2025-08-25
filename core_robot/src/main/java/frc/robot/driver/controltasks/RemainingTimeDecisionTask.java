package frc.robot.driver.controltasks;

import frc.lib.driver.IControlTask;
import frc.lib.robotprovider.IDriverStation;
import frc.lib.robotprovider.IRobotProvider;

public class RemainingTimeDecisionTask extends DecisionSequentialTask
{
    private final double threshold;

    private final IControlTask lowTask;
    private final IControlTask highTask;

    /**
     * Decision task that runs one of two tasks based on the current match time.
     * @param threshold threshold time in seconds - for values above this time, the highTask will run, otherwise the lowTask will run.
     * @param lowTask the task to run if the match time is below the threshold
     * @param highTask the task to run if the match time is above the threshold
     */
    public RemainingTimeDecisionTask(
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

        IDriverStation ds = this.getInjector().getInstance(IRobotProvider.class).getDriverStation();
        if (ds.getMatchTime() > this.threshold)
        {
            this.AppendTask(this.highTask);
        }
        else
        {
            this.AppendTask(this.lowTask);
        }
    }
}
