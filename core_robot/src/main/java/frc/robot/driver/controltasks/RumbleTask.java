package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;

public class RumbleTask extends TimedTask
{
    public RumbleTask(double duration)
    {
        super(duration);
    }

    @Override
    public void update()
    {
        this.setDigitalOperationState(DigitalOperation.ForceLightDriverRumble, true);
    }

    @Override
    public void end()
    {
        super.end();

        this.setDigitalOperationState(DigitalOperation.ForceLightDriverRumble, false);
    }
}
