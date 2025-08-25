package frc.lib.robotprovider;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class AKBooleanSubscriberWrapper implements IBooleanSubscriber
{
    private final LoggedNetworkBoolean bool;

    AKBooleanSubscriberWrapper(LoggedNetworkBoolean bool)
    {
        this.bool = bool;
    }

    @Override
    public boolean get()
    {
        return this.bool.get();
    }
}
