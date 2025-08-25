package frc.lib.robotprovider;

import org.littletonrobotics.junction.networktables.LoggedNetworkString;

public class AKStringSubscriberWrapper implements IStringSubscriber
{
    private final LoggedNetworkString str;

    AKStringSubscriberWrapper(LoggedNetworkString str)
    {
        this.str = str;
    }

    @Override
    public String get()
    {
        return this.str.get();
    }
}
