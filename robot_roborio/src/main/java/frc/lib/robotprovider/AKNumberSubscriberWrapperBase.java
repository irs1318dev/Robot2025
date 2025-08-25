package frc.lib.robotprovider;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class AKNumberSubscriberWrapperBase
{
    protected final LoggedNetworkNumber number;

    AKNumberSubscriberWrapperBase(LoggedNetworkNumber number)
    {
        this.number = number;
    }
}
