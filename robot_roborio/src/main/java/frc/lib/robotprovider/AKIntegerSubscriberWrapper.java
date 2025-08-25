package frc.lib.robotprovider;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class AKIntegerSubscriberWrapper extends AKNumberSubscriberWrapperBase implements IIntegerSubscriber
{
    AKIntegerSubscriberWrapper(LoggedNetworkNumber number)
    {
        super(number);
    }

    @Override
    public long get()
    {
        return (long)this.number.get();
    }
}
