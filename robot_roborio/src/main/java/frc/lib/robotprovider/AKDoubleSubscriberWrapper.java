package frc.lib.robotprovider;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class AKDoubleSubscriberWrapper extends AKNumberSubscriberWrapperBase implements IDoubleSubscriber
{
    AKDoubleSubscriberWrapper(LoggedNetworkNumber number)
    {
        super(number);
    }

    @Override
    public double get()
    {
        return this.number.get();
    }
}
