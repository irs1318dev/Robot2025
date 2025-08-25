package frc.lib.robotprovider;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

public class AKNetworkTableProvider extends NetworkTableProviderBase
{
    public static final AKNetworkTableProvider Instance = new AKNetworkTableProvider();

    private AKNetworkTableProvider()
    {
    }

    @Override
    public IIntegerSubscriber getIntegerSlider(String title, int initialValue)
    {
        return this.getIntegerSubscriber(title, initialValue);
    }

    @Override
    public IDoubleSubscriber getNumberSlider(String title, double initialValue)
    {
        return this.getDoubleSubscriber(title, initialValue);
    }

    @Override
    public IBooleanSubscriber getCheckbox(String title, boolean initialValue)
    {
        return this.getBooleanSubscriber(title, initialValue);
    }

    @Override
    public <V> ISendableChooser<V> getSendableChooser(String name)
    {
        return new AKSendableChooserWrapper<V>(name);
    }

    @Override
    public IDoubleSubscriber getDoubleSubscriber(String key)
    {
        return this.getDoubleSubscriber(key, 0.0);
    }

    @Override
    public IDoubleSubscriber getDoubleSubscriber(String key, double defaultValue)
    {
        LoggedNetworkNumber number = new LoggedNetworkNumber("/SmartDashboard/" + key, defaultValue);
        return new AKDoubleSubscriberWrapper(number);
    }

    @Override
    public IBooleanSubscriber getBooleanSubscriber(String key)
    {
        return this.getBooleanSubscriber(key, false);
    }

    @Override
    public IBooleanSubscriber getBooleanSubscriber(String key, boolean defaultValue)
    {
        LoggedNetworkBoolean bool = new LoggedNetworkBoolean("/SmartDashboard/" + key, defaultValue);
        return new AKBooleanSubscriberWrapper(bool);
    }

    @Override
    public IIntegerSubscriber getIntegerSubscriber(String key)
    {
        return this.getIntegerSubscriber(key, 0);
    }

    @Override
    public IIntegerSubscriber getIntegerSubscriber(String key, int defaultValue)
    {
        LoggedNetworkNumber number = new LoggedNetworkNumber("/SmartDashboard/" + key, defaultValue);
        return new AKIntegerSubscriberWrapper(number);
    }

    @Override
    public IStringSubscriber getStringSubscriber(String key)
    {
        return this.getStringSubscriber(key, null);
    }

    @Override
    public IStringSubscriber getStringSubscriber(String key, String defaultValue)
    {
        LoggedNetworkString str = new LoggedNetworkString("/SmartDashboard/" + key, defaultValue);
        return new AKStringSubscriberWrapper(str);
    }
}