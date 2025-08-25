package frc.lib.robotprovider;

import edu.wpi.first.wpilibj.DigitalInput;

public class DigitalInputWrapper implements IDigitalInput
{
    private final DigitalInput wrappedObject;

    private boolean isInverted;

    public DigitalInputWrapper(int channel)
    {
        this.wrappedObject = new DigitalInput(channel);
        this.isInverted = false;
    }

    public boolean get()
    {
        return this.wrappedObject.get() != this.isInverted;
    }

    @Override
    public void setInverted(boolean inverted)
    {
        this.isInverted = inverted;
    }
}
