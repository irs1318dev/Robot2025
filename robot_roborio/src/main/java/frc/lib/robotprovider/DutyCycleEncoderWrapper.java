package frc.lib.robotprovider;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.helpers.ExceptionHelpers;

public class DutyCycleEncoderWrapper implements IDutyCycleEncoder
{
    private final DutyCycleEncoder wrappedObject;

    private boolean inverted;
    private double distancePerRotation;
    private double offset;

    public DutyCycleEncoderWrapper(int digitalInputChannel)
    {
        this.wrappedObject = new DutyCycleEncoder(digitalInputChannel);
        this.inverted = false;
        this.distancePerRotation = 1.0;
        this.offset = 0.0;
    }

    public double get()
    {
        double rawValue = this.wrappedObject.get();
        if (this.inverted)
        {
            rawValue *= -1.0;
        }

        return this.offset + rawValue;
    }

    public double getDistance()
    {
        return this.get() * this.distancePerRotation;
    }

    public double getAbsolutePosition()
    {
        double distance = this.getDistance() % this.distancePerRotation;
        if (distance < -0.0)
        {
            distance += this.distancePerRotation;
        }

        return distance;
    }

    public int getFrequency()
    {
        return this.wrappedObject.getFrequency();
    }

    public boolean isConnected()
    {
        return this.wrappedObject.isConnected();
    }

    public void setConnectedFrequencyThreshold(int frequency)
    {
        this.wrappedObject.setConnectedFrequencyThreshold(frequency);
    }

    public void setDistancePerRotation(double distancePerRotation)
    {
        ExceptionHelpers.Assert(distancePerRotation >= 0.0, "Distance per rotation should be a non-negative number.  Actual: %f", distancePerRotation);
        this.distancePerRotation = distancePerRotation;
    }

    public void setInverted(boolean inverted)
    {
        this.inverted = true;
    }

    public void setDutyCycleRange(double min, double max)
    {
        this.wrappedObject.setDutyCycleRange(min, max);
    }

    public void setPositionOffset(double offset)
    {
        this.offset = offset;
    }

    public void reset()
    {
        this.offset = -this.wrappedObject.get();
    }
}
