package frc.lib.robotprovider;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class CANCoderWrapper implements ICANCoder
{
    private static final double timeoutSecs = 0.025;

    private final CANcoder wrappedObject;

    private final String cancoderId;

    private boolean reverse;

    private StatusSignal<Angle> position;
    private StatusSignal<Angle> absolutePosition;
    private StatusSignal<AngularVelocity> velocity;

    public CANCoderWrapper(int deviceNumber)
    {
        this.wrappedObject = new CANcoder(deviceNumber);
        this.cancoderId = String.format("CANcoder %d", deviceNumber);
        this.reverse = false;
    }

    public CANCoderWrapper(int deviceNumber, String canbus)
    {
        this.wrappedObject = new CANcoder(deviceNumber, canbus);
        this.cancoderId = String.format("CANcoder %s-%d", canbus, deviceNumber);
        this.reverse = false;
    }

    public double getPosition()
    {
        if (this.position == null)
        {
            this.position = this.wrappedObject.getPosition();
        }

        this.position.refresh();
        CTREStatusCodeHelper.printError(this.position.getStatus(), this.cancoderId, "CANCoderWrapper.getPosition");
        return this.position.getValue().magnitude() * (this.reverse ? -1.0 : 1.0);
    }

    public double getVelocity()
    {
        if (this.velocity == null)
        {
            this.velocity = this.wrappedObject.getVelocity();
        }

        this.velocity.refresh();
        CTREStatusCodeHelper.printError(this.velocity.getStatus(), this.cancoderId, "CANCoderWrapper.getVelocity");
        return this.velocity.getValue().magnitude() * (this.reverse ? -1.0 : 1.0);
    }

    public double getAbsolutePosition()
    {
        if (this.absolutePosition == null)
        {
            this.absolutePosition = this.wrappedObject.getAbsolutePosition();
        }

        this.absolutePosition.refresh();
        CTREStatusCodeHelper.printError(this.absolutePosition.getStatus(), this.cancoderId, "CANCoderWrapper.getAbsolutePosition");
        return this.absolutePosition.getValue().magnitude() * (reverse ? -1.0 : 1.0);
    }

    public void setPosition(double newPosition)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.setPosition(newPosition, CANCoderWrapper.timeoutSecs),
            this.cancoderId,
            "CANCoderWrapper.setPosition");
    }

    public void configSensorDirection(boolean clockwisePositive)
    {
        this.reverse = clockwisePositive;
    }
}