package frc.lib.robotprovider;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class Pigeon2Wrapper implements IPigeon2
{
    private static final double timeoutSec = 0.025;

    private final Pigeon2 wrappedObject;

    private final String pigeonId;

    private StatusSignal<Angle> yaw;
    private StatusSignal<Angle> pitch;
    private StatusSignal<Angle> roll;

    private StatusSignal<AngularVelocity> rollRate;
    private StatusSignal<AngularVelocity> pitchRate;
    private StatusSignal<AngularVelocity> yawRate;

    public Pigeon2Wrapper(int deviceNumber)
    {
        this.wrappedObject = new Pigeon2(deviceNumber);
        this.pigeonId = String.format("Pigeon2 %d", deviceNumber);
    }

    public Pigeon2Wrapper(int deviceNumber, String canbus)
    {
        this.wrappedObject = new Pigeon2(deviceNumber, canbus);
        this.pigeonId = String.format("Pigeon2 %s-%d", canbus, deviceNumber);
    }

    public void getYawPitchRoll(double[] ypr_deg)
    {
        if (this.yaw == null)
        {
            this.yaw = this.wrappedObject.getYaw();
        }

        if (this.pitch == null)
        {
            this.pitch = this.wrappedObject.getPitch();
        }

        if (this.roll == null)
        {
            this.roll = this.wrappedObject.getRoll();
        }

        this.yaw.refresh();
        this.pitch.refresh();
        this.roll.refresh();
        CTREStatusCodeHelper.printError(this.yaw.getStatus(), this.pigeonId, "Pigeon2.getYawPitchRoll-yaw");
        CTREStatusCodeHelper.printError(this.pitch.getStatus(), this.pigeonId, "Pigeon2.getYawPitchRoll-pitch");
        CTREStatusCodeHelper.printError(this.roll.getStatus(), this.pigeonId, "Pigeon2.getYawPitchRoll-roll");
        ypr_deg[0] = this.yaw.getValue().magnitude();
        ypr_deg[1] = this.pitch.getValue().magnitude();
        ypr_deg[2] = this.roll.getValue().magnitude();
    }

    public void getRollPitchYawRates(double[] xyz_dps)
    {
        if (this.rollRate == null)
        {
            this.rollRate = this.wrappedObject.getAngularVelocityXWorld();
        }

        if (this.pitchRate == null)
        {
            this.pitchRate = this.wrappedObject.getAngularVelocityYWorld();
        }

        if (this.yawRate == null)
        {
            this.yawRate = this.wrappedObject.getAngularVelocityZWorld();
        }

        this.rollRate.refresh();
        this.pitchRate.refresh();
        this.yawRate.refresh();
        CTREStatusCodeHelper.printError(this.rollRate.getStatus(), this.pigeonId, "Pigeon2.getRawGyro-rollRate");
        CTREStatusCodeHelper.printError(this.pitchRate.getStatus(), this.pigeonId, "Pigeon2.getRawGyro-pitchRate");
        CTREStatusCodeHelper.printError(this.yawRate.getStatus(), this.pigeonId, "Pigeon2.getRawGyro-yawRate");
        xyz_dps[0] = this.roll.getValue().magnitude();
        xyz_dps[1] = this.pitch.getValue().magnitude();
        xyz_dps[2] = this.yaw.getValue().magnitude();
    }

    public void setYaw(double angleDeg)
    {
        CTREStatusCodeHelper.printError(
            this.wrappedObject.setYaw(angleDeg, Pigeon2Wrapper.timeoutSec),
            this.pigeonId,
            "Pigeon2.setYaw");
    }

    public void setYPRUpdateFrequency(double frequencyHz)
    {
        if (this.yaw == null)
        {
            this.yaw = this.wrappedObject.getYaw();
        }

        if (this.pitch == null)
        {
            this.pitch = this.wrappedObject.getPitch();
        }

        if (this.roll == null)
        {
            this.roll = this.wrappedObject.getRoll();
        }

        CTREStatusCodeHelper.printError(this.yaw.setUpdateFrequency(frequencyHz, Pigeon2Wrapper.timeoutSec), this.pigeonId, "Pigeon2.setYPRUpdatePeriod-yaw");
        CTREStatusCodeHelper.printError(this.pitch.setUpdateFrequency(frequencyHz, Pigeon2Wrapper.timeoutSec), this.pigeonId, "Pigeon2.setYPRUpdatePeriod-pitch");
        CTREStatusCodeHelper.printError(this.roll.setUpdateFrequency(frequencyHz, Pigeon2Wrapper.timeoutSec), this.pigeonId, "Pigeon2.setYPRUpdatePeriod-roll");
    }

    public void setRPYRateUpdateFrequency(double frequencyHz)
    {
        if (this.rollRate == null)
        {
            this.rollRate = this.wrappedObject.getAngularVelocityXWorld();
        }

        if (this.pitchRate == null)
        {
            this.pitchRate = this.wrappedObject.getAngularVelocityXWorld();
        }

        if (this.yawRate == null)
        {
            this.yawRate = this.wrappedObject.getAngularVelocityXWorld();
        }

        CTREStatusCodeHelper.printError(this.rollRate.setUpdateFrequency(frequencyHz, Pigeon2Wrapper.timeoutSec), this.pigeonId, "Pigeon2.setGyroUpdatePeriod-rollRate");
        CTREStatusCodeHelper.printError(this.pitchRate.setUpdateFrequency(frequencyHz, Pigeon2Wrapper.timeoutSec), this.pigeonId, "Pigeon2.setGyroUpdatePeriod-pitchRate");
        CTREStatusCodeHelper.printError(this.yawRate.setUpdateFrequency(frequencyHz, Pigeon2Wrapper.timeoutSec), this.pigeonId, "Pigeon2.setGyroUpdatePeriod-yawRate");
    }
}