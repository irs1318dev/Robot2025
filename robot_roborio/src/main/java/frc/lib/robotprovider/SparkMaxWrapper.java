package frc.lib.robotprovider;

import com.revrobotics.spark.SparkMax;

public class SparkMaxWrapper extends SparkBaseWrapper implements ISparkMax
{
    private SparkMaxWrapper(SparkMax wrappedObject, String SparkMaxId)
    {
        super(wrappedObject, SparkMaxId);
    }

    public static SparkMaxWrapper create(int deviceId, SparkMotorType motorType)
    {
        SparkMax motor = new SparkMax(deviceId, SparkBaseWrapper.getMotorType(motorType));
        String sparkMaxId = String.format("SparkMax %s-%d", motorType, deviceId);
        return new SparkMaxWrapper(motor, sparkMaxId);
    }
}
