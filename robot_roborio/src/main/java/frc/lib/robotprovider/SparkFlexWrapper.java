package frc.lib.robotprovider;

import com.revrobotics.spark.SparkFlex;

public class SparkFlexWrapper extends SparkBaseWrapper implements ISparkFlex
{
    private SparkFlexWrapper(SparkFlex wrappedObject, String sparkFlexId)
    {
        super(wrappedObject, sparkFlexId);
    }

    public static SparkFlexWrapper create(int deviceId, SparkMotorType motorType)
    {
        SparkFlex motor = new SparkFlex(deviceId, SparkBaseWrapper.getMotorType(motorType));
        String sparkFlexId = String.format("SparkFlex %s-%d", motorType, deviceId);
        return new SparkFlexWrapper(motor, sparkFlexId);
    }
}
