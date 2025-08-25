package frc.lib.robotprovider;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public abstract class NetworkTableProviderBase implements INetworkTableProvider
{
    @Override
    public void startShuffleboardRecording()
    {
        Shuffleboard.startRecording();
    }

    @Override
    public void stopShuffleboardRecording()
    {
        Shuffleboard.stopRecording();
    }

    @Override
    public IAlert createAlert(String text, AlertType type)
    {
        return AlertWrapper.create(text, type);
    }

    @Override
    public IField2d getField2d(String name)
    {
        return Field2dWrapper.create(name);
    }
}