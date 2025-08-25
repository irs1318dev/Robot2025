package frc.lib.robotprovider;

import edu.wpi.first.wpilibj.Alert;

public class AlertWrapper implements IAlert
{
    final Alert alert;

    private AlertWrapper(Alert alert)
    {
        this.alert = alert;
    }

    public static IAlert create(String text, AlertType type)
    {
        edu.wpi.first.wpilibj.Alert.AlertType wpiAlertType;
        switch (type)
        {
            case Info:
                wpiAlertType = edu.wpi.first.wpilibj.Alert.AlertType.kInfo;
                break;

            case Warning:
                wpiAlertType = edu.wpi.first.wpilibj.Alert.AlertType.kWarning;
                break;

            case Error:
            default:
                wpiAlertType = edu.wpi.first.wpilibj.Alert.AlertType.kError;
                break;
        }

        return new AlertWrapper(new Alert(text, wpiAlertType));
    }

    @Override
    public AlertType getType()
    {
        switch (this.alert.getType())
        {
            case kInfo:
                return AlertType.Info;

            case kWarning:
                return AlertType.Warning;

            case kError:
            default:
                return AlertType.Error;
        }
    }

    @Override
    public String getText()
    {
        return this.alert.getText();
    }

    @Override
    public void updateText(String newText)
    {
        this.alert.setText(newText);
    }

    @Override
    public void enable()
    {
        this.set(true);
    }

    @Override
    public void disable()
    {
        this.set(false);
    }

    @Override
    public void set(boolean enabled)
    {
        this.alert.set(enabled);
    }
}