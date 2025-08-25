package frc.robot.mechanisms;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.lib.driver.IDriver;
import frc.lib.driver.descriptions.UserInputDevice;
import frc.lib.helpers.Helpers;
import frc.lib.mechanisms.IMechanism;
import frc.lib.robotprovider.IDriverStation;
import frc.lib.robotprovider.IRobotProvider;
import frc.lib.robotprovider.ITimer;
import frc.lib.robotprovider.JoystickRumbleType;
import frc.lib.robotprovider.RobotMode;
import frc.robot.TuningConstants;
import frc.robot.driver.DigitalOperation;

/**
 * Driver feedback manager
 *
 * This class manages things like controller rumbler and indicator lights on the robot.
 *
 */
@Singleton
public class DriverFeedbackManager implements IMechanism
{
    private static enum RumbleMode
    {
        None,
        Coral,
        Endgame,
        Force
    }

    private final IDriver driver;
    private final ITimer timer;
    private final IDriverStation ds;
    private final PowerManager powerMan;
    private final CoralEndEffectorMechanism coralIntake;

    private Double coralIntakeTime;

    @Inject
    public DriverFeedbackManager(
        IDriver driver,
        IRobotProvider provider,
        ITimer timer,
        PowerManager powerMan,
        CoralEndEffectorMechanism coralIntake)
    {
        this.driver = driver;
        this.timer = timer;

        this.ds = provider.getDriverStation();
        this.powerMan = powerMan;
        this.coralIntake = coralIntake;

        this.coralIntakeTime = null;
    }

    @Override
    public void readSensors()
    {
    }

    @Override
    public void update(RobotMode mode)
    {
        RumbleMode rumbleMode = RumbleMode.None;
        if (mode == RobotMode.Teleop)
        {
            double currTime = this.timer.get();
            if (this.coralIntakeTime == null &&
                this.coralIntake.hasGamePiece())
            {
                this.coralIntakeTime = currTime;

                rumbleMode = RumbleMode.Coral;
            }
            else if (this.coralIntakeTime != null)
            {
                if (this.coralIntake.hasGamePiece())
                {
                    if (currTime - this.coralIntakeTime <= TuningConstants.CORAL_INTAKE_RUMBLE_TIME)
                    {
                        rumbleMode = RumbleMode.Coral;
                    }
                }
                else
                {
                    this.coralIntakeTime = null;
                }
            }

            if (this.ds.isFMSMode() &&
                Helpers.withinRange(this.ds.getMatchTime(), TuningConstants.ENDGAME_RUMBLE - 3.0, TuningConstants.ENDGAME_RUMBLE))
            {
                this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Right, 0.25);
                rumbleMode = RumbleMode.Endgame;
            }
        }

        if (mode != RobotMode.Autonomous &&
            this.driver.getDigital(DigitalOperation.ForceLightDriverRumble))
        {
            rumbleMode = RumbleMode.Force;
        }

        switch (rumbleMode)
        {
            case Coral:
                this.driver.setRumble(UserInputDevice.Codriver, JoystickRumbleType.Left, 0.25);
                this.driver.setRumble(UserInputDevice.Codriver, JoystickRumbleType.Right, 0.25);
                this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Left, 0.25);
                this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Right, 0.25);
                break;
            case Force:
                this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Left, 0.25);
                this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Right, 0.25);
                this.driver.setRumble(UserInputDevice.Codriver, JoystickRumbleType.Left, 0.0);
                this.driver.setRumble(UserInputDevice.Codriver, JoystickRumbleType.Right, 0.0);
                break;
            case Endgame:
                this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Left, 0.25);
                this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Right, 0.0);
                this.driver.setRumble(UserInputDevice.Codriver, JoystickRumbleType.Left, 0.0);
                this.driver.setRumble(UserInputDevice.Codriver, JoystickRumbleType.Right, 0.0);
                break;
            default:
            case None:
                this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Left, 0.0);
                this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Right, 0.0);
                this.driver.setRumble(UserInputDevice.Codriver, JoystickRumbleType.Left, 0.0);
                this.driver.setRumble(UserInputDevice.Codriver, JoystickRumbleType.Right, 0.0);
                break;
        }
    }

    @Override
    public void stop()
    {
        this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Left, 0.0);
        this.driver.setRumble(UserInputDevice.Driver, JoystickRumbleType.Right, 0.0);
        this.driver.setRumble(UserInputDevice.Codriver, JoystickRumbleType.Left, 0.0);
        this.driver.setRumble(UserInputDevice.Codriver, JoystickRumbleType.Right, 0.0);

        this.coralIntakeTime = null;
    }
}
