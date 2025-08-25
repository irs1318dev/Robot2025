package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.ClimberMechanism;

public class ClimberWinchTask extends ControlTaskBase
{
    public enum ClimbMode
    {
        Normal,
        Extra,
        Destroy
    } 

    private final ClimbMode climbMode;

    private ClimberMechanism climberMechanism;

    public ClimberWinchTask()
    {
        this(ClimbMode.Normal);
    }

    public ClimberWinchTask(ClimbMode climbMode)
    {
        this.climbMode = climbMode;
    }

    @Override
    public void begin()
    {
        this.climberMechanism = this.getInjector().getInstance(ClimberMechanism.class);

        this.setDigitalOperationState(DigitalOperation.ClimberElbowEnableSimpleMode, true);
        this.setAnalogOperationState(AnalogOperation.ClimberElbowPower, TuningConstants.CLIMBER_ELBOW_WINCHING_POWER);
        this.setDigitalOperationState(DigitalOperation.ClimberWinchClimb, true);
    }

    @Override
    public void update()
    {
        this.setDigitalOperationState(DigitalOperation.ClimberElbowEnableSimpleMode, true);
        this.setAnalogOperationState(AnalogOperation.ClimberElbowPower, TuningConstants.CLIMBER_ELBOW_WINCHING_POWER);
        this.setDigitalOperationState(DigitalOperation.ClimberWinchClimb, true);
    }

    @Override
    public void end()
    {
        this.setDigitalOperationState(DigitalOperation.ClimberElbowEnableSimpleMode, false);
        this.setAnalogOperationState(AnalogOperation.ClimberElbowPower, TuningConstants.CLIMBER_ELBOW_WINCHING_POWER);
        this.setDigitalOperationState(DigitalOperation.ClimberWinchClimb, false);
    }
    
    @Override
    public boolean hasCompleted()
    {
        if (this.climbMode == ClimbMode.Destroy)
        {
            return false;
        }

        Double elbowAngle = this.climberMechanism.getAngle();
        if (elbowAngle == null)
        {
            return false;
        }

        if (this.climbMode == ClimbMode.Extra)
        {
            return this.climberMechanism.getLimitSwitchStatus() || elbowAngle <= TuningConstants.CLIMBER_ELBOW_SUPER_DOWN_ANGLE;
        }

        return this.climberMechanism.getLimitSwitchStatus() || elbowAngle <= TuningConstants.CLIMBER_ELBOW_DOWN_ANGLE;
    }
}