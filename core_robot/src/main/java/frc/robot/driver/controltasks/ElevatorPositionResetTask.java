package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.mechanisms.ElevatorMechanism;

public class ElevatorPositionResetTask extends ControlTaskBase
{
    public ElevatorMechanism elevator; 

    public ElevatorPositionResetTask()
    {
    }

    @Override
    public void begin()
    {
        this.elevator = this.getInjector().getInstance(ElevatorMechanism.class);
    }

    @Override
    public void update()
    {
        this.setAnalogOperationState(AnalogOperation.ElevatorPower, TuningConstants.ELEVATOR_SLOW_POWER);
    }
    
    @Override
    public void end()
    {
        this.setAnalogOperationState(AnalogOperation.ElevatorPower, 0.0);
    }

    @Override
    public boolean hasCompleted() 
    {
        return this.elevator.getBottomLimitSwitch();
    }
}