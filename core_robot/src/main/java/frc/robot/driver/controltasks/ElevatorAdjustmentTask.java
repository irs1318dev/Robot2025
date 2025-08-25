package frc.robot.driver.controltasks;


import frc.robot.driver.AnalogOperation;

public class ElevatorAdjustmentTask extends ControlTaskBase
{
    private final double lifterAdjustmentPosition;

    public ElevatorAdjustmentTask(double lifterAdjustmentPosition)
    {
        this.lifterAdjustmentPosition = lifterAdjustmentPosition;
    }


    @Override
    public void begin() {
        this.setAnalogOperationState(AnalogOperation.ElevatorPositionAdjustment, this.lifterAdjustmentPosition);
    }

    @Override
    public void update() {
        this.setAnalogOperationState(AnalogOperation.ElevatorPositionAdjustment, this.lifterAdjustmentPosition);
    }

    @Override
    public void end() {
        this.setAnalogOperationState(AnalogOperation.ElevatorPositionAdjustment, this.lifterAdjustmentPosition);

    }

    @Override
    public boolean hasCompleted() {
        return false;
    }
    
}