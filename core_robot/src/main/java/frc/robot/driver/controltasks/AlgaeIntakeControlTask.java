package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.AlgaeIntakeMechanism;

public class AlgaeIntakeControlTask extends CompositeOperationTask
{
    public enum AlgaeResult
    {
        Intake,
        Outtake,
        Launch,
    }

    private static final DigitalOperation[] possibleOperations = 
        new DigitalOperation[]
        {
            DigitalOperation.AlgaeIntake,
            DigitalOperation.AlgaeOuttake,
            DigitalOperation.AlgaeLaunch,
        };

    private final AlgaeResult desiredResult;

    private AlgaeIntakeMechanism endEffector;

    public AlgaeIntakeControlTask(AlgaeResult desiredResult)
    {
        super(
            desiredResult == AlgaeResult.Intake ? DigitalOperation.AlgaeIntake : (desiredResult == AlgaeResult.Outtake ? DigitalOperation.AlgaeOuttake : DigitalOperation.AlgaeLaunch),
            AlgaeIntakeControlTask.possibleOperations,
            false);

        this.desiredResult = desiredResult;
    }

    public AlgaeIntakeControlTask(AlgaeResult desiredResult, double timeout)
    {
        super(
            desiredResult == AlgaeResult.Intake ? DigitalOperation.AlgaeIntake : (desiredResult == AlgaeResult.Outtake ? DigitalOperation.AlgaeOuttake : DigitalOperation.AlgaeLaunch),
            AlgaeIntakeControlTask.possibleOperations,
            timeout);

        this.desiredResult = desiredResult;
    }

    public AlgaeIntakeControlTask(AlgaeResult desiredResult, boolean runIndefinitely)
    {
        super(
            desiredResult == AlgaeResult.Intake ? DigitalOperation.AlgaeIntake : (desiredResult == AlgaeResult.Outtake ? DigitalOperation.AlgaeOuttake : DigitalOperation.AlgaeLaunch),
            AlgaeIntakeControlTask.possibleOperations,
            runIndefinitely);

        this.desiredResult = desiredResult;
    }

    @Override
    public void begin()
    {
        super.begin();

        this.endEffector = this.getInjector().getInstance(AlgaeIntakeMechanism.class);
    }

    @Override
    public boolean hasCompleted()
    {
        if (this.runIndefinitely)
        {
            return false;
        }

        if (this.timeoutMode)
        {
            return super.hasCompleted();
        }

        switch (this.desiredResult)
        {
            case Intake:
                return this.endEffector.hasGamePiece();

            case Outtake:
            case Launch:
                return !this.endEffector.hasGamePiece();

            default:
                return super.hasCompleted();
        }
    }
}