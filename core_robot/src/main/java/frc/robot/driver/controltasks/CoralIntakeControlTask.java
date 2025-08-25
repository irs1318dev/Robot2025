package frc.robot.driver.controltasks;

import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.CoralEndEffectorMechanism;
import frc.robot.mechanisms.ElevatorMechanism;

public class CoralIntakeControlTask extends CompositeOperationTask
{
    public enum CoralResult
    {
        Intake,
        IntakeSlow,
        Place,
        PlaceSlow,
        Reverse,
        ReverseSlow,
    }

    private static final DigitalOperation[] possibleOperations = 
        new DigitalOperation[]
        {
            DigitalOperation.CoralIntake,
            DigitalOperation.CoralIntakeSlow,
            DigitalOperation.CoralPlace,
            DigitalOperation.CoralPlaceSlow,
            DigitalOperation.CoralReverse,
            DigitalOperation.CoralReverseSlow,
        };

    private final CoralResult desiredResult;

    private CoralEndEffectorMechanism endEffector;

    public CoralIntakeControlTask(CoralResult desiredResult)
    {
        super(
            determineOperation(desiredResult),
            CoralIntakeControlTask.possibleOperations,
            true);

        this.desiredResult = desiredResult;
    }

    public CoralIntakeControlTask(CoralResult desiredResult, double timeout)
    {
        super(
            determineOperation(desiredResult),
            CoralIntakeControlTask.possibleOperations,
            timeout);

        this.desiredResult = desiredResult;
    }

    private static DigitalOperation determineOperation(CoralResult result)
    {
        switch(result)
        {
            case Intake:
                return DigitalOperation.CoralIntake;

            case IntakeSlow:
                return DigitalOperation.CoralIntakeSlow;

            case Place:
                return DigitalOperation.CoralPlace;

            case PlaceSlow:
                return DigitalOperation.CoralPlaceSlow;

            case Reverse:
                return DigitalOperation.CoralReverse;

            default:
            case ReverseSlow:
                return DigitalOperation.CoralReverseSlow;
        }
    }

    @Override
    public void begin()
    {
        super.begin();

        this.endEffector = this.getInjector().getInstance(CoralEndEffectorMechanism.class);
    }

    @Override
    public boolean hasCompleted()
    {
        switch (this.desiredResult)
        {
            case Intake:
            case IntakeSlow:
                if (this.endEffector.hasGamePiece())
                {
                    return true;
                }

                break;

            case Place:
            case PlaceSlow:
                if (!this.endEffector.hasGamePiece())
                {
                    return true;
                }

                break;

            case ReverseSlow:
                if (!this.endEffector.hasGamePiece())
                {
                    return true;
                }

                break;

            case Reverse:
                return false;
        }

        return super.hasCompleted();
    }
}