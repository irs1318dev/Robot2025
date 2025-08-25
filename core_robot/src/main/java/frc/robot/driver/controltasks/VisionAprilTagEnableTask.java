package frc.robot.driver.controltasks;

import frc.lib.helpers.ExceptionHelpers;
import frc.robot.TuningConstants;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.OffboardVisionManager;

public class VisionAprilTagEnableTask extends TimedTask
{
    private DigitalOperation visionOperation;

    /**
     * Initializes an instance of the VisionAprilTagEnableTask class
     * @param visionOperation the vision operation to use to find the tag
     * @param time to execute for (timeout)
     */
    public VisionAprilTagEnableTask(DigitalOperation visionOperation, double time)
    {
        super(time);

        if (TuningConstants.THROW_EXCEPTIONS)
        {
            // if we are cool with throwing exceptions (testing), check if toPerform is in
            // the possibleOperations set and throw an exception if it is not
            boolean containsToPerform = false;
            for (DigitalOperation op : OffboardVisionManager.PossibleVisionOperations)
            {
                if (op == visionOperation)
                {
                    containsToPerform = true;
                    break;
                }
            }

            ExceptionHelpers.Assert(containsToPerform, visionOperation.toString() + " not contained in the set of possible vision operations");
        }

        this.visionOperation = visionOperation;
    }

    @Override
    public void begin()
    {
        super.begin();

        for (DigitalOperation op : OffboardVisionManager.PossibleVisionOperations)
        {
            this.setDigitalOperationState(op, op == this.visionOperation);
        }
    }

    @Override
    public void update()
    {
        for (DigitalOperation op : OffboardVisionManager.PossibleVisionOperations)
        {
            this.setDigitalOperationState(op, op == this.visionOperation);
        }
    }

    @Override
    public void end()
    {
        super.end();

        for (DigitalOperation op : OffboardVisionManager.PossibleVisionOperations)
        {
            this.setDigitalOperationState(op, false);
        }
    }
}