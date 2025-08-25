package frc.robot.driver.controltasks;

import java.util.function.Function;

import frc.lib.driver.IControlTask;
import frc.lib.driver.TrajectoryManager;
import frc.lib.helpers.ExceptionHelpers;
import frc.lib.helpers.Helpers;
import frc.lib.robotprovider.IPathPlanner;
import frc.lib.robotprovider.IRobotProvider;
import frc.lib.robotprovider.ITrajectory;
import frc.lib.robotprovider.PathPlannerWaypoint;
import frc.robot.TuningConstants;
import frc.robot.driver.DigitalOperation;
import frc.robot.driver.PathPlannerTrajectoryGenerator;
import frc.robot.driver.controltasks.FollowPathTask.Type;
import frc.robot.mechanisms.OffboardVisionManager;
import frc.robot.mechanisms.PigeonManager;

public class VisionApproachAprilTagTask extends DecisionSequentialTask
{
    public enum ApproachType
    {
        Forward,
        Direct,
        Backward
    }

    private static final String PATH_NAME = "ApproachAprilTagTaskPath";

    private final double xOffset;
    private final double yOffset;
    private final double yawOffset;
    private final ApproachType approachType;
    private final Function<Double, IControlTask> creator;

    private OffboardVisionManager vision;
    private IRobotProvider provider;
    private TrajectoryManager trajectoryManager;

    private DigitalOperation visionOperation;

    private int noAprilTags;

    private enum State
    {
        PreReadWait,
        ReadAprilTag,
        ApproachAprilTag
    }

    private State state;

    /**
     * Initializes an instance of the ApproachAprilTagTask class
     * @param xOffset the distance the robot should end up in front of the tag
     * @param yOffset the distance the robot should end up to the left of the tag
     * @param visionOperation the vision operation to use to find the tag
     */
    public VisionApproachAprilTagTask(double xOffset, double yOffset, DigitalOperation visionOperation)
    {
        this(xOffset, yOffset, 0.0, ApproachType.Forward, visionOperation, null);
    }

    /**
     * Initializes an instance of the ApproachAprilTagTask class
     * @param xOffset the distance the robot should end up in front of the tag
     * @param yOffset the distance the robot should end up to the left of the tag
     * @param visionOperation the vision operation to use to find the tag
     * @param creator creates a task to preform given the duration
     */
    public VisionApproachAprilTagTask(double xOffset, double yOffset, DigitalOperation visionOperation, Function<Double, IControlTask> creator)
    {
        this(xOffset, yOffset, 0.0, ApproachType.Forward, visionOperation, creator);
    }

    /**
     * Initializes an instance of the ApproachAprilTagTask class
     * @param xOffset the distance the robot should end up in front of the tag
     * @param yOffset the distance the robot should end up to the left of the tag
     * @param yawOffset the desired yaw offset from the tag
     * @param approachType the desired direction of travel when approaching the final destination
     * @param visionOperation the vision operation to use to find the tag
     * @param creator creates a task to preform given the duration
     */
    public VisionApproachAprilTagTask(double xOffset, double yOffset, double yawOffset, ApproachType approachType, DigitalOperation visionOperation, Function<Double, IControlTask> creator)
    {
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

        this.xOffset = xOffset;
        this.yOffset = yOffset;
        this.yawOffset = yawOffset;
        this.approachType = approachType;
        this.visionOperation = visionOperation;
        this.creator = creator;
    }

    @Override
    public void begin()
    {
        super.begin();

        PigeonManager pigeonManager = this.getInjector().getInstance(PigeonManager.class);
        double currentYaw = pigeonManager.getYaw() + (pigeonManager.getAllianceSwapForward() ? 180.0 : 0.0);
        currentYaw = Helpers.updateAngleRange360(currentYaw);

        this.vision = this.getInjector().getInstance(OffboardVisionManager.class);     
        this.provider = this.getInjector().getInstance(IRobotProvider.class);
        this.trajectoryManager = this.getInjector().getInstance(TrajectoryManager.class);

        if (TuningConstants.VISION_USE_PRECEDENCE && this.visionOperation == DigitalOperation.VisionFindReefTagsOnly)
        {
            if (currentYaw >= 330.0 || currentYaw <= 30.0)
            {
                this.visionOperation = DigitalOperation.VisionFindReefCenterCloseTagOnly;
            }
            else if (currentYaw > 270.0)
            {  
                this.visionOperation = DigitalOperation.VisionFindReefLeftCloseTagOnly;
            }
            else if (currentYaw > 210.0)
            {
                this.visionOperation = DigitalOperation.VisionFindReefLeftFarTagOnly;
            }
            else if (currentYaw > 150.0)
            {
                this.visionOperation = DigitalOperation.VisionFindReefCenterFarTagOnly;
            }
            else if (currentYaw > 90.0)
            {
                this.visionOperation = DigitalOperation.VisionFindReefRightFarTagOnly;
            }
            else if (currentYaw > 30.0)
            {
                this.visionOperation = DigitalOperation.VisionFindReefRightCloseTagOnly;
            }
        }

        for (DigitalOperation op : OffboardVisionManager.PossibleVisionOperations)
        {
            this.setDigitalOperationState(op, op == this.visionOperation);
        }

        this.state = State.PreReadWait;
    }

    @Override
    public void update()
    {
        if (this.state == State.PreReadWait)
        {
            // run at least one update cycle without trying to ask vision system for tag id to ensure that they are being properly filtered
            for (DigitalOperation op : OffboardVisionManager.PossibleVisionOperations)
            {
                this.setDigitalOperationState(op, op == this.visionOperation);
            }

            this.state = State.ReadAprilTag;
        }
        else if (this.state == State.ReadAprilTag)
        {
            for (DigitalOperation op : OffboardVisionManager.PossibleVisionOperations)
            {
                this.setDigitalOperationState(op, op == this.visionOperation);
            }

            if (this.vision.getAprilTagId() != null)
            {
                IPathPlanner pathPlanner = this.provider.getPathPlanner();
                PathPlannerTrajectoryGenerator.configureRobot(pathPlanner);

                double tagXOffset = vision.getAprilTagXOffset();
                double tagYOffset = vision.getAprilTagYOffset();
                double tagYawOffset = vision.getAprilTagYaw();

                double xGoal = tagXOffset - Helpers.cosd(tagYawOffset) * this.xOffset - Helpers.sind(tagYawOffset) * this.yOffset;
                double yGoal = tagYOffset + Helpers.cosd(tagYawOffset) * this.yOffset - Helpers.sind(tagYawOffset) * this.xOffset;
                double yawGoal = tagYawOffset + this.yawOffset;

                double tangent = Helpers.atan2d(yGoal, xGoal);

                double finalHeading;
                switch (this.approachType)
                {
                    case Direct:
                        finalHeading = tangent;
                        break;

                    case Backward:
                        finalHeading = -yawGoal;
                        break;
                    
                    case Forward:
                    default:
                        finalHeading = yawGoal;
                        break;
                }

                // generate the path
                ITrajectory approachTrajectory =
                    pathPlanner.buildTrajectory(
                        TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                        TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                        TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                        TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                        new PathPlannerWaypoint(0, 0, tangent, 0),
                        new PathPlannerWaypoint(xGoal, yGoal, finalHeading, yawGoal));

                this.trajectoryManager.addTrajectory(VisionApproachAprilTagTask.PATH_NAME, approachTrajectory);

                IControlTask taskToPreform = new FollowPathTask(VisionApproachAprilTagTask.PATH_NAME, Type.RobotRelativeFromCurrentPose);
                if (this.creator != null)
                {
                    taskToPreform = ConcurrentTask.AllTasks(taskToPreform, this.creator.apply(approachTrajectory.getDuration()));
                }

                this.AppendTask(taskToPreform);
                this.state = State.ApproachAprilTag;
            }
            else
            {
                this.noAprilTags++;
            }
        }
        else // if (this.state == State.ApproachAprilTag)
        {
            super.update();
        }
    }

    @Override
    public void end()
    {
        for (DigitalOperation op : OffboardVisionManager.PossibleVisionOperations)
        {
            this.setDigitalOperationState(op, false);
        }

        if (this.state == State.ApproachAprilTag)
        {
            super.end();
        }
    }

    @Override
    public boolean hasCompleted()
    {
        if (this.state != State.ApproachAprilTag)
        {
            return false;
        }

        return super.hasCompleted();
    }

    @Override
    public boolean shouldCancel()
    {
        if (this.state == State.ReadAprilTag)
        {
            return this.noAprilTags > TuningConstants.TAGS_MISSED_THRESHOLD;
        }

        return super.shouldCancel();
    }
}