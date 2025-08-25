package frc.robot.driver;

import java.util.ArrayList;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import frc.lib.driver.TrajectoryManager;
import frc.lib.helpers.Helpers;
import frc.lib.robotprovider.IPathPlanner;
import frc.lib.robotprovider.IPathPlannerGoal;
import frc.lib.robotprovider.ITrajectory;
import frc.lib.robotprovider.MotorType;
import frc.lib.robotprovider.PathPlannerRotationTarget;
import frc.lib.robotprovider.PathPlannerWaypoint;
import frc.lib.robotprovider.Point2d;
import frc.lib.robotprovider.TrajectoryState;
import frc.robot.FieldConstants;

public class PathPlannerTrajectoryTests
{
    private static final double MINOR_BUFFER = 12.0;

    private static final double MIN_ALLOWED_X_POSITION_BLUE = -0.5 * FieldConstants.FIELD_WIDTH - MINOR_BUFFER;
    private static final double MAX_ALLOWED_X_POSITION_BLUE = 0.0;

    private static final double MIN_ALLOWED_X_POSITION_RED = 0.0;
    private static final double MAX_ALLOWED_X_POSITION_RED = 0.5 * FieldConstants.FIELD_WIDTH + MINOR_BUFFER;

    private static final double MIN_ALLOWED_Y_POSITION = -0.5 * FieldConstants.FIELD_LENGTH - MINOR_BUFFER;
    private static final double MAX_ALLOWED_Y_POSITION = 0.5 * FieldConstants.FIELD_LENGTH + MINOR_BUFFER;

    @Test
    public void verifyTrajectoryGenerationConstraints()
    {
        TrajectoryManager trajectoryManager = new TrajectoryManager();
        PathPlannerTrajectoryGenerator.generateTrajectories(
            trajectoryManager,
            new PathPlannerVerifier());

        // check that the trajectory points are within the allowed bounds
        for (String name : trajectoryManager.getNames())
        {
            boolean isRed;
            if (name.endsWith("Red"))
            {
                isRed = true;
            }
            else if (name.endsWith("Blue"))
            {
                isRed = false;
            }
            else
            {
                continue;
            }

            PathPlannerVerifier.DummyTrajectory dummyTrajectory = (PathPlannerVerifier.DummyTrajectory)trajectoryManager.getTrajectory(name);
            for (Point2d point : dummyTrajectory.points)
            {
                if (isRed)
                {
                    Assertions.assertTrue(point.x >= MIN_ALLOWED_X_POSITION_RED && point.x <= MAX_ALLOWED_X_POSITION_RED, "x position out of bounds for red: " + point.x + " in trajectory " + name);
                }
                else
                {
                    Assertions.assertTrue(point.x >= MIN_ALLOWED_X_POSITION_BLUE && point.x <= MAX_ALLOWED_X_POSITION_BLUE, "x position out of bounds for blue: " + point.x + " in trajectory " + name);
                }

                Assertions.assertTrue(point.y >= MIN_ALLOWED_Y_POSITION && point.y <= MAX_ALLOWED_Y_POSITION, "y position out of bounds: " + point.y + " in trajectory " + name);
            }
        }
    }

    private class PathPlannerVerifier implements IPathPlanner
    {
        private boolean configured = false;

        @Override
        public boolean isConfigured()
        {
            return this.configured;
        }

        @Override
        public void configureRobot(
            double robotWeight,
            double robotMomentOfInertia,
            double swerveModuleWheelRadius,
            double swerveModuleMaxVelocity,
            double swerveModuleWheelCoefficientOfFriction,
            double swerveDriveGearReduction,
            MotorType swerveDriveMotorType,
            int swerveDriveMotorCount,
            double swerveDriveMotorCurrentLimit,
            double horizontalModuleCenterDistance,
            double verticalModuleCenterDistance)
        {
            this.configured = true;
        }

        @Override
        public ITrajectory buildTrajectory(
            double maxVelocity,
            double maxAcceleration,
            double maxAngularVelocity,
            double maxAngularAcceleration,
            IPathPlannerGoal... goalPoints)
        {
            Assertions.assertTrue(maxVelocity > 0.0, "maxVelocity must be positive");
            Assertions.assertTrue(maxAcceleration > 0.0, "maxAcceleration must be positive");
            Assertions.assertTrue(maxAngularVelocity > 0.0, "maxAngularVelocity must be positive");
            Assertions.assertTrue(maxAngularAcceleration > 0.0, "maxAngularAcceleration must be positive");

            Assertions.assertNotNull(goalPoints, "goalPoints must be provided");
            Assertions.assertTrue(goalPoints.length >= 2, "at least 2 goalPoints must be provided");

            ArrayList<Point2d> points = new ArrayList<Point2d>();

            double lastPercentage = 0.0;
            Double prevX = null;
            Double prevY = null;
            for (int i = 0; i < goalPoints.length; i++)
            {
                IPathPlannerGoal goalPoint = goalPoints[i];
                Assertions.assertNotNull(goalPoint, "goal point at index " + i + " must be non-null");
                if (goalPoint instanceof PathPlannerRotationTarget)
                {
                    PathPlannerRotationTarget rotationTarget = (PathPlannerRotationTarget)goalPoint;
                    Assertions.assertTrue(rotationTarget.percentage > 0.0 && rotationTarget.percentage < 1.0, "rotation target percentage must be between 0 and 1, non-inclusive.  Actual: " + rotationTarget.percentage);
                    Assertions.assertTrue(rotationTarget.percentage >= lastPercentage, "rotation target percentage must be increasing");
                    lastPercentage = rotationTarget.percentage;
                }
                else if (goalPoint instanceof PathPlannerWaypoint)
                {
                    lastPercentage = 0.0;

                    PathPlannerWaypoint waypoint = (PathPlannerWaypoint)goalPoint;
                    if (prevX != null && prevY != null)
                    {
                        Assertions.assertTrue(
                            !Helpers.roughEquals(prevX, waypoint.x) || !Helpers.roughEquals(prevY, waypoint.y), 
                            "Don't expect to see the same point two times in a row: (" + prevX + ", " + prevY + ")");
                    }

                    points.add(new Point2d(waypoint.x, waypoint.y));

                    prevX = waypoint.x;
                    prevY = waypoint.y;
                }
                else
                {
                    Assertions.fail("Unknown type for goalpoint " + i + ": " + goalPoint.getClass().getName());
                }
            }

            Assertions.assertInstanceOf(PathPlannerWaypoint.class, goalPoints[0], "the first goal point must be a PathPlannerWaypoint");
            PathPlannerWaypoint firstWaypoint = (PathPlannerWaypoint)goalPoints[0];
            Assertions.assertTrue(firstWaypoint.orientation.isPresent(), "the first waypoint must have an orientation");

            Assertions.assertInstanceOf(PathPlannerWaypoint.class, goalPoints[goalPoints.length - 1], "the last goal point must be a PathPlannerWaypoint");
            PathPlannerWaypoint lastWaypoint = (PathPlannerWaypoint)goalPoints[goalPoints.length - 1];
            Assertions.assertTrue(lastWaypoint.orientation.isPresent(), "the lasst waypoint must have an orientation");

            return new DummyTrajectory(points);
        }

        @Override
        public ITrajectory loadTrajectory(String name)
        {
            return new DummyTrajectory(null);
        }

        @Override
        public ITrajectory loadTrajectory(String name, boolean reversed)
        {
            return new DummyTrajectory(null);
        }

        static class DummyTrajectory implements ITrajectory
        {
            final ArrayList<Point2d> points;

            public DummyTrajectory(ArrayList<Point2d> points)
            {
                this.points = points;
            }

            @Override
            public double getDuration()
            {
                return 0.0;
            }

            @Override
            public TrajectoryState get(double time)
            {
                return null;
            }
        }
    }
}