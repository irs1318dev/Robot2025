package frc.robot.driver;

import java.util.Optional;

import frc.lib.driver.TrajectoryManager;
import frc.lib.helpers.ExceptionHelpers;
import frc.lib.robotprovider.Alliance;
import frc.lib.robotprovider.IPathPlanner;
import frc.lib.robotprovider.ITrajectory;
import frc.lib.robotprovider.PathPlannerWaypoint;
import frc.robot.AutonLocManager;
import frc.robot.HardwareConstants;
import frc.robot.TuningConstants;

public class PathPlannerTrajectoryGenerator
{
    public static void configureRobot(IPathPlanner pathPlanner)
    {
        if (!pathPlanner.isConfigured())
        {
            pathPlanner.configureRobot(
                HardwareConstants.ROBOT_WEIGHT,
                HardwareConstants.ROBOT_MOI,
                HardwareConstants.SDSDRIVETRAIN_DRIVE_WHEEL_DIAMETER / 2.0,
                TuningConstants.SDSDRIVETRAIN_MAX_MODULE_VELOCITY,
                HardwareConstants.SDSDRIVETRAIN_WHEEL_COEFFICIENT_OF_FRICTION,
                HardwareConstants.SDSDRIVETRAIN_DRIVE_GEAR_RATIO,
                HardwareConstants.SDSDRIVETRAIN_DRIVE_MOTOR_TYPE,
                HardwareConstants.SDSDRIVETRAIN_DRIVE_MOTOR_COUNT,
                TuningConstants.SDSDRIVETRAIN_DRIVE_STATOR_CURRENT_LIMIT,
                HardwareConstants.SDSDRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE,
                HardwareConstants.SDSDRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE);
        }
    }

    public static void generateTrajectories(TrajectoryManager trajectoryManager, IPathPlanner pathPlanner)
    {
        PathPlannerTrajectoryGenerator.configureRobot(pathPlanner);

        PathPlannerTrajectoryGenerator.generateTrajectories(false, trajectoryManager, pathPlanner);
        PathPlannerTrajectoryGenerator.generateTrajectories(true, trajectoryManager, pathPlanner);

        // ------------------------------- Macro paths --------------------------------------------
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 0.0, 0.0),
                new PathPlannerWaypoint(15.0, 0.0, 0.0, 0.0),
                new PathPlannerWaypoint(30.0, 0.0, 0.0, 0.0)),
            "goForwards30in",
            Optional.empty());
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 0.0, 0.0),
                new PathPlannerWaypoint(-18.0, 0.0, 0.0, 0.0)),
            "goBackwards18in",
            Optional.empty());
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 0.0, 0.0),
                new PathPlannerWaypoint(30.0, 0.0, 0.0, 0.0),
                new PathPlannerWaypoint(45.0, 0.0, 0.0, 0.0)),
            "goForwards45in",
            Optional.empty());
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 0.0, 0.0),
                new PathPlannerWaypoint(68.0, 0.0, 0.0, 0.0)),
            "goForwards68in",
            Optional.empty());
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 0.0, 0.0),
                new PathPlannerWaypoint(20.0, 0.0, 0.0, 0.0)),
            "goForwards20in",
            Optional.empty());

            addTrajectory(
                trajectoryManager,
                pathPlanner.buildTrajectory(
                    TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                    TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                    TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                    TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                    new PathPlannerWaypoint(0.0, 0.0, 0.0, 0.0),
                    new PathPlannerWaypoint(4.0, 0.0, 0.0, 0.0)),
                "goForwards4in",
                Optional.empty());

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 0.0, 0.0),
                new PathPlannerWaypoint(20.0, 0.0, 0.0, 0.0)),
            "goForwards36in",
            Optional.empty());
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 0.0, 0.0),
                new PathPlannerWaypoint(20.0, 0.0, 0.0, 0.0)),
            "DriveForward20inTrap",
            Optional.empty());
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-36.0, 0.0, 180.0, 0.0)),
            "goBackwards36in",
            Optional.empty());

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-20.0, 0.0, 180.0, 0.0)),
            "goBackwards20in",
            Optional.empty());

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 180.0, 0.0),
                new PathPlannerWaypoint(-8.0, 0.0, 180.0, 0.0)),
            "goBackwards8in",
            Optional.empty());
            
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 90.0, 0.0),
                new PathPlannerWaypoint(9.0, 16.0, 0.0, 0.0),
                new PathPlannerWaypoint(18.0, 32.0, 0.0, 0.0)),
            "goLeft32inForward18in",
            Optional.empty());
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 270.0, 0.0),
                new PathPlannerWaypoint(9.0, -16.0, 0.0, 0.0),
                new PathPlannerWaypoint(18.0, -32.0, 0.0, 0.0)),
            "goRight32inForward18in",
            Optional.empty());
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 90.0, 0.0),
                new PathPlannerWaypoint(0.0, 11.0, 90.0, 0.0),
                new PathPlannerWaypoint(0.0, 22.0, 90.0, 0.0)),
            "goLeft22in",
            Optional.empty());
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(0.0, 0.0, 270.0, 0.0),
                new PathPlannerWaypoint(0.0, -11.0, 270.0, 0.0),
                new PathPlannerWaypoint(0.0, -22.0, 270.0, 0.0)),
            "goRight22in",
            Optional.empty());
    }

    public static void generateTrajectories(boolean isRed, TrajectoryManager trajectoryManager, IPathPlanner pathPlanner)
    {
        AutonLocManager locManager = new AutonLocManager(isRed);
        Optional<Alliance> alliance = isRed ? Optional.of(Alliance.Red) : Optional.of(Alliance.Blue);

        // ----------------------> EXAMPLE PATH <-----------------------------
        /*
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.P1, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.P2, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(180.0))),
            "ExamplePath",
            alliance);
        */

        // ----------------------> AUTON STAGE PATHING <-----------------------------
        // B1 to IS9
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B1, locManager.getOrientationOrHeading(270.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IB1, locManager.getOrientationOrHeading(270.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IG5, locManager.getOrientationOrHeading(270.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IS9, locManager.getOrientationOrHeading(300.0), locManager.getOrientationOrHeading(300.0))),
            "B1ToIS9",
            alliance);

        // S9 to CB
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.S9, locManager.getOrientationOrHeading(120.0), locManager.getOrientationOrHeading(300.0)),
                new PathPlannerWaypoint(locManager.CB, locManager.getOrientationOrHeading(126.0), locManager.getOrientationOrHeading(306.0))),
            "S9ToCB",
            alliance);
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_SUPER_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_SUPER_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.S9, locManager.getOrientationOrHeading(120.0), locManager.getOrientationOrHeading(300.0)),
                new PathPlannerWaypoint(locManager.CBF, locManager.getOrientationOrHeading(126.0), locManager.getOrientationOrHeading(306.0))),
            "S9ToCBFast",
            alliance);

        // S9 to CBR
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.S9, locManager.getOrientationOrHeading(120.0), locManager.getOrientationOrHeading(300.0)),
                new PathPlannerWaypoint(locManager.CBR, locManager.getOrientationOrHeading(126.0), locManager.getOrientationOrHeading(306.0))),
            "S9ToCBR",
            alliance);

        // CB to IS7
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_SUPER_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_SUPER_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.CB, locManager.getOrientationOrHeading(306.0), locManager.getOrientationOrHeading(306.0)),
                new PathPlannerWaypoint(locManager.ICB, locManager.getOrientationOrHeading(306.0), locManager.getOrientationOrHeading(306.0)),
                new PathPlannerWaypoint(locManager.IS7, locManager.getOrientationOrHeading(300.0), locManager.getOrientationOrHeading(300.0))),
            "CBToIS7",
            alliance);
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_SUPER_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_SUPER_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.CB, locManager.getOrientationOrHeading(306.0), locManager.getOrientationOrHeading(306.0)),
                new PathPlannerWaypoint(locManager.ICB, locManager.getOrientationOrHeading(306.0), locManager.getOrientationOrHeading(306.0)),
                new PathPlannerWaypoint(locManager.IS7, locManager.getOrientationOrHeading(300.0), locManager.getOrientationOrHeading(300.0))),
            "CBToIS7Fast",
            alliance);

        // S7 to CB
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.S7, locManager.getOrientationOrHeading(120.0), locManager.getOrientationOrHeading(300.0)),
                new PathPlannerWaypoint(locManager.ICB, locManager.getOrientationOrHeading(126.0), locManager.getOrientationOrHeading(306.0)),
                new PathPlannerWaypoint(locManager.CB, locManager.getOrientationOrHeading(126.0), locManager.getOrientationOrHeading(306.0))),
            "S7ToCB",
            alliance);

        // CB to IS10
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.CB, locManager.getOrientationOrHeading(306.0), locManager.getOrientationOrHeading(306.0)),
                new PathPlannerWaypoint(locManager.ICB, locManager.getOrientationOrHeading(306.0), locManager.getOrientationOrHeading(306.0)),
                new PathPlannerWaypoint(locManager.IPB, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(0.0)),
                new PathPlannerWaypoint(locManager.IS10, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(0.0))),
            "CBToIS10",
            alliance);

        // B2 to IS1
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B2, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IS1, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0))),
            "B2ToIS1",
            alliance);

        // B3 to IS15
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B3, locManager.getOrientationOrHeading(90.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IB3, locManager.getOrientationOrHeading(90.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IS15, locManager.getOrientationOrHeading(60.0), locManager.getOrientationOrHeading(60.0))),
            "B3ToIS15",
            alliance);

        // B3a to IS15
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B3a, locManager.getOrientationOrHeading(135.0), locManager.getOrientationOrHeading(135.0)),
                new PathPlannerWaypoint(locManager.IS15, locManager.getOrientationOrHeading(120.0), locManager.getOrientationOrHeading(120.0))),
            "B3aToIS15",
            alliance);

        // IS15 to S15
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.IS15, locManager.getOrientationOrHeading(60.0), locManager.getOrientationOrHeading(60.0)),
                new PathPlannerWaypoint(locManager.S15, locManager.getOrientationOrHeading(60.0), locManager.getOrientationOrHeading(60.0))),
            "IS15ToS15",
            alliance);

        // From S15 to CT
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.S15, locManager.getOrientationOrHeading(240.0), locManager.getOrientationOrHeading(60.0)),
                new PathPlannerWaypoint(locManager.CT, locManager.getOrientationOrHeading(234.0), locManager.getOrientationOrHeading(54.0))),
            "S15ToCT",
            alliance);

        // From CT to IS12
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.CT, locManager.getOrientationOrHeading(54.0), locManager.getOrientationOrHeading(54.0)),
                new PathPlannerWaypoint(locManager.IPT, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(0.0)),
                new PathPlannerWaypoint(locManager.IS12, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(0.0))),
            "CTToIS12",
            alliance);


        // From IS12 to S12
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.IS12, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(0.0)),
                new PathPlannerWaypoint(locManager.S12, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(0.0))),
            "IS12ToS12",
            alliance);

        // From B3 to IS18
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B3, locManager.getOrientationOrHeading(90.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IB3, locManager.getOrientationOrHeading(90.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IS18, locManager.getOrientationOrHeading(120.0), locManager.getOrientationOrHeading(120.0))),
            "B3ToIS18",
            alliance);

        // From S18 to CT
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.S18, locManager.getOrientationOrHeading(300.0), locManager.getOrientationOrHeading(120.0)),
                new PathPlannerWaypoint(locManager.IG2, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(54.0)),
                new PathPlannerWaypoint(locManager.CT, locManager.getOrientationOrHeading(234.0), locManager.getOrientationOrHeading(54.0))),
            "S18ToCT",
            alliance);

        // From B2 to IS3
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B2, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IS3, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0))),
            "B2ToIS3",
            alliance);

        // From S3 to CB
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.S3, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IS3, locManager.getOrientationOrHeading(90.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IG6, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(306.0)),
                new PathPlannerWaypoint(locManager.ICB, locManager.getOrientationOrHeading(126.0), locManager.getOrientationOrHeading(306.0)),
                new PathPlannerWaypoint(locManager.CB, locManager.getOrientationOrHeading(126.0), locManager.getOrientationOrHeading(306.0))),
            "S3ToCB",
            alliance);

        // From B3 to IS16
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B3, locManager.getOrientationOrHeading(90.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IB3, locManager.getOrientationOrHeading(90.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IS16, locManager.getOrientationOrHeading(120.0), locManager.getOrientationOrHeading(120.0))),
            "B3ToIS16",
            alliance);

        // From B3b to IS16
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_SUPER_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_SUPER_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B3b, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IS16, locManager.getOrientationOrHeading(120.0), locManager.getOrientationOrHeading(120.0))),
            "B3bToIS16",
            alliance);
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_SUPER_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_SUPER_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B3b, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IS16, locManager.getOrientationOrHeading(120.0), locManager.getOrientationOrHeading(120.0))),
            "B3bToIS16Fast",
            alliance);
    
        // From IS16 to S16
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.IS16, locManager.getOrientationOrHeading(120.0), locManager.getOrientationOrHeading(120.0)),
                new PathPlannerWaypoint(locManager.S16, locManager.getOrientationOrHeading(120.0), locManager.getOrientationOrHeading(120.0))),
            "IS16ToS16",
            alliance);

        // From S16 to CT
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.S16, locManager.getOrientationOrHeading(300.0), locManager.getOrientationOrHeading(120.0)),
                new PathPlannerWaypoint(locManager.IG2, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(54.0)),
                new PathPlannerWaypoint(locManager.CT, locManager.getOrientationOrHeading(234.0), locManager.getOrientationOrHeading(54.0))),
            "S16ToCT",
            alliance);
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_SUPER_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_SUPER_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.S16, locManager.getOrientationOrHeading(300.0), locManager.getOrientationOrHeading(120.0)),
                new PathPlannerWaypoint(locManager.IG2, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(54.0)),
                new PathPlannerWaypoint(locManager.CT, locManager.getOrientationOrHeading(234.0), locManager.getOrientationOrHeading(54.0))),
            "S16ToCTFast",
            alliance);

        // From CT to IS13
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_SUPER_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_SUPER_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.CT, locManager.getOrientationOrHeading(54.0), locManager.getOrientationOrHeading(54.0)),
                new PathPlannerWaypoint(locManager.IS13, locManager.getOrientationOrHeading(60.0), locManager.getOrientationOrHeading(60.0))),
            "CTToIS13",
            alliance);
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_SUPER_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_SUPER_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.CT, locManager.getOrientationOrHeading(54.0), locManager.getOrientationOrHeading(54.0)),
                new PathPlannerWaypoint(locManager.IS13, locManager.getOrientationOrHeading(60.0), locManager.getOrientationOrHeading(60.0))),
            "CTToIS13Fast",
            alliance);

         // From B2 to IS18
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B2, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IS18, locManager.getOrientationOrHeading(120.0), locManager.getOrientationOrHeading(120.0))),
            "B2ToIS18",
            alliance);

        // From IS18 to S18
         addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.IS18, locManager.getOrientationOrHeading(120.0), locManager.getOrientationOrHeading(120.0)),
                new PathPlannerWaypoint(locManager.S18, locManager.getOrientationOrHeading(120.0), locManager.getOrientationOrHeading(120.0))),
            "IS18ToS18",
            alliance);

        // From B2 to IS15
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B2, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IS18, locManager.getOrientationOrHeading(240.0), locManager.getOrientationOrHeading(60.0)),
                new PathPlannerWaypoint(locManager.IS15, locManager.getOrientationOrHeading(60.0), locManager.getOrientationOrHeading(60.0))),
            "B2ToIS15",
            alliance);
        
        // From S4 to CB
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.S4, locManager.getOrientationOrHeading(60.0), locManager.getOrientationOrHeading(240.0)),
                new PathPlannerWaypoint(locManager.IG5, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(306.0)),
                new PathPlannerWaypoint(locManager.ICB, locManager.getOrientationOrHeading(126.0), locManager.getOrientationOrHeading(306.0)),
                new PathPlannerWaypoint(locManager.CB, locManager.getOrientationOrHeading(126.0), locManager.getOrientationOrHeading(306.0))),
            "S4ToCB",
            alliance);

        // From CB to IS6
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.CB, locManager.getOrientationOrHeading(306.0), locManager.getOrientationOrHeading(306.0)),
                new PathPlannerWaypoint(locManager.ICB, locManager.getOrientationOrHeading(306.0), locManager.getOrientationOrHeading(306.0)),
                new PathPlannerWaypoint(locManager.IG5, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(240.0)),
                new PathPlannerWaypoint(locManager.IS6, locManager.getOrientationOrHeading(240.0), locManager.getOrientationOrHeading(240.0))),
            "CBToIS6",
            alliance);

        // From CT to IS1
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.CT, locManager.getOrientationOrHeading(54.0), locManager.getOrientationOrHeading(54.0)),
                new PathPlannerWaypoint(locManager.IG1, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IS1, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0))),
            "CTToIS1",
            alliance);

        // From B2 to IS12
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B2, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IG2, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IPT, locManager.getOrientationOrHeading(90.0), locManager.getOrientationOrHeading(0.0)),
                new PathPlannerWaypoint(locManager.IS12, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(0.0))),
            "B2ToIS12",
            alliance);

        // From B2 to IS10
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B2, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IG5, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IPB, locManager.getOrientationOrHeading(270.0), locManager.getOrientationOrHeading(0.0)),
                new PathPlannerWaypoint(locManager.IS10, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(0.0))),
            "B2ToIS10",
            alliance);

        // From IS10 to S10
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.IS10, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(0.0)),
                new PathPlannerWaypoint(locManager.S10, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(0.0))),
            "IS10ToS10",
            alliance);

        // From B1 to IS3
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B1, locManager.getOrientationOrHeading(270.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IB1, locManager.getOrientationOrHeading(270.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IS3, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0))),
            "B1ToIS3",
            alliance);

        // From B1 to IS4
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B1, locManager.getOrientationOrHeading(270.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IB1, locManager.getOrientationOrHeading(270.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IS4, locManager.getOrientationOrHeading(240.0), locManager.getOrientationOrHeading(240.0))),
            "B1ToIS4",
            alliance);

        // From B1 to IS6
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY, // TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION, // TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B1, locManager.getOrientationOrHeading(270.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IB1, locManager.getOrientationOrHeading(270.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IS6, locManager.getOrientationOrHeading(240.0), locManager.getOrientationOrHeading(240.0))),
            "B1ToIS6",
            alliance);

        // From B1b to IS6
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_SUPER_MAX_PATH_TRANSLATIONAL_VELOCITY, // TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_SUPER_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION, // TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B1b, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IS6, locManager.getOrientationOrHeading(240.0), locManager.getOrientationOrHeading(240.0))),
            "B1bToIS6",
            alliance);

        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_SUPER_MAX_PATH_TRANSLATIONAL_VELOCITY, // TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_SUPER_MAX_PATH_TRANSLATIONAL_ACCELERATION, // TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B1b, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IS6, locManager.getOrientationOrHeading(240.0), locManager.getOrientationOrHeading(240.0))),
            "B1bToIS6Fast",
            alliance);

        // From B1 to IS6
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B1a, locManager.getOrientationOrHeading(225.0), locManager.getOrientationOrHeading(225.0)),
                new PathPlannerWaypoint(locManager.IS6, locManager.getOrientationOrHeading(240.0), locManager.getOrientationOrHeading(240.0))),
            "B1aToIS6",
            alliance);

        // From B1 to IS7
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B1, locManager.getOrientationOrHeading(270.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IB1, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IG5, locManager.getOrientationOrHeading(225.0), locManager.getOrientationOrHeading(300.0)),
                new PathPlannerWaypoint(locManager.IS7, locManager.getOrientationOrHeading(300.0), locManager.getOrientationOrHeading(300.0))),
            "B1ToIS7",
            alliance);

        // From B1 to IS10
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B1, locManager.getOrientationOrHeading(270.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IB1, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IG5, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(0.0)),
                new PathPlannerWaypoint(locManager.IS10, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(0.0))),
            "B1ToIS10",
            alliance);

        // From B2 to IS4
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B2, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IS4, locManager.getOrientationOrHeading(240.0), locManager.getOrientationOrHeading(240.0))),
            "B2ToIS4",
            alliance);

        // From B2 to IS6
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B2, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IS6, locManager.getOrientationOrHeading(240.0), locManager.getOrientationOrHeading(240.0))),
            "B2ToIS6",
            alliance);

        // From B2 to IS7
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B2, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IG5, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(300.0)),
                new PathPlannerWaypoint(locManager.IS7, locManager.getOrientationOrHeading(300.0), locManager.getOrientationOrHeading(300.0))),
            "B2ToIS7",
            alliance);

        // From B2 to IS9
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B2, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IG5, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(300.0)),
                new PathPlannerWaypoint(locManager.IS9, locManager.getOrientationOrHeading(300.0), locManager.getOrientationOrHeading(300.0))),
            "B2ToIS9",
            alliance);

        // From B2 to S13
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B2, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IG2, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(60.0)),
                new PathPlannerWaypoint(locManager.IS13, locManager.getOrientationOrHeading(60.0), locManager.getOrientationOrHeading(60.0))),
            "B2ToIS13",
            alliance);

        // From B2 to IS15
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B2, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IG2, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(60.0)),
                new PathPlannerWaypoint(locManager.IS15, locManager.getOrientationOrHeading(60.0), locManager.getOrientationOrHeading(60.0))),
            "B2ToIS15",
            alliance);

        // From B2 to S16
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B2, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IS16, locManager.getOrientationOrHeading(120.0), locManager.getOrientationOrHeading(120.0))),
            "B2ToS16",
            alliance);

        // From B3 to IS1
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B3, locManager.getOrientationOrHeading(90.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IB3, locManager.getOrientationOrHeading(90.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IS1, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0))),
            "B3ToIS1",
            alliance);

        // From B3 to IS13
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B3, locManager.getOrientationOrHeading(90.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IB3, locManager.getOrientationOrHeading(90.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IG1, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(60.0)),
                new PathPlannerWaypoint(locManager.IS13, locManager.getOrientationOrHeading(60.0), locManager.getOrientationOrHeading(60.0))),
            "B3ToIS13",
            alliance);

        // From IS13 to S13
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.IS13, locManager.getOrientationOrHeading(60.0), locManager.getOrientationOrHeading(60.0)),
                new PathPlannerWaypoint(locManager.S13, locManager.getOrientationOrHeading(60.0), locManager.getOrientationOrHeading(60.0))),
            "IS13ToS13",
            alliance);

        // From B3 to IS12
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.B3, locManager.getOrientationOrHeading(90.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IB3, locManager.getOrientationOrHeading(90.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IG1, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IPT, locManager.getOrientationOrHeading(90.0), locManager.getOrientationOrHeading(0.0)),
                new PathPlannerWaypoint(locManager.IS12, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(0.0))),
            "B3ToIS12",
            alliance);

        // From CB to IS3
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.CB, locManager.getOrientationOrHeading(306.0), locManager.getOrientationOrHeading(306.0)),
                new PathPlannerWaypoint(locManager.ICB, locManager.getOrientationOrHeading(306.0), locManager.getOrientationOrHeading(306.0)),
                new PathPlannerWaypoint(locManager.IG6, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IS3, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0))),
            "CBToIS3",
            alliance);  

        // From CB to IS4
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.CB, locManager.getOrientationOrHeading(306.0), locManager.getOrientationOrHeading(306.0)),
                new PathPlannerWaypoint(locManager.ICB, locManager.getOrientationOrHeading(306.0), locManager.getOrientationOrHeading(306.0)),
                new PathPlannerWaypoint(locManager.IG6, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(240.0)),
                new PathPlannerWaypoint(locManager.IS4, locManager.getOrientationOrHeading(240.0), locManager.getOrientationOrHeading(240.0))),
            "CBToIS4",
            alliance);

        // From CB to IS9
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_SUPER_MAX_PATH_TRANSLATIONAL_VELOCITY, // TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_SUPER_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION, // TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.CB, locManager.getOrientationOrHeading(306.0), locManager.getOrientationOrHeading(306.0)),
                new PathPlannerWaypoint(locManager.ICB, locManager.getOrientationOrHeading(306.0), locManager.getOrientationOrHeading(306.0)),
                new PathPlannerWaypoint(locManager.IS9, locManager.getOrientationOrHeading(300.0), locManager.getOrientationOrHeading(300.0))),
            "CBToIS9",
            alliance);
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_SUPER_MAX_PATH_TRANSLATIONAL_VELOCITY, // TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_SUPER_MAX_PATH_TRANSLATIONAL_ACCELERATION, // TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.CB, locManager.getOrientationOrHeading(306.0), locManager.getOrientationOrHeading(306.0)),
                new PathPlannerWaypoint(locManager.ICB, locManager.getOrientationOrHeading(306.0), locManager.getOrientationOrHeading(306.0)),
                new PathPlannerWaypoint(locManager.IS9, locManager.getOrientationOrHeading(300.0), locManager.getOrientationOrHeading(300.0))),
            "CBToIS9Fast",
            alliance);

        // From CB to IS9
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_SUPER_MAX_PATH_TRANSLATIONAL_VELOCITY, // TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.CB, locManager.getOrientationOrHeading(306.0), locManager.getOrientationOrHeading(306.0)),
                new PathPlannerWaypoint(locManager.ICB, locManager.getOrientationOrHeading(306.0), locManager.getOrientationOrHeading(306.0)),
                new PathPlannerWaypoint(locManager.S9, locManager.getOrientationOrHeading(300.0), locManager.getOrientationOrHeading(300.0))),
            "CBToS9",
            alliance);

        // From CT to IS18
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.CT, locManager.getOrientationOrHeading(54.0), locManager.getOrientationOrHeading(54.0)),
                new PathPlannerWaypoint(locManager.IG1, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(120.0)),
                new PathPlannerWaypoint(locManager.IS18, locManager.getOrientationOrHeading(120.0), locManager.getOrientationOrHeading(120.0))),
            "CTToIS18",
            alliance);


        // From CT to IS16
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.CT, locManager.getOrientationOrHeading(54.0), locManager.getOrientationOrHeading(54.0)),
                new PathPlannerWaypoint(locManager.IG1, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(120.0)),
                new PathPlannerWaypoint(locManager.IS16, locManager.getOrientationOrHeading(120.0), locManager.getOrientationOrHeading(120.0))),
            "CTToIS16",
            alliance);

        // From CT to IS15
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_SUPER_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_SUPER_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.CT, locManager.getOrientationOrHeading(54.0), locManager.getOrientationOrHeading(54.0)),
                new PathPlannerWaypoint(locManager.IS15, locManager.getOrientationOrHeading(60.0), locManager.getOrientationOrHeading(60.0))),
            "CTToIS15",
            alliance);
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_SUPER_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_SUPER_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.CT, locManager.getOrientationOrHeading(54.0), locManager.getOrientationOrHeading(54.0)),
                new PathPlannerWaypoint(locManager.IS15, locManager.getOrientationOrHeading(60.0), locManager.getOrientationOrHeading(60.0))),
            "CTToIS15Fast",
            alliance);

        // From S6 to CB
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY, // TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION, // TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.S6, locManager.getOrientationOrHeading(60.0), locManager.getOrientationOrHeading(240.0)),
                new PathPlannerWaypoint(locManager.IG5, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(306.0)),
                new PathPlannerWaypoint(locManager.ICB, locManager.getOrientationOrHeading(126.0), locManager.getOrientationOrHeading(306.0)),
                new PathPlannerWaypoint(locManager.CB, locManager.getOrientationOrHeading(126.0), locManager.getOrientationOrHeading(306.0))),
            "S6ToCB",
            alliance);
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_SUPER_MAX_PATH_TRANSLATIONAL_VELOCITY, // TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_SUPER_MAX_PATH_TRANSLATIONAL_ACCELERATION, // TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.S6, locManager.getOrientationOrHeading(60.0), locManager.getOrientationOrHeading(240.0)),
                new PathPlannerWaypoint(locManager.IG5, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(306.0)),
                new PathPlannerWaypoint(locManager.ICB, locManager.getOrientationOrHeading(126.0), locManager.getOrientationOrHeading(306.0)),
                new PathPlannerWaypoint(locManager.CB, locManager.getOrientationOrHeading(126.0), locManager.getOrientationOrHeading(306.0))),
            "S6ToCBFast",
            alliance);
        
        // From S10 to CB
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.S10, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(0.0)),
                new PathPlannerWaypoint(locManager.IS10, locManager.getOrientationOrHeading(90.0), locManager.getOrientationOrHeading(306.0)),
                new PathPlannerWaypoint(locManager.ICB, locManager.getOrientationOrHeading(126.0), locManager.getOrientationOrHeading(306.0)),
                new PathPlannerWaypoint(locManager.CB, locManager.getOrientationOrHeading(126.0), locManager.getOrientationOrHeading(306.0))),
            "S10ToCB",
            alliance);

        // From S1 to CT
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.S1, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IS1, locManager.getOrientationOrHeading(90.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IS4, locManager.getOrientationOrHeading(54.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IG5, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(54.0)),
                new PathPlannerWaypoint(locManager.CT, locManager.getOrientationOrHeading(54.0), locManager.getOrientationOrHeading(54.0))),
            "S1ToCT",
            alliance);

        // From S13 to CT
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.S13, locManager.getOrientationOrHeading(240.0), locManager.getOrientationOrHeading(60.0)),
                new PathPlannerWaypoint(locManager.CT, locManager.getOrientationOrHeading(234.0), locManager.getOrientationOrHeading(54.0))),
            "S13ToCT",
            alliance);
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_SUPER_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_SUPER_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.S13, locManager.getOrientationOrHeading(240.0), locManager.getOrientationOrHeading(60.0)),
                new PathPlannerWaypoint(locManager.CTF, locManager.getOrientationOrHeading(234.0), locManager.getOrientationOrHeading(54.0))),
            "S13ToCTFast",
            alliance);

        // From S13 to CTR
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.S13, locManager.getOrientationOrHeading(240.0), locManager.getOrientationOrHeading(60.0)),
                new PathPlannerWaypoint(locManager.CTR, locManager.getOrientationOrHeading(234.0), locManager.getOrientationOrHeading(54.0))),
            "S13ToCTR",
            alliance);

        // From S12 to CT
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.S12, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(0.0)),
                new PathPlannerWaypoint(locManager.IS12, locManager.getOrientationOrHeading(270.0), locManager.getOrientationOrHeading(0.0)),
                new PathPlannerWaypoint(locManager.IPT, locManager.getOrientationOrHeading(270.0), locManager.getOrientationOrHeading(54.0)),
                new PathPlannerWaypoint(locManager.CT, locManager.getOrientationOrHeading(234.0), locManager.getOrientationOrHeading(54.0))),
            "S12ToCT",
            alliance);

        // From IS1 to S1
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.IS1, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.S1, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0))),
            "IS1ToS1",
            alliance);

        // From IS3 to S3
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.IS3, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.S3, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0))),
            "IS3ToS3",
            alliance);

        // From IS4 to S4
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.IS4, locManager.getOrientationOrHeading(240.0), locManager.getOrientationOrHeading(240.0)),
                new PathPlannerWaypoint(locManager.S4, locManager.getOrientationOrHeading(240.0), locManager.getOrientationOrHeading(240.0))),
            "IS4ToS4",
            alliance);

        // From IS6 to S6
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.IS6, locManager.getOrientationOrHeading(240.0), locManager.getOrientationOrHeading(240.0)),
                new PathPlannerWaypoint(locManager.S6, locManager.getOrientationOrHeading(240.0), locManager.getOrientationOrHeading(240.0))),
            "IS6ToS6",
            alliance);

        // From IS7 to S7
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.IS7, locManager.getOrientationOrHeading(300.0), locManager.getOrientationOrHeading(300.0)),
                new PathPlannerWaypoint(locManager.S7, locManager.getOrientationOrHeading(300.0), locManager.getOrientationOrHeading(300.0))),
            "IS7ToS7",
            alliance); 

        // From IS9 to S9
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.IS9, locManager.getOrientationOrHeading(300.0), locManager.getOrientationOrHeading(300.0)),
                new PathPlannerWaypoint(locManager.S9, locManager.getOrientationOrHeading(300.0), locManager.getOrientationOrHeading(300.0))),
            "IS9ToS9",
            alliance); 

        // From S1 to IS2
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.S1, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IS2, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0))),
            "S1ToIS2",
            alliance);

        // From IS2 to S2
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.IS2, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.S2, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0))),
            "IS2ToS2",
            alliance); 

        // From S2 to IS2
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.S2, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IS2, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(180.0))),
            "S2ToIS2",
            alliance); 

        // From S2 to IP
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.S2, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IS2, locManager.getOrientationOrHeading(270.0), locManager.getOrientationOrHeading(270.0)),
                new PathPlannerWaypoint(locManager.IP, locManager.getOrientationOrHeading(270.0), locManager.getOrientationOrHeading(270.0))),
            "S2ToIP",
            alliance);

        // From IP to P
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.IP, locManager.getOrientationOrHeading(270.0), locManager.getOrientationOrHeading(270.0)),
                new PathPlannerWaypoint(locManager.P, locManager.getOrientationOrHeading(270.0), locManager.getOrientationOrHeading(270.0))),
            "IPToP",
            alliance);

        // From S18 to IS17
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.S18, locManager.getOrientationOrHeading(300.0), locManager.getOrientationOrHeading(120.0)),
                new PathPlannerWaypoint(locManager.IS17, locManager.getOrientationOrHeading(300.0), locManager.getOrientationOrHeading(120.0))),
            "S18ToIS17",
            alliance);

        // From IS17 to S17
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.IS17, locManager.getOrientationOrHeading(120.0), locManager.getOrientationOrHeading(120.0)),
                new PathPlannerWaypoint(locManager.S17, locManager.getOrientationOrHeading(120.0), locManager.getOrientationOrHeading(120.0))),
            "IS17ToS17",
            alliance);

        // From IS17 to S17
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.S17, locManager.getOrientationOrHeading(300.0), locManager.getOrientationOrHeading(120.0)),
                new PathPlannerWaypoint(locManager.IP, locManager.getOrientationOrHeading(270.0), locManager.getOrientationOrHeading(270.0))),
            "S17ToIP",
            alliance);

        // From S6 to IS7
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.S6, locManager.getOrientationOrHeading(60.0), locManager.getOrientationOrHeading(240.0)),
                new PathPlannerWaypoint(locManager.IS6, locManager.getOrientationOrHeading(60.0), locManager.getOrientationOrHeading(240.0)),
                new PathPlannerWaypoint(locManager.IG5, locManager.getOrientationOrHeading(180.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.IS7, locManager.getOrientationOrHeading(120.0), locManager.getOrientationOrHeading(120.0))),
            "S6ToIS7",
            alliance);

        // IS7 to CB
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.IS7, locManager.getOrientationOrHeading(120.0), locManager.getOrientationOrHeading(300.0)),
                new PathPlannerWaypoint(locManager.ICB, locManager.getOrientationOrHeading(126.0), locManager.getOrientationOrHeading(306.0)),
                new PathPlannerWaypoint(locManager.CB, locManager.getOrientationOrHeading(126.0), locManager.getOrientationOrHeading(306.0))),
            "IS7ToCB",
            alliance);

        // IS2 to N
        addTrajectory(
            trajectoryManager,
            pathPlanner.buildTrajectory(
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY,
                TuningConstants.SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION,
                new PathPlannerWaypoint(locManager.IS2, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(180.0)),
                new PathPlannerWaypoint(locManager.N, locManager.getOrientationOrHeading(0.0), locManager.getOrientationOrHeading(0.0))),
            "IS2ToN",
            alliance);
    }

    private static void addTrajectory(TrajectoryManager trajectoryManager, ITrajectory trajectory, String name, Optional<Alliance> alliance)
    {
        ExceptionHelpers.Assert(trajectory != null, "Adding null trajectory '%s'!", name);
        try
        {
            if (alliance.isPresent())
            {
                name += alliance.get().toString();
            }

            trajectoryManager.addTrajectory(name, trajectory);
        }
        catch (Exception ex)
        {
            System.err.println("Encountered exception generating path " + name + ": " + ex.toString());
            if (TuningConstants.THROW_EXCEPTIONS)
            {
                throw ex;
            }
        }
    }
}
