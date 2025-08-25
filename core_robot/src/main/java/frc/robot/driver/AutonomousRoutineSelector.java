package frc.robot.driver;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.lib.driver.IControlTask;
import frc.lib.driver.TrajectoryManager;
import frc.lib.mechanisms.LoggingManager;
import frc.lib.robotprovider.Alliance;
import frc.lib.robotprovider.IDriverStation;
import frc.lib.robotprovider.ILogger;
import frc.lib.robotprovider.IRobotProvider;
import frc.lib.robotprovider.RobotMode;
import frc.robot.AutonLocManager;
import frc.robot.HardwareConstants;
import frc.robot.LoggingKey;
import frc.robot.TuningConstants;
import frc.robot.driver.SmartDashboardSelectionManager.AutoRoutine;
import frc.robot.driver.SmartDashboardSelectionManager.StartPosition;
import frc.robot.driver.controltasks.AlgaeIntakeControlTask;
import frc.robot.driver.controltasks.AlgaeIntakeControlTask.AlgaeResult;
import frc.robot.driver.controltasks.AlgaeWristSetAngleTask;
import frc.robot.driver.controltasks.ConcurrentTask;
import frc.robot.driver.controltasks.CoralIntakeControlTask;
import frc.robot.driver.controltasks.CoralIntakeControlTask.CoralResult;
import frc.robot.driver.controltasks.CoralWristSetAngleTask;
import frc.robot.driver.controltasks.DriveTrainDriveTask;
import frc.robot.driver.controltasks.ElevatorPositionTask;
import frc.robot.driver.controltasks.ElevatorPositionWaitTask;
import frc.robot.driver.controltasks.FallbackTask;
import frc.robot.driver.controltasks.FollowPathTask;
import frc.robot.driver.controltasks.FollowPathTask.Type;
import frc.robot.driver.controltasks.PositionStartingTask;
import frc.robot.driver.controltasks.RemainingTimeDecisionTask;
import frc.robot.driver.controltasks.SequentialTask;
import frc.robot.driver.controltasks.VisionApproachAprilTagTask;
import frc.robot.driver.controltasks.VisionAprilTagEnableTask;
import frc.robot.driver.controltasks.WaitTask;

@Singleton
public class AutonomousRoutineSelector
{
    private final ILogger logger;

    private final TrajectoryManager trajectoryManager;
    private final SmartDashboardSelectionManager selectionManager;
    private final IDriverStation driverStation;
    private final AutonLocManager locManager;

    /**
     * Initializes a new AutonomousRoutineSelector
     */
    @Inject
    public AutonomousRoutineSelector(
        LoggingManager logger,
        TrajectoryManager trajectoryManager,
        SmartDashboardSelectionManager selectionManager,
        IRobotProvider provider)
    {
        this.logger = logger;
        this.trajectoryManager = trajectoryManager;
        this.selectionManager = selectionManager;

        this.driverStation = provider.getDriverStation();

        this.locManager = new AutonLocManager(provider);

        PathPlannerTrajectoryGenerator.generateTrajectories(this.trajectoryManager, provider.getPathPlanner());
    }

    /**
     * Check what routine we want to use and return it
     * @param mode that is starting
     * @return autonomous routine to execute during autonomous mode
     */
    public IControlTask selectRoutine(RobotMode mode)
    {
        String driverStationMessage = this.driverStation.getGameSpecificMessage();
        this.logger.logString(LoggingKey.AutonomousDSMessage, driverStationMessage);
        if (mode == RobotMode.Test)
        {
            // currently not supported...
            return new WaitTask(0.0);
        }

        if (mode == RobotMode.Autonomous)
        {
            this.locManager.updateAlliance();
            StartPosition startPosition = this.selectionManager.getSelectedStartPosition();
            AutoRoutine routine = this.selectionManager.getSelectedAutoRoutine();

            boolean isRed = this.locManager.getIsRed();
            Alliance alliance = isRed ? Alliance.Red : Alliance.Blue;

            this.logger.logString(LoggingKey.AutonomousSelection, startPosition.toString() + "." + routine.toString());
            switch (startPosition) {
                case Right -> {
                    switch (routine) {
                        case taxi -> {
                            return Taxi(this.locManager);
                        }
                        case algaeProcess -> {
                            return RightAlgaeProcess(this.locManager, alliance);
                        }
                        case None -> {
                            return SetOrientationRoutine();
                        }
                        case relative -> {
                            return RelativeSideOneCoral(this.locManager, false);
                        }
                        case oneCoral -> {
                            return VisionRightOneCoral(this.locManager, alliance);
                        }
                        case twoCoral -> {
                            return VisionRightTwoCoral(this.locManager, alliance);
                        }
                        case threeCoral -> {
                            return VisionRightThreeCoral(this.locManager, alliance);
                        }
                        case threeCoralV2 -> {
                            return VisionRightThreeCoralV2(this.locManager, alliance);
                        }
                        case threeCoralWithTimeBasedFallback -> {
                            return VisionRightThreeCoralWithTimeBasedFallback(this.locManager, alliance);
                        }
                        default -> {
                        }
                    }
                }


                case Center -> {
                    switch (routine) {
                        case taxi -> {
                            return Taxi(this.locManager);
                        }
                        case algaeProcess -> {
                            return MiddleAlgaeProcess(this.locManager, alliance);
                        }
                        case None -> {
                            return SetOrientationRoutine();
                        }
                        case relative -> {
                            return RelativeCenterOneCoral(this.locManager);
                        }
                        case oneCoral -> {
                            return VisionMiddleOneCoral(this.locManager, alliance);
                        }
                        case twoCoral -> {
                            return SetOrientationRoutine();
                        }
                        case threeCoral -> {
                            return SetOrientationRoutine();
                        }
                        case netOneCoralOneAlgae -> {
                            return CenterNetOneAlgae(locManager, alliance);
                        }
                        case test -> {
                            return ElevatorVisionTest(locManager, alliance);
                        }
                        
                        default -> {
                        }
                    }
                }


                case Left -> {
                    switch (routine) {
                        case taxi -> {
                            return Taxi(this.locManager);
                        }
                        case algaeProcess -> {
                            return SetOrientationRoutine();
                        }
                        case None -> {
                            return SetOrientationRoutine();
                        }
                        case relative -> {
                            return RelativeSideOneCoral(this.locManager, true);
                        }
                        case oneCoral -> {
                            return VisionLeftOneCoral(this.locManager, alliance);
                        }
                        case twoCoral -> {
                            return VisionLeftTwoCoral(this.locManager, alliance);
                        }
                        case threeCoral -> {
                            return VisionLeftThreeCoral(locManager, alliance);
                        }
                        case threeCoralWithTimeBasedFallback -> {
                            return VisionLeftThreeCoralWithTimeBasedFallback(locManager, alliance);
                        }
                        case twoCoralAngled -> {
                            return VisionLeftTwoCoralAngled(this.locManager, alliance);
                        }
                        default -> {
                        }
                    }
                }


                default -> {
                }
            }
            
            return SetOrientationRoutine();
        }

        return GetFillerRoutine();
    }

    private IControlTask SetOrientationRoutine()
    {
        return new PositionStartingTask(this.locManager.B3, this.locManager.getOrientationOrHeading(180.0), true, true);
    }

    /**
     * Gets an autonomous routine that does nothing
     */
    private static IControlTask GetFillerRoutine()
    {
        return new WaitTask(0.0);
    }

    private static IControlTask RightAlgaeProcess(AutonLocManager locManager, Alliance alliance)
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(locManager.B3, locManager.getOrientationOrHeading(180.0), true, true),
            ConcurrentTask.AllTasks(
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE),
                new FollowPathTask(alliance, "B3ToIS18", Type.Absolute)
            ),
            ConcurrentTask.AllTasks(
                new FollowPathTask(alliance, "IS18ToS18", Type.Absolute),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT)
            ),
            new CoralIntakeControlTask(CoralResult.Place),
            ConcurrentTask.AllTasks(
                new FollowPathTask(alliance, "S18ToIS17", Type.Absolute),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_ALGAE_L34_HEIGHT),
                new AlgaeWristSetAngleTask(TuningConstants.ALGAE_WRIST_REEF_PICKUP_L34_ANGLE)
            ),
            ConcurrentTask.AllTasks(
                new FollowPathTask(alliance, "IS17ToS17", Type.Absolute),
                new AlgaeIntakeControlTask(AlgaeResult.Intake)
            ),
            new AlgaeWristSetAngleTask(TuningConstants.ALGAE_WRIST_HOLD_ANGLE),
            ConcurrentTask.AllTasks(
                new FollowPathTask(alliance, "S17ToIP", Type.Absolute),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_PROCESSOR_HEIGHT)
            ),
            new FollowPathTask(alliance, "IPToP", Type.Absolute),
            new AlgaeIntakeControlTask(AlgaeResult.Outtake)
        );
    }

    private static IControlTask VisionRightTwoCoral(AutonLocManager locManager, Alliance alliance)
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(locManager.B3a, locManager.getOrientationOrHeading(180.0), true, true),
            ConcurrentTask.AllTasks(
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                new FollowPathTask(alliance, "B3ToIS16", Type.Absolute)  
            ),
            new WaitTask(0.1),
            ConcurrentTask.AllTasks(
                new FallbackTask(
                    new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH + 1.5, 8.0, DigitalOperation.VisionFindReefRightFarTagOnly),
                    new FollowPathTask(alliance, "IS16ToS16", Type.Absolute)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT)
            ),
            new PositionStartingTask(locManager.S16, null, false, false),
            new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE),
            new CoralIntakeControlTask(CoralResult.Place),
            ConcurrentTask.AllTasks(
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                new FollowPathTask(alliance, "S16ToCT", Type.Absolute),
                SequentialTask.Sequence(
                    new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT),
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE))
            ),
            new CoralIntakeControlTask(CoralResult.IntakeSlow),
            ConcurrentTask.AllTasks(
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                new FollowPathTask(alliance, "CTToIS13", Type.Absolute)
            ),
            new WaitTask(0.1),
            ConcurrentTask.AllTasks(
                new FallbackTask(
                    new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH + 1.5, 8.0, DigitalOperation.VisionFindReefRightCloseTagOnly),
                    new FollowPathTask(alliance, "IS13ToS13", Type.Absolute)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT)
            ),
            new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE),
            new CoralIntakeControlTask(CoralResult.Place),
            new PositionStartingTask(locManager.S13, null, false, false),
            ConcurrentTask.AllTasks(
                SequentialTask.Sequence(
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_STOW_ANGLE),
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT),
                new FollowPathTask(alliance, "S13ToCTR", Type.Absolute)
            ),
            new CoralIntakeControlTask(CoralResult.IntakeSlow)
        );
    }

    private static IControlTask VisionRightThreeCoral(AutonLocManager locManager, Alliance alliance)
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(locManager.B3b, locManager.getOrientationOrHeading(180.0), true, true),
            ConcurrentTask.AnyTasks(
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                        new FollowPathTask(alliance, "B3bToIS16Fast", Type.Absolute)  
                    )),
                new VisionAprilTagEnableTask(DigitalOperation.VisionFindReefRightFarTagOnly, 5.0)),
            ConcurrentTask.AllTasks(
                new FallbackTask(
                    new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH + 1.5, 8.0, DigitalOperation.VisionFindReefRightFarTagOnly),
                    new FollowPathTask(alliance, "IS16ToS16", Type.Absolute)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT),
                SequentialTask.Sequence(
                    new ElevatorPositionWaitTask(TuningConstants.ELEVATOR_ALGAE_L34_CLEARING_HEIGHT, true),
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE)
                )
            ),
            new PositionStartingTask(locManager.S16, null, false, false),
            new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE),
            new CoralIntakeControlTask(CoralResult.Place),
            ConcurrentTask.AllTasks(
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                new FollowPathTask(alliance, "S16ToCTFast", Type.Absolute),
                SequentialTask.Sequence(
                    new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT),
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE))
            ),
            ConcurrentTask.AnyTasks(
                new CoralIntakeControlTask(CoralResult.IntakeSlow),
                new DriveTrainDriveTask(15.0, true, -0.1, 0.0)),
            ConcurrentTask.AnyTasks(
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                        new FollowPathTask(alliance, "CTToIS13Fast", Type.Absolute)
                    )
                ),
                new VisionAprilTagEnableTask(DigitalOperation.VisionFindReefRightCloseTagOnly, 5.0)
            ),
            ConcurrentTask.AllTasks(
                new FallbackTask(
                    new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH + 1.5, 8.0, DigitalOperation.VisionFindReefRightCloseTagOnly),
                    new FollowPathTask(alliance, "IS13ToS13", Type.Absolute)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT),
                SequentialTask.Sequence(
                    new ElevatorPositionWaitTask(TuningConstants.ELEVATOR_ALGAE_L23_CLEARING_HEIGHT, true),
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE)
                )
            ),
            new PositionStartingTask(locManager.S13, null, false, false),
            new CoralIntakeControlTask(CoralResult.Place),
            ConcurrentTask.AllTasks(
                SequentialTask.Sequence(
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT),
                new FollowPathTask(alliance, "S13ToCTFast", Type.Absolute)
            ),
            ConcurrentTask.AnyTasks(
                new CoralIntakeControlTask(CoralResult.IntakeSlow),
                new DriveTrainDriveTask(15.0, true, -0.1, 0.0)),
            new PositionStartingTask(locManager.CT, null, false, false),
            ConcurrentTask.AnyTasks(
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                        new FollowPathTask(alliance, "CTToIS15Fast", Type.Absolute)
                    ),
                    new WaitTask(0.1)
                    ),
                new VisionAprilTagEnableTask(DigitalOperation.VisionFindReefRightCloseTagOnly, 5.0)
            ),
            ConcurrentTask.AllTasks(
                new FallbackTask(
                    new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH + 1.5, -5.0, DigitalOperation.VisionFindReefRightCloseTagOnly),
                    new FollowPathTask(alliance, "IS15ToS15", Type.Absolute)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT),
                SequentialTask.Sequence(
                    new ElevatorPositionWaitTask(TuningConstants.ELEVATOR_ALGAE_L23_CLEARING_HEIGHT, true),
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE)
                )
            ),
            new CoralIntakeControlTask(CoralResult.Place),
            new PositionStartingTask(locManager.S15, null, false, false),
            new AlgaeWristSetAngleTask(TuningConstants.ALGAE_WRIST_STOW_ANGLE),
            new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_STOW_ANGLE),
            new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT)
        );
    }

    private static IControlTask VisionRightThreeCoralV2(AutonLocManager locManager, Alliance alliance)
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(locManager.B3b, locManager.getOrientationOrHeading(180.0), true, true),
            ConcurrentTask.AnyTasks(
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                        new FollowPathTask(alliance, "B3bToIS16Fast", Type.Absolute)  
                    )),
                new VisionAprilTagEnableTask(DigitalOperation.VisionFindReefRightFarTagOnly, 5.0)),
            ConcurrentTask.AllTasks(
                
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT),
                SequentialTask.Sequence(
                    new ElevatorPositionWaitTask(TuningConstants.ELEVATOR_ALGAE_L34_CLEARING_HEIGHT, true),
                    ConcurrentTask.AllTasks(
                        new FallbackTask(
                            new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH + 1.5, 8.0, DigitalOperation.VisionFindReefRightFarTagOnly),
                            new FollowPathTask(alliance, "IS16ToS16", Type.Absolute)
                        ),
                        new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE)
                    )
                )
            ),
            new PositionStartingTask(locManager.S16, null, false, false),
            new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE),
            new CoralIntakeControlTask(CoralResult.Place),
            ConcurrentTask.AllTasks(
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                new FollowPathTask(alliance, "S16ToCTFast", Type.Absolute),
                SequentialTask.Sequence(
                    new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT),
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE))
            ),
            ConcurrentTask.AnyTasks(
                new CoralIntakeControlTask(CoralResult.IntakeSlow),
                new DriveTrainDriveTask(15.0, true, -0.1, 0.0)),
            ConcurrentTask.AnyTasks(
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                        new FollowPathTask(alliance, "CTToIS13Fast", Type.Absolute)
                    )
                ),
                new VisionAprilTagEnableTask(DigitalOperation.VisionFindReefRightCloseTagOnly, 5.0)
            ),
            ConcurrentTask.AllTasks(
                new FallbackTask(
                    new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH + 1.5, 8.0, DigitalOperation.VisionFindReefRightCloseTagOnly),
                    new FollowPathTask(alliance, "IS13ToS13", Type.Absolute)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT),
                SequentialTask.Sequence(
                    new ElevatorPositionWaitTask(TuningConstants.ELEVATOR_ALGAE_L23_CLEARING_HEIGHT, true),
                    ConcurrentTask.AllTasks(
                        new FallbackTask(
                            new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH + 1.5, 8.0, DigitalOperation.VisionFindReefRightCloseTagOnly),
                            new FollowPathTask(alliance, "IS13ToS13", Type.Absolute)
                        ),
                        new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE)
                    )
                )
            ),
            new PositionStartingTask(locManager.S13, null, false, false),
            new CoralIntakeControlTask(CoralResult.Place),
            ConcurrentTask.AllTasks(
                SequentialTask.Sequence(
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT),
                new FollowPathTask(alliance, "S13ToCTFast", Type.Absolute)
            ),
            ConcurrentTask.AnyTasks(
                new CoralIntakeControlTask(CoralResult.IntakeSlow),
                new DriveTrainDriveTask(15.0, true, -0.1, 0.0)),
            new PositionStartingTask(locManager.CT, null, false, false),
            ConcurrentTask.AnyTasks(
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                        new FollowPathTask(alliance, "CTToIS15Fast", Type.Absolute)
                    ),
                    new WaitTask(0.1)
                    ),
                new VisionAprilTagEnableTask(DigitalOperation.VisionFindReefRightCloseTagOnly, 5.0)
            ),
            ConcurrentTask.AllTasks(
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT),
                SequentialTask.Sequence(
                    new ElevatorPositionWaitTask(TuningConstants.ELEVATOR_ALGAE_L23_CLEARING_HEIGHT, true),
                    ConcurrentTask.AllTasks(
                        new FallbackTask(
                            new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH + 1.5, -5.0, DigitalOperation.VisionFindReefRightCloseTagOnly),
                            new FollowPathTask(alliance, "IS15ToS15", Type.Absolute)
                        ),
                        new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE)
                    )   
                )
            ),
            new CoralIntakeControlTask(CoralResult.Place),
            new PositionStartingTask(locManager.S15, null, false, false),
            new AlgaeWristSetAngleTask(TuningConstants.ALGAE_WRIST_STOW_ANGLE),
            new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_STOW_ANGLE),
            new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT)
        );
    }

    private static IControlTask VisionRightThreeCoralWithTimeBasedFallback(AutonLocManager locManager, Alliance alliance)
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(locManager.B3b, locManager.getOrientationOrHeading(180.0), true, true),
            ConcurrentTask.AnyTasks(
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                        new FollowPathTask(alliance, "B3bToIS16Fast", Type.Absolute)  
                    )),
                new VisionAprilTagEnableTask(DigitalOperation.VisionFindReefRightFarTagOnly, 5.0)),
            ConcurrentTask.AllTasks(
                new FallbackTask(
                    new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH + 1.5, 8.0, DigitalOperation.VisionFindReefRightFarTagOnly),
                    new FollowPathTask(alliance, "IS16ToS16", Type.Absolute)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT),
                SequentialTask.Sequence(
                    new ElevatorPositionWaitTask(TuningConstants.ELEVATOR_ALGAE_L34_CLEARING_HEIGHT, true),
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE)
                )
            ),
            new PositionStartingTask(locManager.S16, null, false, false),
            new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE),
            new CoralIntakeControlTask(CoralResult.Place),
            ConcurrentTask.AllTasks(
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                new FollowPathTask(alliance, "S16ToCTFast", Type.Absolute),
                SequentialTask.Sequence(
                    new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT),
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE))
            ),
            new CoralIntakeControlTask(CoralResult.IntakeSlow),
            ConcurrentTask.AnyTasks(
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                        new FollowPathTask(alliance, "CTToIS13Fast", Type.Absolute)
                    )
                ),
                new VisionAprilTagEnableTask(DigitalOperation.VisionFindReefRightCloseTagOnly, 5.0)
            ),
            ConcurrentTask.AllTasks(
                new FallbackTask(
                    new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH + 1.5, 8.0, DigitalOperation.VisionFindReefRightCloseTagOnly),
                    new FollowPathTask(alliance, "IS13ToS13", Type.Absolute)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT),
                SequentialTask.Sequence(
                    new ElevatorPositionWaitTask(TuningConstants.ELEVATOR_ALGAE_L23_CLEARING_HEIGHT, true),
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE)
                )
            ),
            new PositionStartingTask(locManager.S13, null, false, false),
            new CoralIntakeControlTask(CoralResult.Place),
            ConcurrentTask.AllTasks(
                SequentialTask.Sequence(
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT),
                new FollowPathTask(alliance, "S13ToCTFast", Type.Absolute)
            ),
            new CoralIntakeControlTask(CoralResult.IntakeSlow),
            new PositionStartingTask(locManager.CT, null, false, false),
            new RemainingTimeDecisionTask(TuningConstants.AUTO_3_CORAL_DO_LAST_CORAL_TIME_THRESHOLD,
            SequentialTask.Sequence(
                ConcurrentTask.AnyTasks(
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                        new FollowPathTask(alliance, "CTToIS15Fast", Type.Absolute)
                    ),
                    new WaitTask(0.1)
                    ),
                new VisionAprilTagEnableTask(DigitalOperation.VisionFindReefRightCloseTagOnly, 5.0)
            ),
            ConcurrentTask.AllTasks(
                new FallbackTask(
                    new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH + 1.5, 8, DigitalOperation.VisionFindReefRightCloseTagOnly),
                    new FollowPathTask(alliance, "IS15ToS15", Type.Absolute)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L1_HEIGHT),
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE)
            ),
            new CoralIntakeControlTask(CoralResult.PlaceSlow)),
            
            SequentialTask.Sequence(
                ConcurrentTask.AnyTasks(
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                        new FollowPathTask(alliance, "CTToIS15Fast", Type.Absolute)
                    ),
                    new WaitTask(0.1)
                    ),
                new VisionAprilTagEnableTask(DigitalOperation.VisionFindReefRightCloseTagOnly, 5.0)
            ),
            ConcurrentTask.AllTasks(
                new FallbackTask(
                    new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH + 1.5, -5.0, DigitalOperation.VisionFindReefRightCloseTagOnly),
                    new FollowPathTask(alliance, "IS15ToS15", Type.Absolute)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT),
                SequentialTask.Sequence(
                    new ElevatorPositionWaitTask(TuningConstants.ELEVATOR_ALGAE_L23_CLEARING_HEIGHT, true),
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE)
                )
            ),
            new CoralIntakeControlTask(CoralResult.Place),
            new PositionStartingTask(locManager.S15, null, false, false)
            )),

            new AlgaeWristSetAngleTask(TuningConstants.ALGAE_WRIST_STOW_ANGLE),
            new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_STOW_ANGLE),
            new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT)
        );
    }

    private static IControlTask VisionRightOneCoral(AutonLocManager locManager, Alliance alliance)
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(locManager.B3, locManager.getOrientationOrHeading(180.0), true, true),
            new FollowPathTask(alliance, "B3ToIS18", Type.Absolute),
            ConcurrentTask.AllTasks(
                new FallbackTask(
                    new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH, -5.0, DigitalOperation.VisionFindReefTagsOnly), 
                    new FollowPathTask(alliance, "IS18ToS18", Type.Absolute)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT)
            ),
            new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE),
            new CoralIntakeControlTask(CoralResult.Place),
            new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_STOW_ANGLE),
            new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT)
        );
    }

    private static IControlTask MiddleAlgaeProcess(AutonLocManager locManager, Alliance alliance)
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(locManager.B2, locManager.getOrientationOrHeading(180.0), true, true),
            ConcurrentTask.AllTasks(
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                new FollowPathTask(alliance, "B2ToIS1", Type.Absolute)
            ),
            ConcurrentTask.AllTasks(
                new FallbackTask(
                    new VisionApproachAprilTagTask(TuningConstants.VISION_REEF_TAG_FORWARD_OFFSET_VALUE, TuningConstants.VISION_REEF_TAG_HORIZONTAL_LEFT_OFFSET_VALUE, DigitalOperation.VisionFindReefTagsOnly),
                    new FollowPathTask(alliance, "IS1ToS1", Type.Absolute)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT)
            ),
            new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE),
            new CoralIntakeControlTask(CoralResult.Place),
            new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_STOW_ANGLE),
            new ElevatorPositionTask(TuningConstants.ELEVATOR_ALGAE_L23_HEIGHT),
            new FollowPathTask(alliance, "S1ToIS2", Type.Absolute),
            ConcurrentTask.AllTasks(
                new FollowPathTask(alliance, "IS2ToS2", Type.Absolute),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_ALGAE_L23_HEIGHT),
                new AlgaeIntakeControlTask(AlgaeResult.Intake),
                new AlgaeWristSetAngleTask(TuningConstants.ALGAE_WRIST_REEF_PICKUP_L23_ANGLE)
            ),
            new AlgaeWristSetAngleTask(TuningConstants.ALGAE_WRIST_HOLD_ANGLE),
            new FollowPathTask(alliance, "S2ToIS2", Type.Absolute),
            ConcurrentTask.AllTasks(
                new FollowPathTask(alliance, "S2ToIS2", Type.Absolute),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_PROCESSOR_HEIGHT)
            ),
            new FollowPathTask(alliance, "S2ToIP", Type.Absolute),
            ConcurrentTask.AllTasks(
                new FollowPathTask(alliance, "IPToP", Type.Absolute),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_PROCESSOR_HEIGHT),
                new AlgaeWristSetAngleTask(TuningConstants.ALGAE_WRIST_PROCESSOR_ANGLE)
            ),
            new AlgaeIntakeControlTask(AlgaeResult.Outtake),
            new AlgaeWristSetAngleTask(TuningConstants.ALGAE_WRIST_STOW_ANGLE),
            new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_STOW_ANGLE),
            new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT)
        );
    }
    
    private static IControlTask VisionMiddleOneCoral(AutonLocManager locManager, Alliance alliance)
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(locManager.B2, locManager.getOrientationOrHeading(180.0), true, true),
            ConcurrentTask.AllTasks(
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE),
                new FollowPathTask(alliance, "B2ToIS1", Type.Absolute)
            ),
            ConcurrentTask.AllTasks(
                new FallbackTask(
                    new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH, 8.0, DigitalOperation.VisionFindReefTagsOnly),
                    new FollowPathTask(alliance, "IS1ToS1", Type.Absolute)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT)
            ),
            new CoralIntakeControlTask(CoralResult.Place),
            new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_STOW_ANGLE),
            new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT)
        );
    }

    private static IControlTask RelativeCenterOneCoral(AutonLocManager locManager)
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(locManager.B1, locManager.getOrientationOrHeading(180.0), true, true),
            ConcurrentTask.AllTasks(
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_STOW_ANGLE),
                new FollowPathTask("goForwards68in", Type.RobotRelativeFromCurrentPose)
            ),
            ConcurrentTask.AllTasks(
                new FallbackTask(
                    new VisionApproachAprilTagTask(TuningConstants.VISION_REEF_TAG_FORWARD_OFFSET_VALUE, TuningConstants.VISION_REEF_TAG_HORIZONTAL_LEFT_OFFSET_VALUE, DigitalOperation.VisionFindReefCenterFarTagOnly),
                    new FollowPathTask("goForwards20in", Type.RobotRelativeFromCurrentPose)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT)
            ),
            new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE),
            new CoralIntakeControlTask(CoralResult.Place),
            new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_STOW_ANGLE),
            new FollowPathTask("goBackwards8in", Type.RobotRelativeFromCurrentPose),
            new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT)
        );
    }

    private static IControlTask RelativeSideOneCoral(AutonLocManager locManager, boolean left)
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(locManager.B1, locManager.getOrientationOrHeading(180.0 + (left ? 45.0 : -45.0)), true, true),
            ConcurrentTask.AllTasks(
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_STOW_ANGLE),
                new FollowPathTask("goForwards68in", Type.RobotRelativeFromCurrentPose)
            ),
            new FallbackTask(
                SequentialTask.Sequence(
                    new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH, 8.0, DigitalOperation.VisionFindReefTagsOnly),
                    new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT),
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE),
                    new CoralIntakeControlTask(CoralResult.Place)
                ),
                SequentialTask.Sequence(
                    new FollowPathTask("goForwards20in", Type.RobotRelativeFromCurrentPose),
                    new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT),
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE),
                    new CoralIntakeControlTask(CoralResult.PlaceSlow)
                )
            ),
            new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_STOW_ANGLE),
            new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT)
        );
    }


    private static IControlTask VisionLeftTwoCoral(AutonLocManager locManager, Alliance alliance)
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(locManager.B1, locManager.getOrientationOrHeading(180.0), true, true),
            ConcurrentTask.AllTasks(
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                new FollowPathTask(alliance, "B1ToIS6", Type.Absolute)  
            ),
            new WaitTask(0.1),
            ConcurrentTask.AllTasks(
                new FallbackTask(
                    new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH + 1.5, -5.0, DigitalOperation.VisionFindReefLeftFarTagOnly),
                    new FollowPathTask(alliance, "IS6ToS6", Type.Absolute)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT)
            ),
            new PositionStartingTask(locManager.S6, null, false, false),

            new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE),
            new CoralIntakeControlTask(CoralResult.Place),
            ConcurrentTask.AllTasks(
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                new FollowPathTask(alliance, "S6ToCB", Type.Absolute),
                SequentialTask.Sequence(
                    new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT),
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE))
            ),
            new CoralIntakeControlTask(CoralResult.Intake),
            ConcurrentTask.AllTasks(
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                new FollowPathTask(alliance, "CBToIS9", Type.Absolute)
            ),
            new WaitTask(0.1),
            ConcurrentTask.AllTasks(
                new FallbackTask(
                    new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH + 1.5, -5.0, DigitalOperation.VisionFindReefLeftCloseTagOnly),
                    new FollowPathTask(alliance, "IS9ToS9", Type.Absolute)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT)
            ),
            new PositionStartingTask(locManager.S9, null, false, false),
            new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE),
            new CoralIntakeControlTask(CoralResult.Place),
            ConcurrentTask.AllTasks(
                SequentialTask.Sequence(
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT),
                new FollowPathTask(alliance, "S9ToCBR", Type.Absolute)
            ),
            new CoralIntakeControlTask(CoralResult.IntakeSlow)
        );
    }

    private static IControlTask VisionLeftThreeCoral(AutonLocManager locManager, Alliance alliance)
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(locManager.B1b, locManager.getOrientationOrHeading(180.0), true, true),
            ConcurrentTask.AnyTasks(
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                        new FollowPathTask(alliance, "B1bToIS6Fast", Type.Absolute)  
                    )),
                new VisionAprilTagEnableTask(DigitalOperation.VisionFindReefLeftFarTagOnly, 5.0)),
            ConcurrentTask.AllTasks(
                new FallbackTask(
                    new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH + 1.5, -5.0, DigitalOperation.VisionFindReefLeftFarTagOnly),
                    new FollowPathTask(alliance, "IS6ToS6", Type.Absolute)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT),
                SequentialTask.Sequence(
                    new ElevatorPositionWaitTask(TuningConstants.ELEVATOR_ALGAE_L34_CLEARING_HEIGHT, true),
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE)
                )
            ),
            new PositionStartingTask(locManager.S6, null, false, false),
            new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE),
            new CoralIntakeControlTask(CoralResult.Place),
            ConcurrentTask.AllTasks(
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                new FollowPathTask(alliance, "S6ToCBFast", Type.Absolute),
                SequentialTask.Sequence(
                    new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT),
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE))
            ),
            ConcurrentTask.AnyTasks(
                new CoralIntakeControlTask(CoralResult.IntakeSlow),
                new DriveTrainDriveTask(15.0, true, -0.1, 0.0)),
            ConcurrentTask.AnyTasks(
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                        new FollowPathTask(alliance, "CBToIS9Fast", Type.Absolute)
                    )
                ),
                new VisionAprilTagEnableTask(DigitalOperation.VisionFindReefLeftCloseTagOnly, 5.0)
            ),
            ConcurrentTask.AllTasks(
                new FallbackTask(
                    new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH + 1.5, -5.0, DigitalOperation.VisionFindReefLeftCloseTagOnly),
                    new FollowPathTask(alliance, "IS9ToS9", Type.Absolute)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT),
                SequentialTask.Sequence(
                    new ElevatorPositionWaitTask(TuningConstants.ELEVATOR_ALGAE_L23_CLEARING_HEIGHT, true),
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE)
                )
            ),
            new PositionStartingTask(locManager.S9, null, false, false),
            new CoralIntakeControlTask(CoralResult.Place),
            ConcurrentTask.AllTasks(
                SequentialTask.Sequence(
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT),
                new FollowPathTask(alliance, "S9ToCBFast", Type.Absolute)
            ),
            ConcurrentTask.AnyTasks(
                new CoralIntakeControlTask(CoralResult.IntakeSlow),
                new DriveTrainDriveTask(15.0, true, -0.1, 0.0)),
            new PositionStartingTask(locManager.CB, null, false, false),
            ConcurrentTask.AnyTasks(
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                        new FollowPathTask(alliance, "CBToIS7Fast", Type.Absolute)
                    ),
                    new WaitTask(0.1)
                    ),
                new VisionAprilTagEnableTask(DigitalOperation.VisionFindReefLeftCloseTagOnly, 5.0)
            ),
            ConcurrentTask.AllTasks(
                new FallbackTask(
                    new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH + 1.5, 8.0, DigitalOperation.VisionFindReefLeftCloseTagOnly),
                    new FollowPathTask(alliance, "IS7ToS7", Type.Absolute)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT),
                SequentialTask.Sequence(
                    new ElevatorPositionWaitTask(TuningConstants.ELEVATOR_ALGAE_L23_CLEARING_HEIGHT, true),
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE)
                )
            ),
            new CoralIntakeControlTask(CoralResult.Place),
            new PositionStartingTask(locManager.S9, null, false, false),
            new AlgaeWristSetAngleTask(TuningConstants.ALGAE_WRIST_STOW_ANGLE),
            new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_STOW_ANGLE),
            new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT)
        );
    }

    private static IControlTask VisionLeftThreeCoralWithTimeBasedFallback(AutonLocManager locManager, Alliance alliance)
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(locManager.B1b, locManager.getOrientationOrHeading(180.0), true, true),
            ConcurrentTask.AnyTasks(
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                        new FollowPathTask(alliance, "B1bToIS6Fast", Type.Absolute)  
                    )),
                new VisionAprilTagEnableTask(DigitalOperation.VisionFindReefLeftFarTagOnly, 5.0)),
            ConcurrentTask.AllTasks(
                new FallbackTask(
                    new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH + 1.5, -5.0, DigitalOperation.VisionFindReefLeftFarTagOnly),
                    new FollowPathTask(alliance, "IS6ToS6", Type.Absolute)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT),
                SequentialTask.Sequence(
                    new ElevatorPositionWaitTask(TuningConstants.ELEVATOR_ALGAE_L34_CLEARING_HEIGHT, true),
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE)
                )
            ),
            new PositionStartingTask(locManager.S6, null, false, false),
            new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE),
            new CoralIntakeControlTask(CoralResult.Place),
            ConcurrentTask.AllTasks(
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                new FollowPathTask(alliance, "S6ToCBFast", Type.Absolute),
                SequentialTask.Sequence(
                    new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT),
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE))
            ),
            new CoralIntakeControlTask(CoralResult.IntakeSlow),
            ConcurrentTask.AnyTasks(
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                        new FollowPathTask(alliance, "CBToIS9Fast", Type.Absolute)
                    )
                ),
                new VisionAprilTagEnableTask(DigitalOperation.VisionFindReefLeftCloseTagOnly, 5.0)
            ),
            ConcurrentTask.AllTasks(
                new FallbackTask(
                    new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH + 1.5, -5.0, DigitalOperation.VisionFindReefLeftCloseTagOnly),
                    new FollowPathTask(alliance, "IS9ToS9", Type.Absolute)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT),
                SequentialTask.Sequence(
                    new ElevatorPositionWaitTask(TuningConstants.ELEVATOR_ALGAE_L23_CLEARING_HEIGHT, true),
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE)
                )
            ),
            new PositionStartingTask(locManager.S9, null, false, false),
            new CoralIntakeControlTask(CoralResult.Place),
            ConcurrentTask.AllTasks(
                SequentialTask.Sequence(
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT),
                new FollowPathTask(alliance, "S9ToCBFast", Type.Absolute)
            ),
            new CoralIntakeControlTask(CoralResult.IntakeSlow),
            new PositionStartingTask(locManager.CB, null, false, false),
            
            new RemainingTimeDecisionTask(TuningConstants.AUTO_3_CORAL_DO_LAST_CORAL_TIME_THRESHOLD,
            SequentialTask.Sequence(
                ConcurrentTask.AnyTasks(
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                        new FollowPathTask(alliance, "CBToIS7Fast", Type.Absolute)
                    ),
                    new WaitTask(0.1)
                    ),
                new VisionAprilTagEnableTask(DigitalOperation.VisionFindReefLeftCloseTagOnly, 5.0)
            ),
            ConcurrentTask.AllTasks(
                new FallbackTask(
                    new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH + 1.5, 8, DigitalOperation.VisionFindReefLeftCloseTagOnly),
                    new FollowPathTask(alliance, "IS7ToS7", Type.Absolute)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L1_HEIGHT),
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE)
            ),
            new CoralIntakeControlTask(CoralResult.PlaceSlow)
            ),
            SequentialTask.Sequence(
                ConcurrentTask.AnyTasks(
                SequentialTask.Sequence(
                    ConcurrentTask.AllTasks(
                        new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                        new FollowPathTask(alliance, "CBToIS7Fast", Type.Absolute)
                    ),
                    new WaitTask(0.1)
                    ),
                new VisionAprilTagEnableTask(DigitalOperation.VisionFindReefLeftCloseTagOnly, 5.0)
            ),
            ConcurrentTask.AllTasks(
                new FallbackTask(
                    new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH + 1.5, 8.0, DigitalOperation.VisionFindReefLeftCloseTagOnly),
                    new FollowPathTask(alliance, "IS7ToS7", Type.Absolute)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT),
                SequentialTask.Sequence(
                    new ElevatorPositionWaitTask(TuningConstants.ELEVATOR_ALGAE_L23_CLEARING_HEIGHT, true),
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE)
                )
            ),
            new CoralIntakeControlTask(CoralResult.Place),
            new PositionStartingTask(locManager.S9, null, false, false)
            )),

            new AlgaeWristSetAngleTask(TuningConstants.ALGAE_WRIST_STOW_ANGLE),
            new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_STOW_ANGLE),
            new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT)
        );
    }

    private static IControlTask VisionLeftTwoCoralAngled(AutonLocManager locManager, Alliance alliance)
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(locManager.B1a, locManager.getOrientationOrHeading(225.0), true, true),
            ConcurrentTask.AllTasks(
                new FollowPathTask(alliance, "B1aToIS6", Type.Absolute)  
            ),
            new WaitTask(0.1),
            ConcurrentTask.AllTasks(
                new FallbackTask(
                    new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH, -5.0, DigitalOperation.VisionFindReefTagsOnly),
                    new FollowPathTask(alliance, "IS6ToS6", Type.Absolute)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT)
            ),
            new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE),
            new CoralIntakeControlTask(CoralResult.Place),
            ConcurrentTask.AllTasks(
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_STOW_ANGLE),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT),
                new FollowPathTask(alliance, "S6ToIS7", Type.Absolute)
            ),
            new FallbackTask(
                new VisionApproachAprilTagTask(0, 0, DigitalOperation.VisionFindReefTagsOnly), // CHANGE THIS WHEN TESTING
                new FollowPathTask(alliance, "IS7ToCB", Type.Absolute)
            ),
            new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE),
            new CoralIntakeControlTask(CoralResult.Intake),
            ConcurrentTask.AllTasks(
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_STOW_ANGLE),
                new FollowPathTask(alliance, "CBToIS9", Type.Absolute)
            ),
            new WaitTask(0.1),
            ConcurrentTask.AllTasks(
                new FallbackTask(
                    new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH, -5.0, DigitalOperation.VisionFindReefTagsOnly),
                    new FollowPathTask(alliance, "IS9ToS9", Type.Absolute)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT)
            ),
            new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE),
            new CoralIntakeControlTask(CoralResult.Place),
            new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_STOW_ANGLE),
            new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT)
        );
    }
    private static IControlTask VisionLeftOneCoral(AutonLocManager locManager, Alliance alliance)
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(locManager.B1, locManager.getOrientationOrHeading(180.0), true, true),
            new FollowPathTask(alliance, "B1ToIS6", Type.Absolute),
            new WaitTask(0.2),
            ConcurrentTask.AllTasks(
                new FallbackTask(
                    new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH + 2, -5.0, DigitalOperation.VisionFindReefTagsOnly), 
                    new FollowPathTask(alliance, "IS6ToS6", Type.Absolute)
                    ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT)
            ),
            new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE),
            new CoralIntakeControlTask(CoralResult.Place),
            new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_STOW_ANGLE),
            new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT)
        );
    }

    private static IControlTask CenterNetOneAlgae(AutonLocManager locManager, Alliance alliance) {
        return SequentialTask.Sequence(
            new PositionStartingTask(locManager.B2, locManager.getOrientationOrHeading(180.0), true, true),
            ConcurrentTask.AnyTasks(
                ConcurrentTask.AllTasks(
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                    new FollowPathTask(alliance, "B2ToIS1", Type.Absolute)
                ),
                new VisionAprilTagEnableTask(DigitalOperation.VisionFindReefCenterFarTagOnly, 5.0)),
            ConcurrentTask.AllTasks(
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT),
                new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH + 2, 8.0, DigitalOperation.VisionFindReefCenterFarTagOnly),
                
                
                
                SequentialTask.Sequence(
                    new ElevatorPositionWaitTask(TuningConstants.ELEVATOR_ALGAE_L23_CLEARING_HEIGHT, true),
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE)
                )
            ),
            new CoralIntakeControlTask(CoralResult.Place),
            new PositionStartingTask(locManager.S1, null, false, false),
            ConcurrentTask.AllTasks(
                new FollowPathTask(alliance, "S1ToIS2", Type.Absolute),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT),
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_STOW_ANGLE)
            ),
            ConcurrentTask.AllTasks(
                new ElevatorPositionTask(TuningConstants.ELEVATOR_ALGAE_L23_HEIGHT),
                new AlgaeWristSetAngleTask(TuningConstants.ALGAE_WRIST_REEF_PICKUP_L23_ANGLE)
            ),
            ConcurrentTask.AllTasks(
                new AlgaeIntakeControlTask(AlgaeResult.Intake),
                new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH + 2, 2.0, DigitalOperation.VisionFindReefCenterFarTagOnly)
            ),
            new PositionStartingTask(locManager.S2, null, false, false),
            new FollowPathTask(alliance, "S2ToIS2", Type.Absolute),
            ConcurrentTask.AllTasks(
                new AlgaeWristSetAngleTask(TuningConstants.ALGAE_WRIST_HOLD_ANGLE),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT),
                new FollowPathTask(alliance, "IS2ToN", Type.Absolute)
            ),
            new ElevatorPositionTask(TuningConstants.ELEVATOR_ALGAE_BARGE_HEIGHT),
            ConcurrentTask.AllTasks(
                new AlgaeWristSetAngleTask(TuningConstants.ALGAE_WRIST_HOLD_ANGLE),
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_STOW_ANGLE)
            ),
            ConcurrentTask.AllTasks(
                new FollowPathTask("goForwards4in", Type.RobotRelativeFromCurrentPose),
                SequentialTask.Sequence(
                    new WaitTask(0.1),
                    new AlgaeIntakeControlTask(AlgaeResult.Outtake, 1.5)
                )
            ),
            ConcurrentTask.AllTasks(
                new FollowPathTask("goBackwards36in", Type.RobotRelativeFromCurrentPose),
                SequentialTask.Sequence(
                    new WaitTask(1.0),
                    ConcurrentTask.AllTasks(
                        new AlgaeWristSetAngleTask(TuningConstants.ALGAE_WRIST_STOW_ANGLE),
                        new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_STOW_ANGLE),
                        new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT)
                    )
                )
            )
        );
    }

    private static IControlTask ElevatorVisionTest(AutonLocManager locManager, Alliance alliance) {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new PositionStartingTask(locManager.B2, locManager.getOrientationOrHeading(180.0), true, true),
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE),
                new VisionApproachAprilTagTask(HardwareConstants.ROBOT_HALF_SIDE_LENGTH + 1.5, 8.0, DigitalOperation.VisionFindReefCenterFarTagOnly,
                (duration) -> SequentialTask.Sequence(
                    new WaitTask((duration >= TuningConstants.AUTO_TIME_TO_RAISE_ELEVATOR) ? duration - TuningConstants.AUTO_TIME_TO_RAISE_ELEVATOR : 0.0),
                    new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT)
                )),

                SequentialTask.Sequence(
                    new ElevatorPositionWaitTask(TuningConstants.ELEVATOR_ALGAE_L34_CLEARING_HEIGHT, true),
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE)
                )
            ),
            new CoralIntakeControlTask(CoralResult.Place),
            new AlgaeWristSetAngleTask(TuningConstants.ALGAE_WRIST_STOW_ANGLE),
            new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_STOW_ANGLE),
            new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT)
        );
    }

    private static IControlTask Taxi(AutonLocManager locManager)
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(locManager.B1, locManager.getOrientationOrHeading(180.0), true, true),
            new FollowPathTask("goForwards45in", Type.RobotRelativeFromCurrentPose)
        );
    }
}



//IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS IRS




































































































































/*
                                      .
                                    .;+;+
                                    .+;;'   `,+'.
                                    ;';;+:..`` :+'+
                                    ,'+`    .+;;;;;+
                                     ;,,, .+;;;;;'+++;
                                     ;' `+;;;;;#+'+'+''#:.
                                     '`+';;;'+;+;+++'''+'.
                                     #';;;;#';+'+'''+''+'
                                     ;;;;#;,+;;+;;;'''''':
                                     ';'++'.`+;;'';;''+'',
                                     :#'#+'``.'+++'#++'':`
                                      `';++##```##+.''.##
                                      +++#   #`#  `++++
                                      +'#+ # :#: # ##'+
                                      `#+#   +`+   #'#`
                                       :,.+,+,`:+,+..,
                                       `,:```,`,`.`;,
                                        :+.;``.``;.#;
                                        .'``'+'+'``'.
                                         ,````````..
                                          :```````:
                                          +``.:,``'
                                          :```````:
                                           +`````+
                                            ';+##
                                            '```'
                                           `'```'`
                                         .+''''''''
                                        +;;;;;;;;''#
                                       :       `   `:
                                      `,            '
                                      +              '
                                     ,;';,``.``.,,,:;#
                                     +;;;;;;;;;;;;;;;'
                                    ,';;;;;;;;;;;;;;;',
                                    +:;;;;;;';;;;;;;;;+
                                   `.   .:,;+;;:::;.``,
                                   :`       #,       `.`
                                   +       # ;        .;
                                  .;;,`    ,         `,+
                                  +;;;;;;''';;;;;;;';;';
                                  +;;;;;;;';;;;;;;;;;'';;
                                 `';;;;;;';;;;;;;;;;;';;+
                                 + `:;;;;+;;;;;;;;';'''::
                                 '     `:  ```````    ,  ,
                                :       '             ;  +
                                '`     ..             ,  ,
                               ,;;;;;..+,`        ```.':;',
                               +;;;;;;'+;;;;;;;;;;;;;;+;;;+
                               ';;;;;;++;;;;;;;;;;;;;;';;;+
                              `.:';;;;;#;;;;;;;;;;;;;;';;;;`
                              ;    `,; ',:;;';;';;;;;:;``  +
                              +      ; ;              ;    `
                              ;      : +              '    `;
                              ';:`` `` '              :`,:;;+
                             `';;;;'+  +,..```````..:;#;;;;;;.
                             `;;;;;;+  +;;;;;;;;;;;;;':';;;;;#
                             .;;;;;;+  ';;;;;;;;;;;;;;,';;;;` .
                             : `.;;'+  +;;;;;;;;;;;;;','.`    +
                             '      ;  +.,,;:;:;;;,..`: ,     ``
                             +      ,  '              : ;   .;'+
                             +.`   ``  +              ;  ;:;;;;':
                             ';;;';;`  +             .'  ;;;;;;;+
                             ';;;;;'   :+++#++##+#+''',   +;;;;.`.
                             +;;;;;'   +;;::;;;+:+;;'',   ,;;.   +
                            ``:;;;;+   +;;:;;;:+;+;;++;    +     .`
                             `   ``'   +;;;;;;;+;+;;'+;     ,   ;#,
                            .      ;   ';;;;;;;;;;;;++'     + .+``.;
                            ``     ;   ';;;;;;+;';;;'+'      #`````:,
                             +++;,:.   ':;''++;:';:;'';      +``````,`
                             ,```,+    +;;';:;;+;;;;'';      +``````,+
                            .``````:   ;:;;++';;;;;;';,      ,``:#``+`.
                            ,``````'   `';;;;:;;;;;;+;`     '+``+:'`..'
                            ,``````'    +;;;;;;;;;;;''     ;:'``#;;.`++
                            ```````;    `;:;;;;;;;;;;#     ':'``++:+`+;
                            ```'`.`;     +;;;;;;;;;;;+    :::#``' +#`';
                            ,``'`:`#     `';;;;;;;;;;+    +:'.`,. ++`;;
                            +`.``+`'     :#;;;;;;;;;;;`   +:# ,`  +;`.'
                           ,.`+`.:.      ##;;;;;;;;;;;'   ,'`     ;:+#
                           '`;.`+`#      ##+;;;;;;;;;;+          ,::;
                           ,+,`:``,     :###;;;;;;;;;:'          +:;`
                            '`,,`+      ';##';;;;;;;;;;.         +:#
                             '+.+       +;;##;;;;;;;;;;'         ;:;
                               `       :;;;+#;;;;;;;;;;+        ;::`
                                       +;;;;#+;;;;;;;;;;        +:'
                                       ';;;;+#;;;;;;;;;;.       ;:'
                                      ,;;;;;;#;;;;;;;;;;+      +::.
                                      +;;;;;;'';;;;;;;;;'      +:+
                                     `;;;;;;;;#;;;;;;;;;;`    `;:+
                                     ,;;;;;;;;+;;;;;;;;;;+    ':;,
                                     +;;;;;;;;;+;;;;;;;;;'    +:+
                                    .;;;;;;;;;+,;;;;;;;;;;`   ;;+
                                    ';;;;;;;;;, ';;;;;;:;;,  +;:,
                                    ';;;;;;;;'  +;;;;;;;;;'  +:+
                                   ;;;;;;;;;;+  ,;;;;;;;;;+  ;:'
                                   +;;;;;;;;;    ';;;;;;;;;`;:;`
                                   ;;;;;;;;;+    +;;;;;;;;;+#:+
                                  ';;;;;;;;;:    ;;;;;;;;;;';:'
                                 `';;;;;;;:'      ';;;;;;;;;;:.
                                 .;;;;;;;;;+      +;;;;;;;;;'+
                                 +;;;;;;;;;       ';;;;;;;;;#+
                                `;;;;;;;;;+       `;;;;;;;;;;`
                                +;;;;;;;;;.        +;;;;;;;;;`
                                ';;;;;;;:'         ;;;;;;;;;;;
                               :;;;;;;;;;:         `;;;;;;;;;+
                               +;;;;;;;;;           ';;;;;;;;;`
                               ;;;;;;;;;+           ';;;;;;;;;:
                              ';;;;;;;;;;           ,;;;;;;;;;+
                              ':;;;;;;;'             +;;;;;;;;;
                             .;:;;;;;;;'             +;;;;;;;;;:
                             +;;;;;;;;;`             .;;;;;;;;;+
                            `;;;;;;;;;+               ;:;;;;;;;;`
                            ;;;;;;;;;;.               +;;;;;;;::.
                            ';;;;;;;;'`               :;;;;;;;;:+
                           :;;;;;;;;:'                ';;;;;;;;;'
                           ';;;;;;;;'`                +#;;;;;;;;;`
                          `;;;;;;;;;+                 '';;;;;;;;;+
                          +;;;;;;;;;.                '::;;;;;;;;;+
                          ;;;;;;;;;+                 #:'';;;;;;;;;`
                         .#;;;;;;;;'                `;:+;;;;;;;;;;;
                         ':'';;;;;;                 '::.,;;;;;;;;;+
                        +::::+';;;+                 ':'  +:;;;;;;;;`
                       `;;;::::;#+:                `;:+  +;;;;;;;:;;      '#+,
                       +#::::::::;'`               +:;,  `;;;;:;;'#';;;;;::;:'`
                      ';:''::::::::#`              +:'    ';:;;+'::;;:;::::::''
                      #+::;+':::::::'.            .:;+    '''+;::;:;:::;:::;':'
                    `';+';;:;'';:::::':    '      +::.     +:::::::::::::;#;:#
                    :+;#'.''##;#;:;;:::'+  #     `+;'      ;:;::::::::;'+;:'+
                   '#;+". ` `+:;+:;::;::+'#+     +:;#     ';:::;:+#+';:::+.
                   ';#''      ,+::+#';::;+'#+    ';::      #:;;'+';'''++:`
                                '':::;'''#+     ,:;;`      #';:;;:+
                                 `:'++;;':       :++       .;;:;;#,
                                       `                    '':``
           _             _      __        __    _     _       
__      _| |__  _   _  (_)___  \ \      / /_ _| | __| | ___  
\ \ /\ / / '_ \| | | | | / __|  \ \ /\ / / _` | |/ _` |/ _ \ 
 \ V  V /| | | | |_| | | \__ \   \ V  V / (_| | | (_| | (_) |
 _\_/\_/ |_| |_|\__, | |_|___/    \_/\_/ \__,_|_|\__,_|\___/ 
| |__   ___ _ __|___/                                        
| '_ \ / _ \ '__/ _ \                                        
| | | |  __/ | |  __/                                        
|_| |_|\___|_|  \___|     

*/
