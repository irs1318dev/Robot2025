package frc.robot.driver;

import frc.lib.driver.IOperation;

public enum DigitalOperation implements IOperation
{
    PositionResetFieldOrientation,
    PositionResetRobotLevel,

    // Driver interaction operations
    ForceLightDriverRumble,

    // Vision operations:
    VisionForceDisable,
    VisionEnableStream,
    VisionFindSpecificAprilTag,
    VisionFindAnyAprilTag,
    VisionFindAbsolutePosition,
    VisionFindReefTagsOnly,
    VisionFindReefRightCloseTagOnly, // 8, 17 APRILTAG_BLUE_REEF_RIGHT_CLOSE_ID, APRILTAG_RED_REEF_RIGHT_CLOSE_ID
    VisionFindReefRightFarTagOnly, // 9, 22 APRILTAG_RED_REEF_RIGHT_FAR_ID, APRILTAG_BLUE_REEF_RIGHT_FAR_ID
    VisionFindReefLeftCloseTagOnly, // 6, 19 APRILTAG_RED_REEF_LEFT_CLOSE_ID, APRILTAG_BLUE_REEF_LEFT_CLOSE_ID
    VisionFindReefLeftFarTagOnly, // 11, 20 APRILTAG_RED_REEF_LEFT_FAR_ID, APRILTAG_BLUE_REEF_LEFT_FAR_ID
    VisionFindReefCenterCloseTagOnly, // 7, 18 APRILTAG_RED_REEF_CENTER_CLOSE_ID, APRILTAG_BLUE_REEF_CENTER_CLOSE_ID
    VisionFindReefCenterFarTagOnly, // 10, 21 APRILTAG_RED_REEF_CENTER_FAR_ID, APRILTAG_BLUE_REEF_CENTER_FAR_ID

    // DriveTrain operations:
    DriveTrainSlowMode,
    DriveTrainPathMode,
    DriveTrainSteerMode,
    DriveTrainMaintainPositionMode,
    DriveTrainReset,
    DriveTrainEnableFieldOrientation,
    DriveTrainDisableFieldOrientation,
    DriveTrainUseRobotOrientation,
    DriveTrainEnableMaintainDirectionMode,
    DriveTrainDisableMaintainDirectionMode,
    DriveTrainResetXYPosition,
    DriveTrainIgnoreSlewRateLimitingMode,

    // Coral End Effector Operations
    CoralIntake, // move coral through end-effector away from alien funnel, toward reef placement
    CoralIntakeSlow, // move coral slowly through end-effector away from alien funnel, toward reef placement
    CoralPlace, // move coral out of end-effector to reef
    CoralPlaceSlow, // move coral slowly out of end-effector to reef
    CoralReverse, // move coral back towards the alien funnel, away from the reef placement
    CoralReverseSlow, // move coral slowly back towards the alien funnel, away from the reef placement
    CoralEnableThroughBeam,
    CoralDisableThroughBeam,
    CoralWristEnableSimpleMode,
    CoralWristDisableSimpleMode,
    CoalWristResetPosition,

    // Alge End Effector Operations
    AlgaeIntake,
    AlgaeOuttake,
    AlgaeLaunch,
    AlgaeEnableLimitSwitchHoldMode,
    AlgaeDisableLimitSwitchHoldMode,
    AlgaeWristEnableSimpleMode,
    AlgaeWristDisableSimpleMode,

    // Elevator Operations:
    ElevatorEnablePID,
    ElevatorDisablePID,

    // Climber operations:
    ClimberElbowEnableSimpleMode,
    ClimberElbowDisableSimpleMode,
    ClimberWinchClimb, 
}
