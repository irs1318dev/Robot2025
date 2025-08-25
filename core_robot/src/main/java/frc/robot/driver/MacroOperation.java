package frc.robot.driver;

import frc.lib.driver.IOperation;

public enum MacroOperation implements IOperation
{
    AutonomousRoutine,

    // DriveTrain operations:
    PIDLightBrake,
    PIDHeavyBrake,
    VisionApproachReefLeftPeg,
    VisionApproachReefRightPeg,
    OrientToReef,
    AlignCoralStationLeft,
    AlignCoralStationRight,

    // Vision operations

    // Path testing:
    FollowPathTest1,
    FollowPathTest2,
    FollowPathTest3,
    FollowPathTest4,
    FollowPathTest5,
    FollowPathTest6,
    FollowPathTest7,
    FollowPathTest10,
    FollowPathTest11,
    CheckWheelDiameter,

    // Coral operations:
    CoralIntake,
    CoralPlace,
    CoralReverse,
    CoralWristStow,

    // Algae operations:
    AlgaeIntake,
    AlgaeOuttake,
    AlgaeWristStowPosition,
    AlgaeWristReefPosition,
    AlgaeWristOutPosition,
    AlgaeWristBargePosition,
    AlgaeHoldPosition,

    // Climber operations:
    ClimberOut,
    ClimberIn,
    ClimberWinch,
    ClimberWinchExtra,
    ClimberWinchDestroy,

    // Elevator operations:
    ElevatorForceReset,
    ElevatorHome, // use instead of ElevatorCoralStationHeight??
    ElevatorCoralL1Height,
    ElevatorCoralL2Height,
    ElevatorCoralL3Height,
    ElevatorCoralL4Height,
    ElevatorCoralStationHeight, // use ElevatorHome instead??
    ElevatorAlgaeProcessorHeight,
    ElevatorAlgaeL23Height, // algae reef bottom
    ElevatorAlgaeL34Height, // algae reef top
    ElevatorAlgaeBargeHeight,
}
