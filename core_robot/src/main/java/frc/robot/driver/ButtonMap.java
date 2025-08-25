package frc.robot.driver;

import java.util.EnumSet;

import javax.inject.Singleton;

import frc.lib.driver.AnalogAxis;
import frc.lib.driver.IButtonMap;
import frc.lib.driver.IOperation;
import frc.lib.driver.UserInputDeviceButton;
import frc.lib.driver.buttons.ButtonType;
import frc.lib.driver.descriptions.AnalogOperationDescription;
import frc.lib.driver.descriptions.DigitalOperationDescription;
import frc.lib.driver.descriptions.MacroOperationDescription;
import frc.lib.driver.descriptions.ShiftDescription;
import frc.lib.driver.descriptions.UserInputDevice;
import frc.lib.helpers.Helpers;
import frc.robot.ElectronicsConstants;
import frc.robot.HardwareConstants;
import frc.robot.TuningConstants;
import frc.robot.driver.controltasks.AlgaeDecisionTask;
import frc.robot.driver.controltasks.AlgaeIntakeControlTask;
import frc.robot.driver.controltasks.AlgaeIntakeControlTask.AlgaeResult;
import frc.robot.driver.controltasks.AlgaePositionBasedDecisionTask;
import frc.robot.driver.controltasks.ClimberWinchTask.ClimbMode;
import frc.robot.driver.controltasks.AlgaeWristSetAngleTask;
import frc.robot.driver.controltasks.ClimberElbowSetPositionTask;
import frc.robot.driver.controltasks.ClimberWinchTask;
import frc.robot.driver.controltasks.ConcurrentTask;
import frc.robot.driver.controltasks.CoralIntakeControlTask;
import frc.robot.driver.controltasks.CoralIntakeControlTask.CoralResult;
import frc.robot.driver.controltasks.CoralWristSetAngleTask;
import frc.robot.driver.controltasks.ElevatorPositionBasedDecisionTask;
import frc.robot.driver.controltasks.ElevatorPositionResetTask;
import frc.robot.driver.controltasks.ElevatorPositionTask;
import frc.robot.driver.controltasks.ElevatorPositionWaitTask;
import frc.robot.driver.controltasks.FallbackTask;
import frc.robot.driver.controltasks.FieldOrientationTask;
import frc.robot.driver.controltasks.FieldOrientationTask.DesiredOrientation;
import frc.robot.driver.controltasks.FollowPathTask;
import frc.robot.driver.controltasks.FollowPathTask.Type;
import frc.robot.driver.controltasks.PIDBrakeTask;
import frc.robot.driver.controltasks.PositionStartingTask;
import frc.robot.driver.controltasks.RumbleTask;
import frc.robot.driver.controltasks.SequentialTask;
import frc.robot.driver.controltasks.VisionApproachAprilTagTask;
import frc.robot.driver.controltasks.WaitTask;

@Singleton
public class ButtonMap implements IButtonMap
{
    private static ShiftDescription[] ShiftButtonSchema = new ShiftDescription[]
    {
        new ShiftDescription(
            Shift.DriverDebug,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_LEFT_STICK_BUTTON),
        new ShiftDescription(
            Shift.CodriverDebug,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_LEFT_BUTTON),
        new ShiftDescription(
            Shift.Test1Debug,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_LEFT_BUTTON),
        new ShiftDescription(
            Shift.Test2Debug,
            UserInputDevice.Test2,
            UserInputDeviceButton.XBONE_LEFT_BUTTON),
    };

    public static AnalogOperationDescription[] AnalogOperationSchema = new AnalogOperationDescription[]
    {
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainMoveForward,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_LSY,
            ElectronicsConstants.INVERT_XBONE_LEFT_Y_AXIS,
            -TuningConstants.SDSDRIVETRAIN_DEAD_ZONE_VELOCITY_Y,
            TuningConstants.SDSDRIVETRAIN_DEAD_ZONE_VELOCITY_Y,
            1.0,
            TuningConstants.SDSDRIVETRAIN_EXPONENTIAL),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainMoveRight,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_LSX,
            ElectronicsConstants.INVERT_XBONE_LEFT_X_AXIS,
            -TuningConstants.SDSDRIVETRAIN_DEAD_ZONE_VELOCITY_X,
            TuningConstants.SDSDRIVETRAIN_DEAD_ZONE_VELOCITY_X,
            1.0,
            TuningConstants.SDSDRIVETRAIN_EXPONENTIAL),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainTurnAngleGoal,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_RSX,
            AnalogAxis.XBONE_RSY,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            !ElectronicsConstants.INVERT_XBONE_RIGHT_X_AXIS, // make left positive...
            ElectronicsConstants.INVERT_XBONE_RIGHT_Y_AXIS,
            0.0,
            TuningConstants.SDSDRIVETRAIN_SKIP_OMEGA_ON_ZERO_DELTA,
            true,
            TuningConstants.MAGIC_NULL_VALUE,
            (x, y) -> Helpers.atan2d(x, y)),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainSpinLeft,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_LT,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ElectronicsConstants.INVERT_XBONE_LEFT_TRIGGER, // turning left should be positive, as counter-clockwise is positive
            -TuningConstants.SDSDRIVETRAIN_DEAD_ZONE_TURN,
            TuningConstants.SDSDRIVETRAIN_DEAD_ZONE_TURN),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainSpinRight,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_RT,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            !ElectronicsConstants.INVERT_XBONE_RIGHT_TRIGGER, // turning right should be negative, as counter-clockwise is positive
            -TuningConstants.SDSDRIVETRAIN_DEAD_ZONE_TURN,
            TuningConstants.SDSDRIVETRAIN_DEAD_ZONE_TURN),
        new AnalogOperationDescription(
            AnalogOperation.CoralWristAdjustAngleLeft,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_LT,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ElectronicsConstants.INVERT_XBONE_LEFT_TRIGGER, // turning left should be positive, as counter-clockwise is positive
            -TuningConstants.CORAL_WRIST_DEAD_ZONE,
            TuningConstants.CORAL_WRIST_DEAD_ZONE),
        new AnalogOperationDescription(
            AnalogOperation.CoralWristAdjustAngleRight,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_RT,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            !ElectronicsConstants.INVERT_XBONE_RIGHT_TRIGGER, // turning right should be negative, as counter-clockwise is positive
            -TuningConstants.CORAL_WRIST_DEAD_ZONE,
            TuningConstants.CORAL_WRIST_DEAD_ZONE),
        new AnalogOperationDescription(
            AnalogOperation.CoralWristPowerA,
            UserInputDevice.Codriver,
            AnalogAxis.XBONE_LT,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.of(Shift.CodriverDebug),
            ElectronicsConstants.INVERT_XBONE_LEFT_TRIGGER, // turning left should be positive, as counter-clockwise is positive
            -TuningConstants.CORAL_WRIST_DEAD_ZONE,
            TuningConstants.CORAL_WRIST_DEAD_ZONE),
        new AnalogOperationDescription(
            AnalogOperation.CoralWristPowerB,
            UserInputDevice.Codriver,
            AnalogAxis.XBONE_RT,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.of(Shift.CodriverDebug),
            !ElectronicsConstants.INVERT_XBONE_RIGHT_TRIGGER, // turning right should be negative, as counter-clockwise is positive
            -TuningConstants.CORAL_WRIST_DEAD_ZONE,
            TuningConstants.CORAL_WRIST_DEAD_ZONE),
        new AnalogOperationDescription(
            AnalogOperation.ElevatorPower,
            UserInputDevice.Codriver,
            AnalogAxis.XBONE_LSY,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.of(Shift.CodriverDebug),
            ElectronicsConstants.INVERT_XBONE_LEFT_Y_AXIS,
            -0.1,
            0.1),
        new AnalogOperationDescription(
            AnalogOperation.ElevatorPositionAdjustment,
            UserInputDevice.Codriver,
            AnalogAxis.XBONE_LSY,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.noneOf(Shift.class),
            ElectronicsConstants.INVERT_XBONE_LEFT_Y_AXIS,
            -0.1,
            0.1),
        new AnalogOperationDescription(
            AnalogOperation.AlgaeWristPower,
            UserInputDevice.Codriver,
            AnalogAxis.XBONE_RSY,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.of(Shift.CodriverDebug),
            ElectronicsConstants.INVERT_XBONE_RIGHT_Y_AXIS,
            -0.1,
            0.1),
        new AnalogOperationDescription(
            AnalogOperation.AlgaeWristAngleAdjustment,
            UserInputDevice.Codriver,
            AnalogAxis.XBONE_RSY,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.noneOf(Shift.class),
            ElectronicsConstants.INVERT_XBONE_RIGHT_Y_AXIS,
            -0.1,
            0.1),

        // apply default "null" values for angle/position control
        new AnalogOperationDescription(
            AnalogOperation.CoralWristDesiredAngle,
            TuningConstants.MAGIC_NULL_VALUE),
        new AnalogOperationDescription(
            AnalogOperation.AlgaeWristDesiredAngle,
            TuningConstants.MAGIC_NULL_VALUE),
        new AnalogOperationDescription(
            AnalogOperation.ElevatorDesiredPosition,
            TuningConstants.MAGIC_NULL_VALUE),
        new AnalogOperationDescription(
            AnalogOperation.ClimberElbowDesiredAngle,
            TuningConstants.MAGIC_NULL_VALUE),

        // testing functions
        new AnalogOperationDescription(
            AnalogOperation.ClimberElbowPower,
            UserInputDevice.Test1,
            AnalogAxis.XBONE_RSY,
            EnumSet.of(Shift.Test1Debug),
            EnumSet.of(Shift.Test1Debug),
            ElectronicsConstants.INVERT_XBONE_RIGHT_Y_AXIS,
            -0.1,
            0.1),
        new AnalogOperationDescription(
            AnalogOperation.ClimberElbowAngleAdjustment,
            UserInputDevice.Test1,
            AnalogAxis.XBONE_RSY,
            EnumSet.of(Shift.Test1Debug),
            EnumSet.noneOf(Shift.class),
            ElectronicsConstants.INVERT_XBONE_RIGHT_Y_AXIS,
            -0.1,
            0.1),
    };

    public static DigitalOperationDescription[] DigitalOperationSchema = new DigitalOperationDescription[]
    {
        // driving operations
        new DigitalOperationDescription(
            DigitalOperation.PositionResetFieldOrientation,
            UserInputDevice.Driver,
            0, // DPAD-up
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Click),
        // new DigitalOperationDescription(
            // DigitalOperation.DriveTrainReset,
            // UserInputDevice.Driver,
            // UserInputDeviceButton.XBONE_Y_BUTTON,
            // EnumSet.of(Shift.DriverDebug),
            // EnumSet.noneOf(Shift.class),
            // ButtonType.Click),
        // new DigitalOperationDescription(
            // DigitalOperation.DriveTrainEnableFieldOrientation,
            // UserInputDevice.Driver,
            // 270,
            // EnumSet.of(Shift.DriverDebug),
            // EnumSet.noneOf(Shift.class),
            // ButtonType.Click),
        // new DigitalOperationDescription(
            // DigitalOperation.DriveTrainDisableFieldOrientation,
            // UserInputDevice.Driver,
            // 270,
            // EnumSet.of(Shift.DriverDebug),
            // EnumSet.of(Shift.DriverDebug),
            // ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainSlowMode,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_RIGHT_STICK_BUTTON,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            ButtonType.Simple),

        new DigitalOperationDescription(
            DigitalOperation.ElevatorEnablePID,
            UserInputDevice.Codriver,
            270,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.ElevatorDisablePID,
            UserInputDevice.Codriver,
            270,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.of(Shift.CodriverDebug),
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.CoralWristEnableSimpleMode,
            UserInputDevice.Codriver,
            90,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.CoralWristDisableSimpleMode,
            UserInputDevice.Codriver,
            90,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.of(Shift.CodriverDebug),
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.CoralEnableThroughBeam,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_START_BUTTON,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.CoralDisableThroughBeam,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_START_BUTTON,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.of(Shift.CodriverDebug),
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.AlgaeWristEnableSimpleMode,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_SELECT_BUTTON,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.AlgaeWristDisableSimpleMode,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_SELECT_BUTTON,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.of(Shift.CodriverDebug),
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.CoalWristResetPosition,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_B_BUTTON,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.of(Shift.CodriverDebug),
            ButtonType.Simple),
    
        // testing operations:
        new DigitalOperationDescription(
            DigitalOperation.VisionFindAnyAprilTag,
            UserInputDevice.Test1,
            0,
            EnumSet.of(Shift.Test1Debug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.VisionFindReefTagsOnly,
            UserInputDevice.Test1,
            0,
            EnumSet.of(Shift.Test1Debug),
            EnumSet.of(Shift.Test1Debug),
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.CoralIntake,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_A_BUTTON,
            EnumSet.of(Shift.Test1Debug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.CoralPlace,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_A_BUTTON,
            EnumSet.of(Shift.Test1Debug),
            EnumSet.of(Shift.Test1Debug),
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.CoralReverse,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_B_BUTTON,
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.AlgaeIntake,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_X_BUTTON,
            EnumSet.of(Shift.Test1Debug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.AlgaeOuttake,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_X_BUTTON,
            EnumSet.of(Shift.Test1Debug),
            EnumSet.of(Shift.Test1Debug),
            ButtonType.Simple),

        // Algae intake operations
        new DigitalOperationDescription(
            DigitalOperation.ClimberWinchClimb,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            ButtonType.Simple),
        // new DigitalOperationDescription(
        //     DigitalOperation.AlgaeWristEnableTMPMode,
        //     UserInputDevice.Codriver,
        //     UserInputDeviceButton.XBONE_X_BUTTON,
        //     ButtonType.Simple),
    };

    public static MacroOperationDescription[] MacroSchema = new MacroOperationDescription[]
    {
        // driving macros
        new MacroOperationDescription(
            MacroOperation.PIDHeavyBrake,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_START_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Simple,
            () -> new PIDBrakeTask(true),
            new IOperation[]
            {
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
            }),
        new MacroOperationDescription(
            MacroOperation.AlignCoralStationLeft,
            UserInputDevice.Driver,
            270,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> new FieldOrientationTask(DesiredOrientation.CoralStationLeft),
            new IOperation[]
            {
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
            }),
        new MacroOperationDescription(
            MacroOperation.AlignCoralStationRight,
            UserInputDevice.Driver,
            90,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> new FieldOrientationTask(DesiredOrientation.CoralStationRight),
            new IOperation[]
            {
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
            }),

        // coral control
        new MacroOperationDescription(
            MacroOperation.CoralIntake,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_RIGHT_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Simple,
            () -> ConcurrentTask.AllTasks(
                new CoralIntakeControlTask(CoralResult.Intake),
                new AlgaeDecisionTask(
                    false,
                    new ElevatorPositionBasedDecisionTask(
                        5.0,
                        new AlgaeIntakeControlTask(AlgaeResult.Intake, true),
                        new WaitTask(0.0)))),
            new IOperation[]
            {
                DigitalOperation.CoralIntake,
                DigitalOperation.CoralIntakeSlow,
                DigitalOperation.CoralPlace,
                DigitalOperation.CoralPlaceSlow,
                DigitalOperation.CoralReverse,
                DigitalOperation.CoralReverseSlow,
                DigitalOperation.AlgaeIntake,
                DigitalOperation.AlgaeOuttake,
                DigitalOperation.AlgaeLaunch
            }),
        new MacroOperationDescription(
            MacroOperation.CoralPlace,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_RIGHT_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Simple,
            () -> new ElevatorPositionBasedDecisionTask(
                TuningConstants.ELEVATOR_CORAL_L1_HEIGHT + TuningConstants.ELEVATOR_POSITION_TOLERANCE,
                new CoralIntakeControlTask(CoralResult.PlaceSlow),
                new CoralIntakeControlTask(CoralResult.Place)),
            new IOperation[]
            {
                DigitalOperation.CoralIntake,
                DigitalOperation.CoralIntakeSlow,
                DigitalOperation.CoralPlace,
                DigitalOperation.CoralPlaceSlow,
                DigitalOperation.CoralReverse,
                DigitalOperation.CoralReverseSlow
            }),
        new MacroOperationDescription(
            MacroOperation.CoralReverse,
            UserInputDevice.Driver,
            180,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            ButtonType.Simple,
            () -> new CoralIntakeControlTask(CoralResult.Reverse),
            new IOperation[]
            {
                DigitalOperation.CoralIntake,
                DigitalOperation.CoralIntakeSlow,
                DigitalOperation.CoralPlace,
                DigitalOperation.CoralPlaceSlow,
                DigitalOperation.CoralReverse,
                DigitalOperation.CoralReverseSlow
            }),

        // algae control
        new MacroOperationDescription(
            MacroOperation.AlgaeIntake,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_LEFT_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Simple,
            () -> new AlgaeIntakeControlTask(AlgaeResult.Intake, true),
            new IOperation[]
            {
                DigitalOperation.AlgaeIntake,
                DigitalOperation.AlgaeOuttake,
                DigitalOperation.AlgaeLaunch
            }),
        new MacroOperationDescription(
            MacroOperation.AlgaeOuttake,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_LEFT_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Simple,
            () -> new AlgaeIntakeControlTask(AlgaeResult.Outtake, true),
            new IOperation[]
            {
                DigitalOperation.AlgaeIntake,
                DigitalOperation.AlgaeOuttake,
                DigitalOperation.AlgaeLaunch
            }),

        // climber control
        new MacroOperationDescription(
            MacroOperation.ClimberOut,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Toggle,
            () -> new ClimberElbowSetPositionTask(TuningConstants.CLIMBER_ELBOW_OUT_ANGLE, true),
            new IOperation[]
            {
                DigitalOperation.ClimberElbowEnableSimpleMode,
                DigitalOperation.ClimberElbowDisableSimpleMode,
                AnalogOperation.ClimberElbowPower,
                AnalogOperation.ClimberElbowDesiredAngle,
                AnalogOperation.ClimberElbowAngleAdjustment,
            }),
        new MacroOperationDescription(
            MacroOperation.ClimberIn,
            UserInputDevice.Driver,
            0,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> new ClimberElbowSetPositionTask(TuningConstants.CLIMBER_ELBOW_INITIAL_ANGLE, true),
            new IOperation[]
            {
                DigitalOperation.ClimberElbowEnableSimpleMode,
                DigitalOperation.ClimberElbowDisableSimpleMode,
                AnalogOperation.ClimberElbowPower,
                AnalogOperation.ClimberElbowDesiredAngle,
                AnalogOperation.ClimberElbowAngleAdjustment,
            }),
        new MacroOperationDescription(
            MacroOperation.ClimberWinch,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Simple,
            () -> new ClimberWinchTask(ClimbMode.Normal),
            new IOperation[]
            {
                DigitalOperation.ClimberWinchClimb,
                DigitalOperation.ClimberElbowEnableSimpleMode,
                DigitalOperation.ClimberElbowDisableSimpleMode,
                AnalogOperation.ClimberElbowPower,
                AnalogOperation.ClimberElbowDesiredAngle,
                AnalogOperation.ClimberElbowAngleAdjustment,
            }),
        new MacroOperationDescription(
            MacroOperation.ClimberWinchExtra,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_A_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Simple,
            () -> new ClimberWinchTask(ClimbMode.Extra),
            new IOperation[]
            {
                DigitalOperation.ClimberWinchClimb,
                DigitalOperation.ClimberElbowEnableSimpleMode,
                DigitalOperation.ClimberElbowDisableSimpleMode,
                AnalogOperation.ClimberElbowPower,
                AnalogOperation.ClimberElbowDesiredAngle,
                AnalogOperation.ClimberElbowAngleAdjustment,
            }),
        new MacroOperationDescription(
            MacroOperation.ClimberWinchDestroy,
            UserInputDevice.Driver,
            180,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Simple,
            () -> new ClimberWinchTask(ClimbMode.Destroy),
            new IOperation[]
            {
                DigitalOperation.ClimberWinchClimb,
                DigitalOperation.ClimberElbowEnableSimpleMode,
                DigitalOperation.ClimberElbowDisableSimpleMode,
                AnalogOperation.ClimberElbowPower,
                AnalogOperation.ClimberElbowDesiredAngle,
                AnalogOperation.ClimberElbowAngleAdjustment,
            }),
    
        new MacroOperationDescription(
            MacroOperation.VisionApproachReefLeftPeg,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_X_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Toggle,
            () -> new FallbackTask(
                new VisionApproachAprilTagTask(
                    TuningConstants.VISION_REEF_TAG_FORWARD_OFFSET_VALUE,
                    TuningConstants.VISION_REEF_TAG_HORIZONTAL_LEFT_OFFSET_VALUE, 
                    DigitalOperation.VisionFindReefTagsOnly),
                new RumbleTask(0.5)),
            new IOperation[]
            {
                DigitalOperation.PositionResetFieldOrientation,
                DigitalOperation.PositionResetRobotLevel,
                AnalogOperation.PositionStartingAngle,
                DigitalOperation.DriveTrainResetXYPosition,
                DigitalOperation.DriveTrainEnableMaintainDirectionMode,
                AnalogOperation.DriveTrainStartingXPosition,
                AnalogOperation.DriveTrainStartingYPosition,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionEnableStream,
                DigitalOperation.VisionFindAnyAprilTag,
                DigitalOperation.VisionFindAbsolutePosition,
                DigitalOperation.VisionForceDisable,
                DigitalOperation.VisionFindSpecificAprilTag,
                DigitalOperation.VisionFindReefTagsOnly,
                DigitalOperation.VisionFindReefRightFarTagOnly,
                DigitalOperation.VisionFindReefLeftFarTagOnly,
                DigitalOperation.VisionFindReefRightCloseTagOnly,
                DigitalOperation.VisionFindReefLeftCloseTagOnly,
                DigitalOperation.VisionFindReefCenterCloseTagOnly,
                DigitalOperation.VisionFindReefCenterFarTagOnly,
                DigitalOperation.ForceLightDriverRumble,
            }),
        new MacroOperationDescription(
            MacroOperation.VisionApproachReefRightPeg,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_X_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> new FallbackTask(
                new VisionApproachAprilTagTask(
                    TuningConstants.VISION_REEF_TAG_FORWARD_OFFSET_VALUE + 1,
                    TuningConstants.VISION_REEF_TAG_HORIZONTAL_RIGHT_OFFSET_VALUE,
                    DigitalOperation.VisionFindReefTagsOnly),
                new RumbleTask(0.5)),
            new IOperation[]
            {
                DigitalOperation.PositionResetFieldOrientation,
                DigitalOperation.PositionResetRobotLevel,
                AnalogOperation.PositionStartingAngle,
                DigitalOperation.DriveTrainResetXYPosition,
                DigitalOperation.DriveTrainEnableMaintainDirectionMode,
                AnalogOperation.DriveTrainStartingXPosition,
                AnalogOperation.DriveTrainStartingYPosition,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionEnableStream,
                DigitalOperation.VisionFindAnyAprilTag,
                DigitalOperation.VisionFindAbsolutePosition,
                DigitalOperation.VisionForceDisable,
                DigitalOperation.VisionFindSpecificAprilTag,
                DigitalOperation.VisionFindReefTagsOnly,
                DigitalOperation.VisionFindReefRightFarTagOnly,
                DigitalOperation.VisionFindReefLeftFarTagOnly,
                DigitalOperation.VisionFindReefRightCloseTagOnly,
                DigitalOperation.VisionFindReefLeftCloseTagOnly,
                DigitalOperation.VisionFindReefCenterCloseTagOnly,
                DigitalOperation.VisionFindReefCenterFarTagOnly,
                DigitalOperation.ForceLightDriverRumble,
            }),
    
        new MacroOperationDescription(
            MacroOperation.OrientToReef,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_B_BUTTON,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> new FieldOrientationTask(DesiredOrientation.Nearest60Degrees),
            new IOperation[]
            {
                AnalogOperation.DriveTrainTurnAngleGoal,
            }),

        // coral position control
        new MacroOperationDescription(
            MacroOperation.CoralWristStow,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_RIGHT_BUTTON,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_STOW_ANGLE),
            new IOperation[]
            {
                AnalogOperation.CoralWristDesiredAngle,
                AnalogOperation.CoralWristAdjustAngleLeft,
                AnalogOperation.CoralWristAdjustAngleRight,
                AnalogOperation.CoralWristPowerA,
                AnalogOperation.CoralWristPowerB,
            }),

        // algae position control
        new MacroOperationDescription(
            MacroOperation.AlgaeWristStowPosition,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_RIGHT_BUTTON,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.of(Shift.CodriverDebug),
            ButtonType.Toggle,
            () -> new AlgaeWristSetAngleTask(TuningConstants.ALGAE_WRIST_STOW_ANGLE),
            new IOperation[]
            {
                AnalogOperation.AlgaeWristPower,
                AnalogOperation.AlgaeWristAngleAdjustment,
                AnalogOperation.AlgaeWristDesiredAngle,
            }),
        new MacroOperationDescription(
            MacroOperation.AlgaeWristOutPosition,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_A_BUTTON,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.of(Shift.CodriverDebug),
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_STOW_ANGLE),
                new AlgaeWristSetAngleTask(TuningConstants.ALGAE_WRIST_PROCESSOR_ANGLE)),
            new IOperation[]
            {
                AnalogOperation.AlgaeWristPower,
                AnalogOperation.AlgaeWristAngleAdjustment,
                AnalogOperation.AlgaeWristDesiredAngle,
                AnalogOperation.CoralWristDesiredAngle,
                AnalogOperation.CoralWristAdjustAngleLeft,
                AnalogOperation.CoralWristAdjustAngleRight,
                AnalogOperation.CoralWristPowerA,
                AnalogOperation.CoralWristPowerB,
            }),
        new MacroOperationDescription(
            MacroOperation.AlgaeWristBargePosition,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_LEFT_STICK_BUTTON,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.of(Shift.CodriverDebug),
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_STOW_ANGLE),
                new AlgaeWristSetAngleTask(TuningConstants.ALGAE_WRIST_BARGE_ANGLE)),
            new IOperation[]
            {
                AnalogOperation.AlgaeWristPower,
                AnalogOperation.AlgaeWristAngleAdjustment,
                AnalogOperation.AlgaeWristDesiredAngle,
                AnalogOperation.CoralWristDesiredAngle,
                AnalogOperation.CoralWristAdjustAngleLeft,
                AnalogOperation.CoralWristAdjustAngleRight,
                AnalogOperation.CoralWristPowerA,
                AnalogOperation.CoralWristPowerB,
            }),
        
        // elevator ++ controls
        new MacroOperationDescription(
            MacroOperation.ElevatorForceReset,
            UserInputDevice.Codriver,
            180,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.of(Shift.CodriverDebug),
            ButtonType.Click,
            () -> SequentialTask.Sequence(
                ConcurrentTask.AllTasks(
                    new ElevatorPositionTask(TuningConstants.ELEVATOR_RESET_HEIGHT),
                    new AlgaeWristSetAngleTask(TuningConstants.ALGAE_WRIST_STOW_ANGLE),
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE)),
                new ElevatorPositionResetTask()),
            new IOperation[]
            {
                AnalogOperation.ElevatorDesiredPosition,
                AnalogOperation.ElevatorPower,
                AnalogOperation.ElevatorPositionAdjustment,
                AnalogOperation.CoralWristDesiredAngle,
                AnalogOperation.CoralWristAdjustAngleLeft,
                AnalogOperation.CoralWristAdjustAngleRight,
                AnalogOperation.CoralWristPowerA,
                AnalogOperation.CoralWristPowerB,
                AnalogOperation.AlgaeWristPower,
                AnalogOperation.AlgaeWristAngleAdjustment,
                AnalogOperation.AlgaeWristDesiredAngle,
            }),
        new MacroOperationDescription(
            MacroOperation.ElevatorHome,
            UserInputDevice.Codriver,
            180,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> ConcurrentTask.AllTasks(
                new ElevatorPositionTask(TuningConstants.ELEVATOR_RESET_HEIGHT),
                new AlgaeDecisionTask(
                    false,
                    SequentialTask.Sequence(
                        new AlgaePositionBasedDecisionTask(
                            TuningConstants.ALGAE_WRIST_REEF_PICKUP_L34_ANGLE,
                            new WaitTask(0),       
                            new AlgaeWristSetAngleTask(TuningConstants.ALGAE_WRIST_REEF_PICKUP_L34_ANGLE)
                            ),
                        new AlgaeWristSetAngleTask(TuningConstants.ALGAE_WRIST_STOW_ANGLE)
                    )),
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE)),
            new IOperation[] {
                AnalogOperation.ElevatorDesiredPosition,
                AnalogOperation.ElevatorPositionAdjustment,
                AnalogOperation.ElevatorPower,
                AnalogOperation.CoralWristDesiredAngle,
                AnalogOperation.CoralWristAdjustAngleLeft,
                AnalogOperation.CoralWristAdjustAngleRight,
                AnalogOperation.CoralWristPowerA,
                AnalogOperation.CoralWristPowerB,
                AnalogOperation.AlgaeWristPower,
                AnalogOperation.AlgaeWristAngleAdjustment,
                AnalogOperation.AlgaeWristDesiredAngle,
            }),
        new MacroOperationDescription(
            MacroOperation.ElevatorCoralL1Height,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_B_BUTTON,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> ConcurrentTask.AllTasks(
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L1_HEIGHT),
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_L1_OUT_ANGLE)),
            new IOperation[]
            {
                AnalogOperation.ElevatorDesiredPosition,
                AnalogOperation.ElevatorPositionAdjustment,
                AnalogOperation.ElevatorPower,
                AnalogOperation.CoralWristDesiredAngle,
                AnalogOperation.CoralWristAdjustAngleLeft,
                AnalogOperation.CoralWristAdjustAngleRight,
                AnalogOperation.CoralWristPowerA,
                AnalogOperation.CoralWristPowerB,
                AnalogOperation.AlgaeWristPower,
                AnalogOperation.AlgaeWristAngleAdjustment,
                AnalogOperation.AlgaeWristDesiredAngle,
            }),
        new MacroOperationDescription(
            MacroOperation.ElevatorCoralL2Height,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_A_BUTTON,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> ConcurrentTask.AllTasks(
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L2_HEIGHT),
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE)),
            new IOperation[]
            {
                AnalogOperation.ElevatorDesiredPosition,
                AnalogOperation.ElevatorPositionAdjustment,
                AnalogOperation.ElevatorPower,
                AnalogOperation.CoralWristDesiredAngle,
                AnalogOperation.CoralWristAdjustAngleLeft,
                AnalogOperation.CoralWristAdjustAngleRight,
                AnalogOperation.CoralWristPowerA,
                AnalogOperation.CoralWristPowerB,
                AnalogOperation.AlgaeWristPower,
                AnalogOperation.AlgaeWristAngleAdjustment,
                AnalogOperation.AlgaeWristDesiredAngle,
            }),
        new MacroOperationDescription(
            MacroOperation.ElevatorCoralL3Height,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_X_BUTTON,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> ConcurrentTask.AllTasks(
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L3_HEIGHT),
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE)),
            new IOperation[]
            {
                AnalogOperation.ElevatorDesiredPosition,
                AnalogOperation.ElevatorPositionAdjustment,
                AnalogOperation.ElevatorPower,
                AnalogOperation.CoralWristDesiredAngle,
                AnalogOperation.CoralWristAdjustAngleLeft,
                AnalogOperation.CoralWristAdjustAngleRight,
                AnalogOperation.CoralWristPowerA,
                AnalogOperation.CoralWristPowerB,
                AnalogOperation.AlgaeWristPower,
                AnalogOperation.AlgaeWristAngleAdjustment,
                AnalogOperation.AlgaeWristDesiredAngle,
            }),
        new MacroOperationDescription(
            MacroOperation.ElevatorAlgaeL23Height,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_X_BUTTON,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.of(Shift.CodriverDebug),
            ButtonType.Toggle,
            () -> ConcurrentTask.AllTasks(
                new ElevatorPositionTask(TuningConstants.ELEVATOR_ALGAE_L23_HEIGHT),
                new AlgaeWristSetAngleTask(TuningConstants.ALGAE_WRIST_REEF_PICKUP_L23_ANGLE),
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_STOW_ANGLE)),
            new IOperation[]
            {
                AnalogOperation.ElevatorDesiredPosition,
                AnalogOperation.ElevatorPositionAdjustment,
                AnalogOperation.ElevatorPower,
                AnalogOperation.CoralWristDesiredAngle,
                AnalogOperation.CoralWristAdjustAngleLeft,
                AnalogOperation.CoralWristAdjustAngleRight,
                AnalogOperation.CoralWristPowerA,
                AnalogOperation.CoralWristPowerB,
                AnalogOperation.AlgaeWristPower,
                AnalogOperation.AlgaeWristAngleAdjustment,
                AnalogOperation.AlgaeWristDesiredAngle,
            }),
        new MacroOperationDescription(
            MacroOperation.ElevatorCoralL4Height,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> ConcurrentTask.AllTasks(
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT),
               new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE)),
            new IOperation[] {
                AnalogOperation.ElevatorDesiredPosition,
                AnalogOperation.ElevatorPositionAdjustment,
                AnalogOperation.ElevatorPower,
                AnalogOperation.CoralWristDesiredAngle,
                AnalogOperation.CoralWristAdjustAngleLeft,
                AnalogOperation.CoralWristAdjustAngleRight,
                AnalogOperation.CoralWristPowerA,
                AnalogOperation.CoralWristPowerB,
                AnalogOperation.AlgaeWristPower,
                AnalogOperation.AlgaeWristAngleAdjustment,
                AnalogOperation.AlgaeWristDesiredAngle,
            }),
        new MacroOperationDescription(
            MacroOperation.ElevatorAlgaeL34Height,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.of(Shift.CodriverDebug),
            ButtonType.Toggle,
            () -> ConcurrentTask.AllTasks(
                new ElevatorPositionTask(TuningConstants.ELEVATOR_ALGAE_L34_HEIGHT),
                new AlgaeWristSetAngleTask(TuningConstants.ALGAE_WRIST_REEF_PICKUP_L34_ANGLE),
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_STOW_ANGLE)),
            new IOperation[] {
                AnalogOperation.ElevatorDesiredPosition,
                AnalogOperation.ElevatorPositionAdjustment,
                AnalogOperation.ElevatorPower,
                AnalogOperation.CoralWristDesiredAngle,
                AnalogOperation.CoralWristAdjustAngleLeft,
                AnalogOperation.CoralWristAdjustAngleRight,
                AnalogOperation.CoralWristPowerA,
                AnalogOperation.CoralWristPowerB,
                AnalogOperation.AlgaeWristPower,
                AnalogOperation.AlgaeWristAngleAdjustment,
                AnalogOperation.AlgaeWristDesiredAngle,
            }),
        new MacroOperationDescription(
            MacroOperation.ElevatorAlgaeBargeHeight,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_LEFT_STICK_BUTTON,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> ConcurrentTask.AllTasks(
                new ElevatorPositionTask(TuningConstants.ELEVATOR_ALGAE_BARGE_HEIGHT),
                new AlgaeWristSetAngleTask(TuningConstants.ALGAE_WRIST_HOLD_ANGLE),
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_STOW_ANGLE)),
            new IOperation[]
            {
                AnalogOperation.ElevatorDesiredPosition,
                AnalogOperation.ElevatorPositionAdjustment,
                AnalogOperation.ElevatorPower,
                AnalogOperation.CoralWristDesiredAngle,
                AnalogOperation.CoralWristAdjustAngleLeft,
                AnalogOperation.CoralWristAdjustAngleRight,
                AnalogOperation.CoralWristPowerA,
                AnalogOperation.CoralWristPowerB,
                AnalogOperation.AlgaeWristPower,
                AnalogOperation.AlgaeWristAngleAdjustment,
                AnalogOperation.AlgaeWristDesiredAngle,
            }),
        new MacroOperationDescription(
            MacroOperation.AlgaeHoldPosition,
            UserInputDevice.Codriver,
            UserInputDeviceButton.XBONE_RIGHT_STICK_BUTTON,
            EnumSet.of(Shift.CodriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> ConcurrentTask.AllTasks(
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L2_HEIGHT),
                new AlgaeWristSetAngleTask(TuningConstants.ALGAE_WRIST_HOLD_ANGLE)),
            new IOperation[]
            {
                AnalogOperation.ElevatorDesiredPosition,
                AnalogOperation.ElevatorPositionAdjustment,
                AnalogOperation.ElevatorPower,
                AnalogOperation.CoralWristDesiredAngle,
                AnalogOperation.CoralWristAdjustAngleLeft,
                AnalogOperation.CoralWristAdjustAngleRight,
                AnalogOperation.CoralWristPowerA,
                AnalogOperation.CoralWristPowerB,
                AnalogOperation.AlgaeWristPower,
                AnalogOperation.AlgaeWristAngleAdjustment,
                AnalogOperation.AlgaeWristDesiredAngle,
            }),

        new MacroOperationDescription(
            MacroOperation.CheckWheelDiameter,
            UserInputDevice.Test2,
            90,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new FollowPathTask("goForwards30in", Type.RobotRelativeFromCurrentPose)
            ),
            new IOperation[]
            {
                DigitalOperation.PositionResetFieldOrientation,
                DigitalOperation.PositionResetRobotLevel,
                AnalogOperation.PositionStartingAngle,
                DigitalOperation.DriveTrainResetXYPosition,
                DigitalOperation.DriveTrainEnableMaintainDirectionMode,
                AnalogOperation.DriveTrainStartingXPosition,
                AnalogOperation.DriveTrainStartingYPosition,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionEnableStream,
                DigitalOperation.VisionFindAnyAprilTag,
                DigitalOperation.VisionFindAbsolutePosition,
                DigitalOperation.VisionForceDisable,
            }),


        // test operations
        new MacroOperationDescription(
            MacroOperation.FollowPathTest1,
            UserInputDevice.Test2,
            0,
            EnumSet.of(Shift.Test2Debug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new FollowPathTask("goForwards30in", Type.RobotRelativeFromCurrentPose)
            ),
            new IOperation[]
            {
                DigitalOperation.PositionResetFieldOrientation,
                DigitalOperation.PositionResetRobotLevel,
                AnalogOperation.PositionStartingAngle,
                DigitalOperation.DriveTrainResetXYPosition,
                DigitalOperation.DriveTrainEnableMaintainDirectionMode,
                AnalogOperation.DriveTrainStartingXPosition,
                AnalogOperation.DriveTrainStartingYPosition,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionEnableStream,
                DigitalOperation.VisionFindAnyAprilTag,
                DigitalOperation.VisionFindAbsolutePosition,
                DigitalOperation.VisionForceDisable,
            }),

        // Full auton test
        new MacroOperationDescription(
            MacroOperation.FollowPathTest5, // Bottom 3 Coral
            UserInputDevice.Test2,
            UserInputDeviceButton.XBONE_START_BUTTON,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new PositionStartingTask(46 - HardwareConstants.ROBOT_HALF_SIDE_LENGTH, -158.5 + HardwareConstants.ROBOT_HALF_SIDE_LENGTH, 180),
                ConcurrentTask.AllTasks(
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE),
                    new FollowPathTask("B1ToIS6Red", Type.Absolute)
                ),
                ConcurrentTask.AllTasks(
                    new FollowPathTask("IS6ToS6Red", Type.Absolute),
                    new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT)
                ),
                new CoralIntakeControlTask(CoralResult.Place),
                ConcurrentTask.AllTasks(
                    new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT),
                    new FollowPathTask("S6ToCBRed", Type.Absolute)
                ),
                new CoralIntakeControlTask(CoralResult.Intake),
                new FollowPathTask("CBToIS7Red", Type.Absolute),
                ConcurrentTask.AllTasks(
                    new FollowPathTask("IS7ToS7Red", Type.Absolute),
                    new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT)
                ),
                new CoralIntakeControlTask(CoralResult.Place),
                ConcurrentTask.AllTasks(
                    new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT),
                    new FollowPathTask("S7ToCBRed", Type.Absolute)
                ),
                new CoralIntakeControlTask(CoralResult.Intake),
                new FollowPathTask("CBToIS9Red", Type.Absolute),
                ConcurrentTask.AllTasks(
                    new FollowPathTask("IS9ToS9Red", Type.Absolute),
                    new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT)
                ),
                new CoralIntakeControlTask(CoralResult.Place)
            ),
            new IOperation[]
            {
                DigitalOperation.PositionResetFieldOrientation,
                DigitalOperation.PositionResetRobotLevel,
                AnalogOperation.PositionStartingAngle,
                DigitalOperation.DriveTrainResetXYPosition,
                AnalogOperation.DriveTrainStartingXPosition,
                AnalogOperation.DriveTrainStartingYPosition,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionEnableStream,
                DigitalOperation.VisionFindAnyAprilTag,
                DigitalOperation.VisionFindAbsolutePosition,
                DigitalOperation.VisionForceDisable,
                AnalogOperation.ElevatorDesiredPosition,
                AnalogOperation.ElevatorPositionAdjustment,
                AnalogOperation.ElevatorPower,
                AnalogOperation.CoralWristDesiredAngle,
                AnalogOperation.CoralWristAdjustAngleLeft,
                AnalogOperation.CoralWristAdjustAngleRight,
                AnalogOperation.CoralWristPowerA,
                AnalogOperation.CoralWristPowerB,
                AnalogOperation.AlgaeWristPower,
                AnalogOperation.AlgaeWristAngleAdjustment,
                AnalogOperation.AlgaeWristDesiredAngle,
        }),
        
        new MacroOperationDescription(
            MacroOperation.FollowPathTest10, // Bottom 2 Coral both L4
            UserInputDevice.Test2,
            UserInputDeviceButton.XBONE_X_BUTTON,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new PositionStartingTask(46 - HardwareConstants.ROBOT_HALF_SIDE_LENGTH, -158.5 + HardwareConstants.ROBOT_HALF_SIDE_LENGTH, 180.0),
                ConcurrentTask.AllTasks(
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE),
                    new FollowPathTask("B1ToIS6Red", Type.Absolute)
                ),
                ConcurrentTask.AllTasks(
                    new FallbackTask(new VisionApproachAprilTagTask(TuningConstants.VISION_REEF_TAG_FORWARD_OFFSET_VALUE, TuningConstants.VISION_REEF_TAG_HORIZONTAL_LEFT_OFFSET_VALUE, DigitalOperation.VisionFindReefTagsOnly), new FollowPathTask("IS6ToS6Red", Type.Absolute)),
                    new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT)
                ),
                new CoralIntakeControlTask(CoralResult.Place),
                ConcurrentTask.AllTasks(
                    new ElevatorPositionTask(TuningConstants.ELEVATOR_HOME_HEIGHT),
                    new FollowPathTask("S6ToCBRed", Type.Absolute)
                ),
                new CoralIntakeControlTask(CoralResult.Intake),
                new FollowPathTask("CBToIS7Red", Type.Absolute),
                ConcurrentTask.AllTasks(
                    new FollowPathTask("IS7ToS7Red", Type.Absolute),
                    new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT)
                ),
                new CoralIntakeControlTask(CoralResult.Place)
            ),
            new IOperation[]
            {
                DigitalOperation.PositionResetFieldOrientation,
                DigitalOperation.PositionResetRobotLevel,
                AnalogOperation.PositionStartingAngle,
                DigitalOperation.DriveTrainResetXYPosition,
                AnalogOperation.DriveTrainStartingXPosition,
                AnalogOperation.DriveTrainStartingYPosition,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionEnableStream,
                DigitalOperation.VisionFindAnyAprilTag,
                DigitalOperation.VisionFindAbsolutePosition,
                DigitalOperation.VisionForceDisable,
                DigitalOperation.VisionFindSpecificAprilTag,
                DigitalOperation.VisionFindReefTagsOnly,
                DigitalOperation.VisionFindReefRightFarTagOnly,
                DigitalOperation.VisionFindReefLeftFarTagOnly,
                DigitalOperation.VisionFindReefRightCloseTagOnly,
                DigitalOperation.VisionFindReefLeftCloseTagOnly,
                DigitalOperation.VisionFindReefCenterCloseTagOnly,
                DigitalOperation.VisionFindReefCenterFarTagOnly,
                AnalogOperation.ElevatorDesiredPosition,
                AnalogOperation.ElevatorPositionAdjustment,
                AnalogOperation.ElevatorPower,
                AnalogOperation.CoralWristDesiredAngle,
                AnalogOperation.CoralWristAdjustAngleLeft,
                AnalogOperation.CoralWristAdjustAngleRight,
                DigitalOperation.CoralIntake,
                DigitalOperation.CoralIntakeSlow,
                DigitalOperation.CoralReverse,
                DigitalOperation.CoralReverseSlow,
                DigitalOperation.CoralPlace,
                DigitalOperation.CoralPlaceSlow,
                AnalogOperation.CoralWristPowerA,
                AnalogOperation.CoralWristPowerB,
                AnalogOperation.AlgaeWristPower,
                AnalogOperation.AlgaeWristAngleAdjustment,
                AnalogOperation.AlgaeWristDesiredAngle,
        }),

        new MacroOperationDescription(
            MacroOperation.FollowPathTest2, // Bottom Taxi Score
            UserInputDevice.Test2,
            UserInputDeviceButton.XBONE_SELECT_BUTTON,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new PositionStartingTask(46 - HardwareConstants.ROBOT_HALF_SIDE_LENGTH, -158.5 + HardwareConstants.ROBOT_HALF_SIDE_LENGTH, 180),
                ConcurrentTask.AllTasks(
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE),
                    new FollowPathTask("B1ToIS4Red", Type.Absolute)
                ),
                ConcurrentTask.AllTasks(
                    new FollowPathTask("IS4ToS4Red", Type.Absolute),   
                    new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT),
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE)
                ),
                new CoralIntakeControlTask(CoralResult.Place),
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_STOW_ANGLE)

            ),
            new IOperation[]
            {
                DigitalOperation.PositionResetFieldOrientation,
                DigitalOperation.PositionResetRobotLevel,
                AnalogOperation.PositionStartingAngle,
                DigitalOperation.DriveTrainResetXYPosition,
                AnalogOperation.DriveTrainStartingXPosition,
                AnalogOperation.DriveTrainStartingYPosition,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionEnableStream,
                DigitalOperation.VisionFindAnyAprilTag,
                DigitalOperation.VisionFindAbsolutePosition,
                DigitalOperation.VisionForceDisable,
                AnalogOperation.ElevatorDesiredPosition,
                AnalogOperation.ElevatorPositionAdjustment,
                AnalogOperation.ElevatorPower,
                AnalogOperation.CoralWristDesiredAngle,
                AnalogOperation.CoralWristAdjustAngleLeft,
                AnalogOperation.CoralWristAdjustAngleRight,
                AnalogOperation.CoralWristPowerA,
                AnalogOperation.CoralWristPowerB,
                AnalogOperation.AlgaeWristPower,
                AnalogOperation.AlgaeWristAngleAdjustment,
                AnalogOperation.AlgaeWristDesiredAngle,
            }),

        new MacroOperationDescription(
            MacroOperation.FollowPathTest3,
            UserInputDevice.Test2,
            270,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> new FollowPathTask("goLeft22in", Type.RobotRelativeFromCurrentPose),
            new IOperation[]
            {
                DigitalOperation.PositionResetFieldOrientation,
                DigitalOperation.PositionResetRobotLevel,
                AnalogOperation.PositionStartingAngle,
                DigitalOperation.DriveTrainResetXYPosition,
                AnalogOperation.DriveTrainStartingXPosition,
                AnalogOperation.DriveTrainStartingYPosition,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionEnableStream,
                DigitalOperation.VisionFindAnyAprilTag,
                DigitalOperation.VisionFindAbsolutePosition,
                DigitalOperation.VisionForceDisable,
            }),

        new MacroOperationDescription(
            MacroOperation.FollowPathTest4,
            UserInputDevice.Test2,
            UserInputDeviceButton.XBONE_RIGHT_BUTTON,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> new FollowPathTask("goLeft22in", Type.RobotRelativeFromCurrentPose),
            new IOperation[]
            {
                DigitalOperation.PositionResetFieldOrientation,
                DigitalOperation.PositionResetRobotLevel,
                AnalogOperation.PositionStartingAngle,
                DigitalOperation.DriveTrainResetXYPosition,
                AnalogOperation.DriveTrainStartingXPosition,
                AnalogOperation.DriveTrainStartingYPosition,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionEnableStream,
                DigitalOperation.VisionFindAnyAprilTag,
                DigitalOperation.VisionFindAbsolutePosition,
                DigitalOperation.VisionForceDisable,
            }),

        new MacroOperationDescription(
            MacroOperation.FollowPathTest6, // Dead Reckoning Path
            UserInputDevice.Test2,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                ConcurrentTask.AllTasks(
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_STOW_ANGLE),
                    new FollowPathTask("goForwards68in", Type.RobotRelativeFromCurrentPose)
                ),
                new FallbackTask(
                    new VisionApproachAprilTagTask(TuningConstants.VISION_REEF_TAG_FORWARD_OFFSET_VALUE, TuningConstants.VISION_REEF_TAG_HORIZONTAL_LEFT_OFFSET_VALUE, DigitalOperation.VisionFindReefTagsOnly),
                    new FollowPathTask("goForwards20in", Type.RobotRelativeFromCurrentPose)
                ),
                new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT),
                new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE),
                new CoralIntakeControlTask(CoralResult.Place)
            ),
            new IOperation[]
            {
                DigitalOperation.PositionResetFieldOrientation,
                DigitalOperation.PositionResetRobotLevel,
                AnalogOperation.PositionStartingAngle,
                DigitalOperation.DriveTrainResetXYPosition,
                AnalogOperation.DriveTrainStartingXPosition,
                AnalogOperation.DriveTrainStartingYPosition,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionEnableStream,
                DigitalOperation.VisionFindAnyAprilTag,
                DigitalOperation.VisionFindAbsolutePosition,
                DigitalOperation.VisionForceDisable,
                DigitalOperation.VisionFindSpecificAprilTag,
                DigitalOperation.VisionFindReefTagsOnly,
                DigitalOperation.VisionFindReefRightFarTagOnly,
                DigitalOperation.VisionFindReefLeftFarTagOnly,
                DigitalOperation.VisionFindReefRightCloseTagOnly,
                DigitalOperation.VisionFindReefLeftCloseTagOnly,
                DigitalOperation.VisionFindReefCenterCloseTagOnly,
                DigitalOperation.VisionFindReefCenterFarTagOnly,
                AnalogOperation.ElevatorDesiredPosition,
                AnalogOperation.ElevatorPositionAdjustment,
                AnalogOperation.ElevatorPower,
                AnalogOperation.CoralWristDesiredAngle,
                AnalogOperation.CoralWristAdjustAngleLeft,
                AnalogOperation.CoralWristAdjustAngleRight,
                DigitalOperation.CoralIntake,
                DigitalOperation.CoralIntakeSlow,
                DigitalOperation.CoralReverse,
                DigitalOperation.CoralReverseSlow,
                DigitalOperation.CoralPlace,
                DigitalOperation.CoralPlaceSlow,
                AnalogOperation.CoralWristPowerA,
                AnalogOperation.CoralWristPowerB,
                AnalogOperation.AlgaeWristPower,
                AnalogOperation.AlgaeWristAngleAdjustment,
                AnalogOperation.AlgaeWristDesiredAngle,
            }),

        new MacroOperationDescription(
            MacroOperation.FollowPathTest7, // Middle Taxi Score Points
            UserInputDevice.Test2,
            UserInputDeviceButton.XBONE_A_BUTTON,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new PositionStartingTask(46 - HardwareConstants.ROBOT_HALF_SIDE_LENGTH, 0, 180),
                ConcurrentTask.AllTasks(
                    new CoralWristSetAngleTask(TuningConstants.CORAL_WRIST_REGULAR_OUT_ANGLE),
                    new FollowPathTask("B2ToIS1Red", Type.Absolute)
                ),
                ConcurrentTask.AllTasks(
                    new FollowPathTask("IS1ToS1Red", Type.Absolute),
                    new ElevatorPositionTask(TuningConstants.ELEVATOR_CORAL_L4_HEIGHT)
                ),
                new CoralIntakeControlTask(CoralResult.Place)
            ),
            new IOperation[]
            {
                DigitalOperation.PositionResetFieldOrientation,
                DigitalOperation.PositionResetRobotLevel,
                AnalogOperation.PositionStartingAngle,
                DigitalOperation.DriveTrainResetXYPosition,
                AnalogOperation.DriveTrainStartingXPosition,
                AnalogOperation.DriveTrainStartingYPosition,
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainSpinLeft,
                AnalogOperation.DriveTrainSpinRight,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainSteerMode,
                DigitalOperation.DriveTrainMaintainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.DriveTrainUseRobotOrientation,
                DigitalOperation.VisionEnableStream,
                DigitalOperation.VisionFindAnyAprilTag,
                DigitalOperation.VisionFindAbsolutePosition,
                DigitalOperation.VisionForceDisable,
                AnalogOperation.ElevatorDesiredPosition,
                AnalogOperation.ElevatorPower,
                AnalogOperation.ElevatorPositionAdjustment,
                AnalogOperation.CoralWristDesiredAngle,
                AnalogOperation.CoralWristAdjustAngleLeft,
                AnalogOperation.CoralWristAdjustAngleRight,
                AnalogOperation.CoralWristPowerA,
                AnalogOperation.CoralWristPowerB,
                AnalogOperation.AlgaeWristPower,
                AnalogOperation.AlgaeWristAngleAdjustment,
                AnalogOperation.AlgaeWristDesiredAngle,
                DigitalOperation.CoralIntake,
                DigitalOperation.CoralIntakeSlow,
                DigitalOperation.CoralPlace,
                DigitalOperation.CoralPlaceSlow,
                DigitalOperation.CoralReverse,
                DigitalOperation.CoralReverseSlow
            }),
    };

    @Override
    public ShiftDescription[] getShiftSchema()
    {
        return ButtonMap.ShiftButtonSchema;
    }

    @Override
    public AnalogOperationDescription[] getAnalogOperationSchema()
    {
        return ButtonMap.AnalogOperationSchema;
    }

    @Override
    public DigitalOperationDescription[] getDigitalOperationSchema()
    {
        return ButtonMap.DigitalOperationSchema;
    }

    @Override
    public MacroOperationDescription[] getMacroOperationSchema()
    {
        return ButtonMap.MacroSchema;
    }
}

