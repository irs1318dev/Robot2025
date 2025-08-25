package frc.robot;

import frc.lib.robotprovider.MotorType;

/**
 * All constants describing the physical structure of the robot (distances and sizes of things).
 * 
 * @author Will
 * 
 */
public class HardwareConstants
{
    public static final double ROBOT_WEIGHT = 140.0; // total weight in pounds, with bumpers, battery, etc.
    public static final double ROBOT_MOI = 127.037; // square-foot-pounds
    public static final double MAX_ROBOT_HEIGHT = 42.0; // inches, max overall height
    public static final double MAX_ROBOT_EXTENSION = 18.0; // inches, max extension beyond frame perimeter
    public static final double ROBOT_FRAME_DIMENSION = 28.0; // frame perimeter / 4.0
    public static final double ROBOT_BUMPER_WIDTH = 3.0; // inches
    public static final double ROBOT_FULL_SIDE_LENGTH = HardwareConstants.ROBOT_FRAME_DIMENSION + HardwareConstants.ROBOT_BUMPER_WIDTH * 2.0; // ROBOT_FRAME_DIMENSION plus 6.0 (for bumpers)
    public static final double ROBOT_HALF_SIDE_LENGTH = HardwareConstants.ROBOT_FRAME_DIMENSION / 2.0 + HardwareConstants.ROBOT_BUMPER_WIDTH; // inches

    //================================================== SDS DriveTrain ==============================================================

    public static final boolean SDSDRIVETRAIN_STEER_MOTOR1_INVERT = true;
    public static final boolean SDSDRIVETRAIN_STEER_MOTOR2_INVERT = true;
    public static final boolean SDSDRIVETRAIN_STEER_MOTOR3_INVERT = true;
    public static final boolean SDSDRIVETRAIN_STEER_MOTOR4_INVERT = true;

    public static final boolean SDSDRIVETRAIN_DRIVE_MOTOR1_INVERT = false;
    public static final boolean SDSDRIVETRAIN_DRIVE_MOTOR2_INVERT = false;
    public static final boolean SDSDRIVETRAIN_DRIVE_MOTOR3_INVERT = false;
    public static final boolean SDSDRIVETRAIN_DRIVE_MOTOR4_INVERT = false;

    public static final double SDSDRIVETRAIN_STEER_GEAR_RATIO = TuningConstants.COMPETITION_ROBOT ? 18.75 : 150.0 / 7.0; // According to SDS Mk4N: 18.75 : 1
    public static final double SDSDRIVETRAIN_STEER_DEGREES = 360.0;
    public static final double SDSDRIVETRAIN_STEER_TICK_DISTANCE = HardwareConstants.SDSDRIVETRAIN_STEER_DEGREES / HardwareConstants.SDSDRIVETRAIN_STEER_GEAR_RATIO; // in degrees
    public static final double SDSDRIVETRAIN_STEER_TICKS_PER_DEGREE = HardwareConstants.SDSDRIVETRAIN_STEER_GEAR_RATIO / HardwareConstants.SDSDRIVETRAIN_STEER_DEGREES; // in rotations

    public static final double SDSDRIVETRAIN_WHEEL_COEFFICIENT_OF_FRICTION = 1.2; // (unitless)
    public static final MotorType SDSDRIVETRAIN_DRIVE_MOTOR_TYPE = MotorType.KrakenX60;
    public static final int SDSDRIVETRAIN_DRIVE_MOTOR_COUNT = 1;
    public static final double SDSDRIVETRAIN_DRIVE_GEAR_RATIO = 38250.0 / 6480.0; // According to SDS Mk4N L2: (50.0 / 13.0) * (17.0 / 27.0) * (45.0 / 15.0) == ~6.12 : 1
    public static final double SDSDRIVETRAIN_DRIVE_WHEEL_DIAMETER = 3.80; // 3.895 // SDS 4-inch wheels are actually ~3.95 inches
    public static final double SDSDRIVETRAIN_DRIVE_WHEEL_CIRCUMFERENCE = Math.PI * HardwareConstants.SDSDRIVETRAIN_DRIVE_WHEEL_DIAMETER;
    public static final double SDSDRIVETRAIN_DRIVE_TICK_DISTANCE = HardwareConstants.SDSDRIVETRAIN_DRIVE_WHEEL_CIRCUMFERENCE / HardwareConstants.SDSDRIVETRAIN_DRIVE_GEAR_RATIO;
    public static final double SDSDRIVETRAIN_DRIVE_TICKS_PER_INCH = HardwareConstants.SDSDRIVETRAIN_DRIVE_GEAR_RATIO / HardwareConstants.SDSDRIVETRAIN_DRIVE_WHEEL_CIRCUMFERENCE;
    public static final double SDSDRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND = HardwareConstants.SDSDRIVETRAIN_DRIVE_TICK_DISTANCE; // converts rotations/sec into inches per second.
    public static final double SDSDRIVETRAIN_DRIVE_INCHES_PER_SECOND_TO_MOTOR_VELOCITY = HardwareConstants.SDSDRIVETRAIN_DRIVE_TICKS_PER_INCH; // converts inches per second into rotations/sec

    public static final double SDSDRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE = 22.5; // (in inches)
    public static final double SDSDRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE = 22.5; // (in inches)
    public static final double SDSDRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE_INV = 1.0 / HardwareConstants.SDSDRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE; // inverse of the horizontal wheel separation
    public static final double SDSDRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE_INV = 1.0 / HardwareConstants.SDSDRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE; // inverse of the vertical wheel separation
    public static final double SDSDRIVETRAIN_HORIZONTAL_WHEEL_CENTER_DISTANCE = HardwareConstants.SDSDRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE / 2.0; // (in inches)
    public static final double SDSDRIVETRAIN_VERTICAL_WHEEL_CENTER_DISTANCE = HardwareConstants.SDSDRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE / 2.0; // (in inches)

    //=================================================================== Elevator ================================================================

    public static final double ELEVATOR_MIN_HEIGHT = 0.0; // the minimum position of the elevator, in inches
    public static final double ELEVATOR_START_HEIGHT = 0.0; // initial height of the elevator in starting configuration, in inches
    public static final double ELEVATOR_MAX_HEIGHT = 107.25; // the maximum position of the elevator, in inches

    public static final double ELEVATOR_GEAR_RATIO = (12.75) / 3.03; 
    public static final double ELEVATOR_WHEEL_DIAMETER = 1.86; // 1.86076 from cad
    public static final double ELEVATOR_WHEEL_CIRCUMFERENCE = Math.PI * HardwareConstants.ELEVATOR_WHEEL_DIAMETER;
    public static final double ELEVATOR_TICK_DISTANCE = HardwareConstants.ELEVATOR_WHEEL_CIRCUMFERENCE / HardwareConstants.ELEVATOR_GEAR_RATIO;
    public static final double ELEVATOR_TICKS_PER_INCH = HardwareConstants.ELEVATOR_GEAR_RATIO / HardwareConstants.ELEVATOR_WHEEL_CIRCUMFERENCE;

    //==================================================== Coral End Effector =====================================================================

    public static final double CORAL_INTAKE_MOTOR_TICK_DISTANCE = 360.0; // analog encoder is directly on the pivot shaft

    //==================================================== Algae End Effector =====================================================================

    public static final double ALGAE_INTAKE_MOTOR_TICK_DISTANCE = 360.0;

    //==================================================== Climber =====================================================================

    public static final double CLIMBER_ELBOW_ABSOLUTE_ENCODER_TICK_DISTANCE = 360.0;
}
