package frc.robot;

import java.util.List;
import java.util.stream.Collectors;

/**
 * All constants related to tuning the operation of the robot.
 * 
 */
public class TuningConstants
{
    public static final boolean COMPETITION_ROBOT = true;
    public static final boolean USE_ADVANTAGE_KIT = true;
    public static final boolean READ_SENSORS_WHILE_DISABLED = true;
    public static final boolean LOG_NULL_WHILE_DISABLED = true && !TuningConstants.READ_SENSORS_WHILE_DISABLED;
    public static final boolean RETREIVE_PDH_FIRST = true;
    public static final boolean TRACER_ENABLED = false;

    public static boolean THROW_EXCEPTIONS = false;
    public static boolean LOG_EXCEPTIONS = true;
    public static double LOOP_DURATION = 0.02; // we expect the robot's main loop to run at roughly ~50 Hz, or 1 update per 20ms (0.02s)
    public static int LOOPS_PER_SECOND = 50; // we expect the robot's main loop to run at roughly ~50 Hz, or 1 update per 20ms (0.02s)

    public static final boolean EXPECT_UNUSED_JOYSTICKS = true;
    public static final boolean PERFORM_COSTLY_TASKS_WHILE_DISABLED = true;

    //================================================== Magic Values ==============================================================

    public static final double MAGIC_NULL_VALUE = -1318.0;
    public static final double ZERO = 0.0;
    public static final double ENDGAME_START_TIME = 30.0;
    public static final double ENDGAME_CLIMB_TIME = 5.0;

    //================================================== Logging  ==============================================================

    public static final int CALENDAR_YEAR = 2025;
    public static final boolean LOG_TO_FILE = false; // TuningConstants.COMPETITION_ROBOT;
    public static final boolean LOG_FILE_ONLY_COMPETITION_MATCHES = false;
    public static final long LOG_FILE_REQUIRED_FREE_SPACE = 50 * 1024 * 1024; // require at least 50 MB of space
    public static final int LOG_FLUSH_THRESHOLD = 25;
    public static final boolean USE_LOGGING_FREQUENCY = true; // TuningConstants.COMPETITION_ROBOT;
    public static final int DEFAULT_LOGGING_FREQUENCY = 10; // number of entries to ignore between logging

    //================================================== Autonomous ==============================================================

    public static final boolean TRAJECTORY_FORCE_BUILD = false;
    public static final double AUTO_3_CORAL_DO_LAST_CORAL_TIME_THRESHOLD = 3.2; // seconds
    public static final double AUTO_TIME_TO_RAISE_ELEVATOR = 1.0;

    //================================================= Power ======================================================

    public static final double POWER_OVERCURRENT_TRACKING_DURATION = 5.0; // duration of time to keep track of the average current
    public static final double POWER_OVERCURRENT_TRACKING_MIN_VALUE = 0.0; // 1000 amos is an unrealistic max value to use for overcurrent
    public static final double POWER_OVERCURRENT_TRACKING_MAX_VALUE = 1000.0; // 1000 amos is an unrealistic max value to use for overcurrent
    public static final double POWER_OVERCURRENT_SAMPLES_PER_LOOP = 1.0; // we may want to increase this if we find our update loop duration isn't very consistent...
    public static final double POWER_OVERCURRENT_SAMPLES_PER_SECOND = TuningConstants.LOOPS_PER_SECOND * TuningConstants.POWER_OVERCURRENT_SAMPLES_PER_LOOP;
    public static final double POWER_OVERCURRENT_THRESHOLD = 140.0;
    public static final double POWER_OVERCURREHT_HIGH_THRESHOLD = 180.0;

    //================================================= Macros/Vision ======================================================

    public static final boolean VISION_USE_SCORE = true;
    public static final boolean VISION_USE_PRECEDENCE = true;
    public static final double VISION_ODOMETRY_ACCURACY_TRESHOLD_RANGE = 30;

    //=========================================== 2025 AprilTag Location guide ==============================================
     //// | TAG                                 |  ID  |    X    |    Y     |    Z   |   YAW   |  PITCH  |
    //// | ------------------------------------------- RED ---------------------------------- | ------- |
    //// | APRILTAG_RED_CORALSTATION_LEFT_ID   |   1  |  331.74 |  -132.38 |  58.50 |  126.0  |    0.0  |
    //// | APRILTAG_RED_CORALSTATION_RIGHT_ID  |   2  |  331.74 |   133.20 |  58.50 |  234.0  |    0.0  |
    //// | APRILTAG_RED_PROCESSOR_ID           |   3  |  129.52 |   159.15 |  51.25 |  270.0  |    0.0  |
    //// | APRILTAG_RED_START_RIGHT_ID         |   4  |   39.57 |   83.64  |  73.54 |    0.0  |   30.0  |
    //// | APRILTAG_RED_START_LEFT_ID          |   5  |   39.57 |  -82.61  |  73.54 |    0.0  |   30.0  |
    //// | APRILTAG_RED_REEF_LEFT_CLOSE_ID     |   6  |  204.86 |  -27.61  |  12.13 |  300.0  |    0.0  |
    //// | APRILTAG_RED_REEF_CENTER_CLOSE_ID   |   7  |  220.23 |   0.50   |  12.13 |    0.0  |    0.0  |
    //// | APRILTAG_RED_REEF_RIGHT_CLOSE_ID    |   8  |  204.86 |   28.83  |  12.13 |    60.0 |    0.0  |
    //// | APRILTAG_RED_REEF_RIGHT_FAR_ID      |   9  |  172.15 |   28.83  |  12.13 |   120.0 |    0.0  |
    //// | APRILTAG_RED_REEF_CENTER_FAR_ID     |  10  |  145.76 |   0.0    |  12.13 |   180.0 |    0.0  |
    //// | APRILTAG_RED_REEF_LEFT_FAR_ID       |  11  |  171.13 |  -15.23  |  12.13 |   240.0 |    0.0  |
    //// | ------------------------------------------- BLUE --------------------------------- | ------- |
    //// | APRILTAG_BLUE_CORALSTATION_RIGHT_ID |  12  | -292.12 |   -25.30 |  58.50 |    54.0 |    0.0  |
    //// | APRILTAG_BLUE_CORALSTATION_LEFT_ID  |  13  | -292.12 |   -16.50 |  58.50 |   306.0 |    0.0  |
    //// | APRILTAG_BLUE_START_LEFT_ID         |  14  | -300.97 |   -19.29 |  73.54 |   180.0 |  -30.0  |
    //// | APRILTAG_BLUE_START_RIGHT_ID        |  15  | -300.97 |  -82.61  |  73.54 |   180.0 |  -30.0  |
    //// | APRILTAG_BLUE_PROCESSOR_ID          |  16  | -201.13 |   -27.61 |  12.13 |   240.0 |    0.0  |
    //// | APRILTAG_BLUE_REEF_RIGHT_CLOSE_ID   |  17  | -184.75 |   0.0    |  12.13 |     0.0 |    0.0  |
    //// | APRILTAG_BLUE_REEF_CENTER_CLOSE_ID  |  18  | -201.13 |   -27.61 |  12.13 |   240.0 |    0.0  |
    //// | APRILTAG_BLUE_REEF_LEFT_CLOSE_ID    |  19  | -184.75 |  -58.23  |  12.13 |   240.0 |    0.0  |
    //// | APRILTAG_BLUE_REEF_LEFT_FAR_ID      |  20  | -184.75 |  -58.23  |  12.13 |   240.0 |    0.0  |
    //// | APRILTAG_BLUE_REEF_CENTER_FAR_ID    |  21  | -184.75 |  -58.23  |  12.13 |   240.0 |    0.0  |
    //// | APRILTAG_BLUE_REEF_RIGHT_FAR_ID     |  22  | -184.75 |  -58.23  |  12.13 |   240.0 |    0.0  |
    //// 
    //// Conversion from FIRST's published values: (x - 325.615, y - ~3.42, z, rot)

    public static final int APRILTAG_RED_CORALSTATION_LEFT_ID = 1;
    public static final int APRILTAG_RED_CORALSTATION_RIGHT_ID = 2;
    public static final int APRILTAG_RED_PROCESSOR_ID = 3;
    public static final int APRILTAG_RED_START_RIGHT_ID = 4;
    public static final int APRILTAG_RED_START_LEFT_ID = 5;
    public static final int APRILTAG_RED_REEF_LEFT_CLOSE_ID = 6;
    public static final int APRILTAG_RED_REEF_CENTER_CLOSE_ID = 7;
    public static final int APRILTAG_RED_REEF_RIGHT_CLOSE_ID = 8;
    public static final int APRILTAG_RED_REEF_RIGHT_FAR_ID = 9;
    public static final int APRILTAG_RED_REEF_CENTER_FAR_ID = 10;
    public static final int APRILTAG_RED_REEF_LEFT_FAR_ID = 11;

    public static final double APRILTAG_RED_CORALSTATION_LEFT_X_POSITION = 331.74;
    public static final double APRILTAG_RED_CORALSTATION_RIGHT_X_POSITION = 331.74;
    public static final double APRILTAG_RED_PROCESSOR_X_POSITION = 129.52;
    public static final double APRILTAG_RED_START_RIGHT_X_POSITION = 39.57;
    public static final double APRILTAG_RED_START_LEFT_X_POSITION = 39.57;
    public static final double APRILTAG_RED_REEF_LEFT_CLOSE_X_POSITION = 204.86;
    public static final double APRILTAG_RED_REEF_CENTER_CLOSE_X_POSITION = 220.23;
    public static final double APRILTAG_RED_REEF_RIGHT_CLOSE_X_POSITION = 204.86;
    public static final double APRILTAG_RED_REEF_RIGHT_FAR_X_POSITION = 172.15;
    public static final double APRILTAG_RED_REEF_CENTER_FAR_X_POSITION = 145.76;
    public static final double APRILTAG_RED_REEF_LEFT_FAR_X_POSITION = 171.13;

    public static final double APRILTAG_RED_CORALSTATION_LEFT_Y_POSITION = -132.38;
    public static final double APRILTAG_RED_CORALSTATION_RIGHT_Y_POSITION = 133.20;
    public static final double APRILTAG_RED_PROCESSOR_Y_POSITION = 159.15;
    public static final double APRILTAG_RED_START_RIGHT_Y_POSITION = 83.64;
    public static final double APRILTAG_RED_START_LEFT_Y_POSITION = -82.61;
    public static final double APRILTAG_RED_REEF_LEFT_CLOSE_Y_POSITION = -27.61;
    public static final double APRILTAG_RED_REEF_CENTER_CLOSE_Y_POSITION = 0.50;
    public static final double APRILTAG_RED_REEF_RIGHT_CLOSE_Y_POSITION = 28.83;
    public static final double APRILTAG_RED_REEF_RIGHT_FAR_Y_POSITION = 28.83;
    public static final double APRILTAG_RED_REEF_CENTER_FAR_Y_POSITION = 0.0;
    public static final double APRILTAG_RED_REEF_LEFT_FAR_Y_POSITION = -15.23;

    public static final double APRILTAG_RED_CORALSTATION_LEFT_Z_POSITION = 58.50;
    public static final double APRILTAG_RED_CORALSTATION_RIGHT_Z_POSITION = 58.50;
    public static final double APRILTAG_RED_PROCESSOR_Z_POSITION = 51.25;
    public static final double APRILTAG_RED_START_RIGHT_Z_POSITION = 73.54;
    public static final double APRILTAG_RED_START_LEFT_Z_POSITION = 73.54;
    public static final double APRILTAG_RED_REEF_LEFT_CLOSE_Z_POSITION = 12.13;
    public static final double APRILTAG_RED_REEF_CENTER_CLOSE_Z_POSITION = 12.13;
    public static final double APRILTAG_RED_REEF_RIGHT_CLOSE_Z_POSITION = 12.13;
    public static final double APRILTAG_RED_REEF_RIGHT_FAR_Z_POSITION = 12.13;
    public static final double APRILTAG_RED_REEF_CENTER_FAR_Z_POSITION = 12.13;
    public static final double APRILTAG_RED_REEF_LEFT_FAR_Z_POSITION = 12.13;

    public static final double APRILTAG_RED_CORALSTATION_LEFT_YAW = 126.0;
    public static final double APRILTAG_RED_CORALSTATION_RIGHT_YAW = 234.0 ;
    public static final double APRILTAG_RED_PROCESSOR_YAW = 270.0;
    public static final double APRILTAG_RED_START_RIGHT_YAW = 0.0;
    public static final double APRILTAG_RED_START_LEFT_YAW = 0.0;
    public static final double APRILTAG_RED_REEF_LEFT_CLOSE_YAW = 300.0;
    public static final double APRILTAG_RED_REEF_CENTER_CLOSE_YAW = 0.0;
    public static final double APRILTAG_RED_REEF_RIGHT_CLOSE_YAW = 60.0;
    public static final double APRILTAG_RED_REEF_RIGHT_FAR_YAW = 120.0;
    public static final double APRILTAG_RED_REEF_CENTER_FAR_YAW = 180.0;
    public static final double APRILTAG_RED_REEF_LEFT_FAR_YAW = 240.0;

    public static final double APRILTAG_RED_CORALSTATION_LEFT_PITCH = 0.0;
    public static final double APRILTAG_RED_CORALSTATION_RIGHT_PITCH = 0.0;
    public static final double APRILTAG_RED_PROCESSOR_PITCH = 0.0;
    public static final double APRILTAG_RED_START_RIGHT_PITCH = 30.0;
    public static final double APRILTAG_RED_START_LEFT_PITCH = 30.0;
    public static final double APRILTAG_RED_REEF_LEFT_CLOSE_PITCH = 0.0;
    public static final double APRILTAG_RED_REEF_CENTER_CLOSE_PITCH = 0.0;
    public static final double APRILTAG_RED_REEF_RIGHT_CLOSE_PITCH = 0.0;
    public static final double APRILTAG_RED_REEF_RIGHT_FAR_PITCH = 0.0;
    public static final double APRILTAG_RED_REEF_CENTER_FAR_PITCH = 0.0;
    public static final double APRILTAG_RED_REEF_LEFT_FAR_PITCH = 0.0;


    public static final int APRILTAG_BLUE_CORALSTATION_LEFT_ID = 12;
    public static final int APRILTAG_BLUE_CORALSTATION_RIGHT_ID = 13;
    public static final int APRILTAG_BLUE_PROCESSOR_ID = 14;
    public static final int APRILTAG_BLUE_START_RIGHT_ID= 15;
    public static final int APRILTAG_BLUE_START_LEFT_ID = 16;
    public static final int APRILTAG_BLUE_REEF_RIGHT_CLOSE_ID = 17;
    public static final int APRILTAG_BLUE_REEF_CENTER_CLOSE_ID = 18;
    public static final int APRILTAG_BLUE_REEF_LEFT_CLOSE_ID = 19;
    public static final int APRILTAG_BLUE_REEF_LEFT_FAR_ID = 20;
    public static final int APRILTAG_BLUE_REEF_CENTER_FAR_ID = 21;
    public static final int APRILTAG_BLUE_REEF_RIGHT_FAR_ID = 22;

    public static final double APRILTAG_BLUE_CORALSTATION_LEFT_X_POSITION = -292.12;
    public static final double APRILTAG_BLUE_CORALSTATION_RIGHT_X_POSITION = -292.12;
    public static final double APRILTAG_BLUE_PROCESSOR_X_POSITION = -300.97;
    public static final double APRILTAG_BLUE_START_RIGHT_X_POSITION = -300.97;
    public static final double APRILTAG_BLUE_START_LEFT_X_POSITION = -201.13 ;
    public static final double APRILTAG_BLUE_REEF_LEFT_CLOSE_X_POSITION = -184.75;
    public static final double APRILTAG_BLUE_REEF_CENTER_CLOSE_X_POSITION = -201.13;
    public static final double APRILTAG_BLUE_REEF_RIGHT_CLOSE_X_POSITION = -184.75;
    public static final double APRILTAG_BLUE_REEF_RIGHT_FAR_X_POSITION = -184.75;
    public static final double APRILTAG_BLUE_REEF_CENTER_FAR_X_POSITION = -184.75;
    public static final double APRILTAG_BLUE_REEF_LEFT_FAR_X_POSITION = -184.75;

    public static final double APRILTAG_BLUE_CORALSTATION_LEFT_Y_POSITION = -25.30;
    public static final double APRILTAG_BLUE_CORALSTATION_RIGHT_Y_POSITION = -16.50;
    public static final double APRILTAG_BLUE_PROCESSOR_Y_POSITION = -19.29;
    public static final double APRILTAG_BLUE_START_RIGHT_Y_POSITION = -82.61;
    public static final double APRILTAG_BLUE_START_LEFT_Y_POSITION = -27.61;
    public static final double APRILTAG_BLUE_REEF_LEFT_CLOSE_Y_POSITION = 0;
    public static final double APRILTAG_BLUE_REEF_CENTER_CLOSE_Y_POSITION = -27.61;
    public static final double APRILTAG_BLUE_REEF_RIGHT_CLOSE_Y_POSITION = -58.23;
    public static final double APRILTAG_BLUE_REEF_RIGHT_FAR_Y_POSITION = -58.23;
    public static final double APRILTAG_BLUE_REEF_CENTER_FAR_Y_POSITION = -58.23;
    public static final double APRILTAG_BLUE_REEF_LEFT_FAR_Y_POSITION = -58.23;

    public static final double APRILTAG_BLUE_CORALSTATION_LEFT_Z_POSITION = 58.50;
    public static final double APRILTAG_BLUE_CORALSTATION_RIGHT_Z_POSITION = 58.50;
    public static final double APRILTAG_BLUE_PROCESSOR_Z_POSITION = 51.25;
    public static final double APRILTAG_BLUE_START_RIGHT_Z_POSITION = 73.54;
    public static final double APRILTAG_BLUE_START_LEFT_Z_POSITION = 73.54;
    public static final double APRILTAG_BLUE_REEF_LEFT_CLOSE_Z_POSITION = 12.13;
    public static final double APRILTAG_BLUE_REEF_CENTER_CLOSE_Z_POSITION = 12.13;
    public static final double APRILTAG_BLUE_REEF_RIGHT_CLOSE_Z_POSITION = 12.13;
    public static final double APRILTAG_BLUE_REEF_RIGHT_FAR_Z_POSITION = 12.13;
    public static final double APRILTAG_BLUE_REEF_CENTER_FAR_Z_POSITION = 12.13;
    public static final double APRILTAG_BLUE_REEF_LEFT_FAR_Z_POSITION = 12.13;

    public static final double APRILTAG_BLUE_CORALSTATION_LEFT_YAW = 126.0;
    public static final double APRILTAG_BLUE_CORALSTATION_RIGHT_YAW = 234.0 ;
    public static final double APRILTAG_BLUE_PROCESSOR_YAW = 270.0;
    public static final double APRILTAG_BLUE_START_RIGHT_YAW = 0.0;
    public static final double APRILTAG_BLUE_START_LEFT_YAW = 0.0;
    public static final double APRILTAG_BLUE_REEF_LEFT_CLOSE_YAW = 300.0;
    public static final double APRILTAG_BLUE_REEF_CENTER_CLOSE_YAW = 0.0;
    public static final double APRILTAG_BLUE_REEF_RIGHT_CLOSE_YAW = 60.0;
    public static final double APRILTAG_BLUE_REEF_RIGHT_FAR_YAW = 120.0;
    public static final double APRILTAG_BLUE_REEF_CENTER_FAR_YAW = 180.0;
    public static final double APRILTAG_BLUE_REEF_LEFT_FAR_YAW = 240.0;

    public static final double APRILTAG_BLUE_CORALSTATION_LEFT_PITCH = 0.0;
    public static final double APRILTAG_BLUE_CORALSTATION_RIGHT_PITCH = 0.0;
    public static final double APRILTAG_BLUE_PROCESSOR_PITCH = 0.0;
    public static final double APRILTAG_BLUE_START_RIGHT_PITCH = -30.0;
    public static final double APRILTAG_BLUE_START_LEFT_PITCH = -30.0;
    public static final double APRILTAG_BLUE_REEF_LEFT_CLOSE_PITCH = 0.0;
    public static final double APRILTAG_BLUE_REEF_CENTER_CLOSE_PITCH = 0.0;
    public static final double APRILTAG_BLUE_REEF_RIGHT_CLOSE_PITCH = 0.0;
    public static final double APRILTAG_BLUE_REEF_RIGHT_FAR_PITCH = 0.0;
    public static final double APRILTAG_BLUE_REEF_CENTER_FAR_PITCH = 0.0;
    public static final double APRILTAG_BLUE_REEF_LEFT_FAR_PITCH = 0.0;

    // Reef all
    public static final List<Integer> VISION_REEF_BLUE_APRILTAGS = List.of(TuningConstants.APRILTAG_BLUE_REEF_RIGHT_CLOSE_ID, TuningConstants.APRILTAG_BLUE_REEF_CENTER_CLOSE_ID, TuningConstants.APRILTAG_BLUE_REEF_LEFT_CLOSE_ID, TuningConstants.APRILTAG_BLUE_REEF_LEFT_FAR_ID, TuningConstants.APRILTAG_BLUE_REEF_CENTER_FAR_ID, TuningConstants.APRILTAG_BLUE_REEF_RIGHT_FAR_ID);
    public static final String VISION_REEF_BLUE_STRING = TuningConstants.VISION_REEF_BLUE_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));
    public static final List<Integer> VISION_REEF_RED_APRILTAGS = List.of(TuningConstants.APRILTAG_RED_REEF_RIGHT_CLOSE_ID, TuningConstants.APRILTAG_RED_REEF_CENTER_CLOSE_ID, TuningConstants.APRILTAG_RED_REEF_LEFT_CLOSE_ID, TuningConstants.APRILTAG_RED_REEF_LEFT_FAR_ID, TuningConstants.APRILTAG_RED_REEF_CENTER_FAR_ID, TuningConstants.APRILTAG_RED_REEF_RIGHT_FAR_ID);
    public static final String VISION_REEF_RED_STRING = TuningConstants.VISION_REEF_RED_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));

    // Reef right far
    public static final List<Integer> VISION_REEF_RIGHT_FAR_BLUE_APRILTAGS = List.of(TuningConstants.APRILTAG_BLUE_REEF_RIGHT_FAR_ID);
    public static final String VISION_REEF_RIGHT_FAR_BLUE_STRING = TuningConstants.VISION_REEF_RIGHT_FAR_BLUE_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));
    public static final List<Integer> VISION_REEF_RIGHT_FAR_RED_APRILTAGS = List.of(TuningConstants.APRILTAG_RED_REEF_RIGHT_FAR_ID);
    public static final String VISION_REEF_RIGHT_FAR_RED_STRING = TuningConstants.VISION_REEF_RIGHT_FAR_RED_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));

    // Reef left far
    public static final List<Integer> VISION_REEF_LEFT_FAR_BLUE_APRILTAGS = List.of(TuningConstants.APRILTAG_BLUE_REEF_LEFT_FAR_ID);
    public static final String VISION_REEF_LEFT_FAR_BLUE_STRING = TuningConstants.VISION_REEF_LEFT_FAR_BLUE_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));
    public static final List<Integer> VISION_REEF_LEFT_FAR_RED_APRILTAGS = List.of(TuningConstants.APRILTAG_RED_REEF_LEFT_FAR_ID);
    public static final String VISION_REEF_LEFT_FAR_RED_STRING = TuningConstants.VISION_REEF_LEFT_FAR_RED_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));

    // Reef center far
    public static final List<Integer> VISION_REEF_CENTER_FAR_BLUE_APRILTAGS = List.of(TuningConstants.APRILTAG_BLUE_REEF_CENTER_FAR_ID);
    public static final String VISION_REEF_CENTER_FAR_BLUE_STRING = TuningConstants.VISION_REEF_CENTER_FAR_BLUE_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));
    public static final List<Integer> VISION_REEF_CENTER_FAR_RED_APRILTAGS = List.of(TuningConstants.APRILTAG_RED_REEF_CENTER_FAR_ID);
    public static final String VISION_REEF_CENTER_FAR_RED_STRING = TuningConstants.VISION_REEF_CENTER_FAR_RED_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));

    // Reef right close
    public static final List<Integer> VISION_REEF_RIGHT_CLOSE_BLUE_APRILTAGS = List.of(TuningConstants.APRILTAG_BLUE_REEF_RIGHT_CLOSE_ID);
    public static final String VISION_REEF_RIGHT_CLOSE_BLUE_STRING = TuningConstants.VISION_REEF_RIGHT_CLOSE_BLUE_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));
    public static final List<Integer> VISION_REEF_RIGHT_CLOSE_RED_APRILTAGS = List.of(TuningConstants.APRILTAG_RED_REEF_RIGHT_CLOSE_ID);
    public static final String VISION_REEF_RIGHT_CLOSE_RED_STRING = TuningConstants.VISION_REEF_RIGHT_CLOSE_RED_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));

    // Reef left close
    public static final List<Integer> VISION_REEF_LEFT_CLOSE_BLUE_APRILTAGS = List.of(TuningConstants.APRILTAG_BLUE_REEF_LEFT_CLOSE_ID);
    public static final String VISION_REEF_LEFT_CLOSE_BLUE_STRING = TuningConstants.VISION_REEF_LEFT_CLOSE_BLUE_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));
    public static final List<Integer> VISION_REEF_LEFT_CLOSE_RED_APRILTAGS = List.of(TuningConstants.APRILTAG_RED_REEF_LEFT_CLOSE_ID);
    public static final String VISION_REEF_LEFT_CLOSE_RED_STRING = TuningConstants.VISION_REEF_LEFT_CLOSE_RED_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));

    // Reef center close
    public static final List<Integer> VISION_REEF_CENTER_CLOSE_BLUE_APRILTAGS = List.of(TuningConstants.APRILTAG_BLUE_REEF_CENTER_CLOSE_ID);
    public static final String VISION_REEF_CENTER_CLOSE_BLUE_STRING = TuningConstants.VISION_REEF_CENTER_CLOSE_BLUE_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));
    public static final List<Integer> VISION_REEF_CENTER_CLOSE_RED_APRILTAGS = List.of(TuningConstants.APRILTAG_RED_REEF_CENTER_CLOSE_ID);
    public static final String VISION_REEF_CENTER_CLOSE_RED_STRING = TuningConstants.VISION_REEF_CENTER_CLOSE_RED_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));

    // Start
    public static final List<Integer> VISION_START_BLUE_APRILTAGS = List.of(TuningConstants.APRILTAG_BLUE_START_LEFT_ID, TuningConstants.APRILTAG_BLUE_START_RIGHT_ID);
    public static final String VISION_START_BLUE_STRING = TuningConstants.VISION_START_BLUE_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));
    public static final List<Integer> VISION_START_RED_APRILTAGS = List.of(TuningConstants.APRILTAG_RED_START_RIGHT_ID, TuningConstants.APRILTAG_RED_START_LEFT_ID);
    public static final String VISION_START_RED_STRING = TuningConstants.VISION_START_RED_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));

    // Processor
    public static final List<Integer> VISION_PROCESSOR_BLUE_APRILTAGS = List.of(TuningConstants.APRILTAG_BLUE_PROCESSOR_ID);
    public static final String VISION_PROCESSOR_BLUE_STRING = TuningConstants.VISION_PROCESSOR_BLUE_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));
    public static final List<Integer> VISION_PROCESSOR_RED_APRILTAGS = List.of(TuningConstants.APRILTAG_RED_PROCESSOR_ID);
    public static final String VISION_PROCESSOR_RED_STRING = TuningConstants.VISION_PROCESSOR_RED_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));

    // Coral Station
    public static final List<Integer> VISION_CORALSTATION_BLUE_APRILTAGS = List.of(TuningConstants.APRILTAG_BLUE_CORALSTATION_RIGHT_ID, TuningConstants.APRILTAG_BLUE_CORALSTATION_LEFT_ID);
    public static final String VISION_CORALSTATION_BLUE_STRING = TuningConstants.VISION_CORALSTATION_BLUE_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));
    public static final List<Integer> VISION_CORALSTATION_RED_APRILTAGS = List.of(TuningConstants.APRILTAG_RED_CORALSTATION_LEFT_ID, TuningConstants.APRILTAG_RED_CORALSTATION_RIGHT_ID);
    public static final String VISION_CORALSTATION_RED_STRING = TuningConstants.VISION_CORALSTATION_RED_APRILTAGS.stream().map((i) -> Integer.toString(i)).collect(Collectors.joining(","));

    // Finding AprilTags to determine if theres enough valid data to translate 
    public static final int TAGS_MISSED_THRESHOLD = 30;
    public static final int TAGS_FOUND_THRESHOLD = 5;
    public static final double ACCEPTABLE_RANGE_IN_X_AND_Y_FOR_ALIGNMENT_TRANSLATE = 1.0; // in inches

    // Acceptable vision centering range values in degrees
    public static final double MAX_PID_TURNING_RANGE_DEGREES = 2.0;

    // How long the robot system must remain centered on the target when using time
    public static final double PID_TURNING_DURATION = 0.75;

    // Acceptable vision distance from tape in inches (as measured by vision system)
    public static final double MAX_VISION_ACCEPTABLE_FORWARD_DISTANCE = 1.75;
    public static final double MAX_VISION_ACCEPTABLE_STRAFE_DISTANCE = 0.7;

    // Acceptable vision distance from tape in angles 
    public static final double MAX_VISION_ACCEPTABLE_MOVING_RR_ANGLE_ERROR = 4.0;

    // PID settings for Centering the robot on a vision target from one stationary place, based on a single sample
    public static final double STATIONARY_SINGLE_TURNING_PID_KP = 0.02;
    public static final double STATIONARY_SINGLE_TURNING_PID_KI = 0.0;
    public static final double STATIONARY_SINGLE_TURNING_PID_KD = 0.0;
    public static final double STATIONARY_SINGLE_TURNING_PID_KF = 0.0;
    public static final double STATIONARY_SINGLE_TURNING_PID_KS = 1.0;
    public static final double STATIONARY_SINGLE_TURNING_PID_MIN = -0.4;
    public static final double STATIONARY_SINGLE_TURNING_PID_MAX = 0.4;

    // PID settings for Centering the robot on a vision target from one stationary place, based on continuous samples
    public static final double STATIONARY_CONTINUOUS_TURNING_PID_KP = 0.02;
    public static final double STATIONARY_CONTINUOUS_TURNING_PID_KI = 0.0;
    public static final double STATIONARY_CONTINUOUS_TURNING_PID_KD = 0.0;
    public static final double STATIONARY_CONTINUOUS_TURNING_PID_KF = 0.0;
    public static final double STATIONARY_CONTINUOUS_TURNING_PID_KS = 1.0;
    public static final double STATIONARY_CONTINUOUS_TURNING_PID_MIN = -0.4;
    public static final double STATIONARY_CONTINUOUS_TURNING_PID_MAX = 0.4;

    // PID settings for rotating the robot based on a vision target while in-motion
    public static final double VISION_MOVING_TURNING_PID_KP = 0.012;
    public static final double VISION_MOVING_TURNING_PID_KI = 0.0;
    public static final double VISION_MOVING_TURNING_PID_KD = 0.0;
    public static final double VISION_MOVING_TURNING_PID_KF = 0.0;
    public static final double VISION_MOVING_TURNING_PID_KS = 1.0;
    public static final double VISION_MOVING_TURNING_PID_MIN = -0.3;
    public static final double VISION_MOVING_TURNING_PID_MAX = 0.3;

    // PID settings for translating the robot based on a vision target
    public static final double VISION_MOVING_PID_KP = 0.015;
    public static final double VISION_MOVING_PID_KI = 0.0;
    public static final double VISION_MOVING_PID_KD = 0.0;
    public static final double VISION_MOVING_PID_KF = 0.0;
    public static final double VISION_MOVING_PID_KS = 1.0;
    public static final double VISION_MOVING_PID_MIN = -0.3;
    public static final double VISION_MOVING_PID_MAX = 0.3;

    // PID settings for translating the robot based on a vision target
    public static final double VISION_AT_TRANSLATION_X_PID_KP = 0.023;
    public static final double VISION_AT_TRANSLATION_X_PID_KI = 0.0;
    public static final double VISION_AT_TRANSLATION_X_PID_KD = 0.0;
    public static final double VISION_AT_TRANSLATION_X_PID_KF = 0.0;
    public static final double VISION_AT_TRANSLATION_X_PID_KS = 1.0;
    public static final double VISION_AT_TRANSLATION_X_PID_MIN = -0.3;
    public static final double VISION_AT_TRANSLATION_X_PID_MAX = 0.3;

    // PID settings for translating the robot based on a vision target
    public static final double VISION_AT_TRANSLATION_Y_PID_KP = 0.023;
    public static final double VISION_AT_TRANSLATION_Y_PID_KI = 0.0;
    public static final double VISION_AT_TRANSLATION_Y_PID_KD = 0.0;
    public static final double VISION_AT_TRANSLATION_Y_PID_KF = 0.0;
    public static final double VISION_AT_TRANSLATION_Y_PID_KS = 1.0;
    public static final double VISION_AT_TRANSLATION_Y_PID_MIN = -0.3;
    public static final double VISION_AT_TRANSLATION_Y_PID_MAX = 0.3;

    // PID settings for translating the robot slowly based on a vision target
    public static final double VISION_SLOW_MOVING_PID_KP = 0.013;
    public static final double VISION_SLOW_MOVING_PID_KI = 0.0;
    public static final double VISION_SLOW_MOVING_PID_KD = 0.0;
    public static final double VISION_SLOW_MOVING_PID_KF = 0.0;
    public static final double VISION_SLOW_MOVING_PID_KS = 1.0;
    public static final double VISION_SLOW_MOVING_PID_MIN = -0.3;
    public static final double VISION_SLOW_MOVING_PID_MAX = 0.3;

    // PID settings for translating the robot quickly based on a vision target
    public static final double VISION_FAST_MOVING_PID_KP = 0.17;
    public static final double VISION_FAST_MOVING_PID_KI = 0.0;
    public static final double VISION_FAST_MOVING_PID_KD = 0.0;
    public static final double VISION_FAST_MOVING_PID_KF = 0.0;
    public static final double VISION_FAST_MOVING_PID_KS = 1.0;
    public static final double VISION_FAST_MOVING_PID_MIN = -0.45;
    public static final double VISION_FAST_MOVING_PID_MAX = 0.45;

    public static final int VISION_MISSED_HEARTBEAT_THRESHOLD = 50;

    public static final double ORIENTATION_TURN_THRESHOLD = 2.0; // number of degrees off at which point we give up trying to face an angle

    public static final double VISION_REEF_TAG_HORIZONTAL_LEFT_OFFSET_VALUE = 7.5; // 6.5;
    public static final double VISION_REEF_TAG_HORIZONTAL_RIGHT_OFFSET_VALUE = -5.5; // -6.5
    public static final double VISION_REEF_TAG_FORWARD_OFFSET_VALUE = 17.5;

    //================================================== Driver Feedback ========================================================

    public static final double ENDGAME_RUMBLE = 20.0;
    public static final double CORAL_INTAKE_RUMBLE_TIME = 0.5;

    //================================================== SDS DriveTrain ==============================================================

    public static final boolean SDSDRIVETRAIN_STEER_MOTORS_USE_MOTION_MAGIC = true;
    public static final boolean SDSDRIVETRAIN_USE_POSE_ESTIMATION_INVERSE_TWIST_CORRECTION = true;
    public static final double SDSDRIVETRAIN_POSE_ESTIMATION_INVERSE_TWIST_CORRECTION_TIMESTEP = 0.02;

    public static final boolean SDSDRIVETRAIN_USE_ODOMETRY = true;
    public static final boolean SDSDRIVETRAIN_RESET_ON_ROBOT_START = true;
    public static final boolean SDSDRIVETRAIN_FIELD_ORIENTED_ON_ROBOT_START = true;
    public static final boolean SDSDRIVETRAIN_MAINTAIN_ORIENTATION_ON_ROBOT_START = false;

    public static final boolean SDSDRIVETRAIN_READSENSORS_ONLY_IN_TEST_MODE = false;
    public static final double SDSDRIVETRAIN_STEER_MOTOR1_ABSOLUTE_OFFSET = TuningConstants.COMPETITION_ROBOT ? 0.37158203125 : 0.2219238; // rotations
    public static final double SDSDRIVETRAIN_STEER_MOTOR2_ABSOLUTE_OFFSET = TuningConstants.COMPETITION_ROBOT ? 0.478759765625 : 0.1450195; // rotations
    public static final double SDSDRIVETRAIN_STEER_MOTOR3_ABSOLUTE_OFFSET = TuningConstants.COMPETITION_ROBOT ? -0.33447265625 : -0.35017; // rotations
    public static final double SDSDRIVETRAIN_STEER_MOTOR4_ABSOLUTE_OFFSET = TuningConstants.COMPETITION_ROBOT ? -0.12109375 : -0.34863; // rotations

    public static final boolean SDSDRIVETRAIN_USE_ROTATIONAL_RATE_LIMITING = true;
    public static final double SDSDRIVETRAIN_ROTATIONAL_VELOCITY_MAX_NEGATIVE_RATE = -4.0 * TuningConstants.SDSDRIVETRAIN_TURN_SCALE;
    public static final double SDSDRIVETRAIN_ROTATIONAL_VELOCITY_MAX_POSITIVE_RATE = 4.0 * TuningConstants.SDSDRIVETRAIN_TURN_SCALE;

    public static final boolean SDSDRIVETRAIN_USE_TRANSLATIONAL_RATE_LIMITING = true;
    public static final double SDSDRIVETRAIN_TRANSLATIONAL_VELOCITY_MAX_NEGATIVE_RATE_DEFAULT = -4.0 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY;
    public static final double SDSDRIVETRAIN_TRANSLATIONAL_VELOCITY_MAX_POSITIVE_RATE_DEFAULT = 4.0 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY;

    public static final boolean SDSDRIVETRAIN_USE_TRANSLATIONAL_RATE_LIMITING_LEVELS = true && TuningConstants.SDSDRIVETRAIN_USE_TRANSLATIONAL_RATE_LIMITING;
    public static final double SDSDRIVETRAIN_TRANSLATIONAL_VELOCITY_MAX_NEGATIVE_RATE_MEDIUM = -2.0 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY;
    public static final double SDSDRIVETRAIN_TRANSLATIONAL_VELOCITY_MAX_POSITIVE_RATE_MEDIUM = 2.0 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY;
    public static final double SDSDRIVETRAIN_TRANSLATIONAL_VELOCITY_MAX_NEGATIVE_RATE_HIGH = -1.0 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY;
    public static final double SDSDRIVETRAIN_TRANSLATIONAL_VELOCITY_MAX_POSITIVE_RATE_HIGH = 1.0 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY;

    public static final double SDSDRIVETRAIN_TRANSLATE_LIMIT_ELEVATOR_THRESHOLD_MEDIUM = HardwareConstants.ELEVATOR_MAX_HEIGHT / 4.0;
    public static final double SDSDRIVETRAIN_TRANSLATE_LIMIT_ELEVATOR_THRESHOLD_HIGH = HardwareConstants.ELEVATOR_MAX_HEIGHT / 2.0;

    // Position PID (angle) per-module
    public static final double SDSDRIVETRAIN_STEER_MOTOR_POSITION_PID_KS = HardwareConstants.SDSDRIVETRAIN_STEER_TICKS_PER_DEGREE;

    public static final double SDSDRIVETRAIN_STEER_MOTORS_POSITION_PID_KP = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_POSITION_PID_KP : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_POSITION_PID_KP;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_POSITION_PID_KI = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_POSITION_PID_KI : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_POSITION_PID_KI;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_POSITION_PID_KD = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_POSITION_PID_KD : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_POSITION_PID_KD;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_POSITION_PID_KF = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_POSITION_PID_KF : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_POSITION_PID_KF;

    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_KP = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_KP : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_KP;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_KI = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_KI : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_KI;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_KD = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_KD : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_KD;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_KV = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_KV : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_KV;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_KS = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_KS : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_KS;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_CRUISE_VELOC = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_CRUISE_VELOC : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_CRUISE_VELOC;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_ACCEL = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_ACCEL : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_ACCEL;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_MM_PID_JERK = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_JERK : TuningConstants.SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_JERK;

    // STEER PRACTICE
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_POSITION_PID_KP = 1.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_POSITION_PID_KI = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_POSITION_PID_KD = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_POSITION_PID_KF = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);

    // STEER RPM ~107.0 was highest speed at full throttle FF on blocks
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_KP = 0.3333333 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_KI = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_KD = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_KV = 0.00934579 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_KS = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_CRUISE_VELOC = 100.0;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_ACCEL = 600.0;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_PRACTICE_MM_PID_JERK = 9999.0;

    // STEER COMP
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_POSITION_PID_KP = 1.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_POSITION_PID_KI = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_POSITION_PID_KD = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_POSITION_PID_KF = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);

    // STEER RPM ~107.0 was highest speed at full throttle FF on blocks
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_KP = 0.3333333 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_KI = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_KD = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_KV = 0.00934579 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_KS = 0.0 * (TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_CRUISE_VELOC = 100.0;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_ACCEL = 600.0;
    public static final double SDSDRIVETRAIN_STEER_MOTORS_COMP_MM_PID_JERK = 9999.0;

    // Velocity PID (drive) per-module
    public static final double SDSDRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_DRIVE_MOTOR_COMP_VELOCITY_PID_KS : TuningConstants.SDSDRIVETRAIN_DRIVE_MOTOR_PRACTICE_VELOCITY_PID_KS; // RPM ~110.0 was highest speed at full throttle FF on blocks

    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KP = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_COMP_VELOCITY_PID_KP : TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_VELOCITY_PID_KP;
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KI = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_COMP_VELOCITY_PID_KI : TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_VELOCITY_PID_KI;
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KD = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_COMP_VELOCITY_PID_KD : TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_VELOCITY_PID_KD;
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_VELOCITY_PID_KF = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_COMP_VELOCITY_PID_KF : TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_VELOCITY_PID_KF;

    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KP = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_COMP_POSITION_PID_KP : TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_POSITION_PID_KP;
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KI = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_COMP_POSITION_PID_KI : TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_POSITION_PID_KI;
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KD = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_COMP_POSITION_PID_KD : TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_POSITION_PID_KD;
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_POSITION_PID_KF = TuningConstants.COMPETITION_ROBOT ? TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_COMP_POSITION_PID_KF : TuningConstants.SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_POSITION_PID_KF;

    // DRIVE PRACTICE
    public static final double SDSDRIVETRAIN_DRIVE_MOTOR_PRACTICE_VELOCITY_PID_KS = 88.0; // RPM ~110.0 was highest speed at full throttle FF on blocks

    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_VELOCITY_PID_KP = 0.02 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_VELOCITY_PID_KI = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_VELOCITY_PID_KD = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_VELOCITY_PID_KF = 0.00909 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0); // 100% control authority (on blocks)

    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_POSITION_PID_KP = 2.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_POSITION_PID_KI = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_POSITION_PID_KD = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_PRACTICE_POSITION_PID_KF = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);

    // DRIVE COMP
    public static final double SDSDRIVETRAIN_DRIVE_MOTOR_COMP_VELOCITY_PID_KS = 88.0; // RPM ~104.0 was highest speed at full throttle FF on blocks

    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_COMP_VELOCITY_PID_KP = 0.02 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_COMP_VELOCITY_PID_KI = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_COMP_VELOCITY_PID_KD = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_COMP_VELOCITY_PID_KF = 0.00961538 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0); // 100% control authority (on blocks)

    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_COMP_POSITION_PID_KP = 2.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_COMP_POSITION_PID_KI = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_COMP_POSITION_PID_KD = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);
    public static final double SDSDRIVETRAIN_DRIVE_MOTORS_COMP_POSITION_PID_KF = 0.0 * (TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED ? TuningConstants.SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION : 1.0);

    public static final double SDSDRIVETRAIN_OMEGA_POSITION_PID_KP = 0.1;
    public static final double SDSDRIVETRAIN_OMEGA_POSITION_PID_KI = 0.0;
    public static final double SDSDRIVETRAIN_OMEGA_POSITION_PID_KD = 0.0;
    public static final double SDSDRIVETRAIN_OMEGA_POSITION_PID_KF = 0.0;
    public static final double SDSDRIVETRAIN_OMEGA_POSITION_PID_KS = 1.0;
    public static final double SDSDRIVETRAIN_OMEGA_MAX_OUTPUT = 5.0;
    public static final double SDSDRIVETRAIN_OMEGA_MIN_OUTPUT = -5.0;

    public static final double SDSDRIVETRAIN_PATH_OMEGA_POSITION_PID_KP = 0.1;
    public static final double SDSDRIVETRAIN_PATH_OMEGA_POSITION_PID_KI = 0.0;
    public static final double SDSDRIVETRAIN_PATH_OMEGA_POSITION_PID_KD = 0.0;
    public static final double SDSDRIVETRAIN_PATH_OMEGA_POSITION_PID_KF = 0.0;
    public static final double SDSDRIVETRAIN_PATH_OMEGA_POSITION_PID_KS = 1.0;
    public static final double SDSDRIVETRAIN_PATH_OMEGA_MAX_OUTPUT = 4.0;
    public static final double SDSDRIVETRAIN_PATH_OMEGA_MIN_OUTPUT = -4.0;

    public static final double SDSDRIVETRAIN_PATH_X_POSITION_PID_KP = 1.0;
    public static final double SDSDRIVETRAIN_PATH_X_POSITION_PID_KI = 0.0;
    public static final double SDSDRIVETRAIN_PATH_X_POSITION_PID_KD = 0.0;
    public static final double SDSDRIVETRAIN_PATH_X_POSITION_PID_KF = 0.0;
    public static final double SDSDRIVETRAIN_PATH_X_POSITION_PID_KS = 1.0;
    public static final double SDSDRIVETRAIN_PATH_X_MAX_OUTPUT = 10.0;
    public static final double SDSDRIVETRAIN_PATH_X_MIN_OUTPUT = -10.0;

    public static final double SDSDRIVETRAIN_PATH_Y_POSITION_PID_KP = 1.0;
    public static final double SDSDRIVETRAIN_PATH_Y_POSITION_PID_KI = 0.0;
    public static final double SDSDRIVETRAIN_PATH_Y_POSITION_PID_KD = 0.0;
    public static final double SDSDRIVETRAIN_PATH_Y_POSITION_PID_KF = 0.0;
    public static final double SDSDRIVETRAIN_PATH_Y_POSITION_PID_KS = 1.0;
    public static final double SDSDRIVETRAIN_PATH_Y_MAX_OUTPUT = 10.0;
    public static final double SDSDRIVETRAIN_PATH_Y_MIN_OUTPUT = -10.0;

    public static final boolean SDSDRIVETRAIN_USE_OVERCURRENT_ADJUSTMENT = true;
    public static final double SDSDRIVETRAIN_OVERCURRENT_ADJUSTMENT = 0.75;
    public static final double SDSDRIVETRAIN_OVERCURRENT_HIGH_ADJUSTMENT = 0.5;

    public static final boolean SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION_ENABLED = true;
    public static final double SDSDRIVETRAIN_DRIVE_VOLTAGE_COMPENSATION = 12.0;
    public static final boolean SDSDRIVETRAIN_DRIVE_SUPPLY_CURRENT_LIMITING_ENABLED = true;
    public static final double SDSDRIVETRAIN_DRIVE_SUPPLY_CURRENT_MAX = 35.0;
    public static final double SDSDRIVETRAIN_DRIVE_SUPPLY_TRIGGER_CURRENT = 35.0;
    public static final double SDSDRIVETRAIN_DRIVE_SUPPLY_TRIGGER_DURATION = 0.25;
    public static final boolean SDSDRIVETRAIN_DRIVE_STATOR_CURRENT_LIMITING_ENABLED = false;
    public static final double SDSDRIVETRAIN_DRIVE_STATOR_CURRENT_LIMIT = 80.0;

    public static final boolean SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION_ENABLED = true;
    public static final double SDSDRIVETRAIN_STEER_VOLTAGE_COMPENSATION = 12.0;
    public static final boolean SDSDRIVETRAIN_STEER_SUPPLY_CURRENT_LIMITING_ENABLED = true;
    public static final double SDSDRIVETRAIN_STEER_SUPPLY_CURRENT_MAX = 20.0;
    public static final double SDSDRIVETRAIN_STEER_SUPPLY_TRIGGER_CURRENT = 30.0;
    public static final double SDSDRIVETRAIN_STEER_SUPPLY_TRIGGER_DURATION = 0.1;
    public static final boolean SDSDRIVETRAIN_STEER_STATOR_CURRENT_LIMITING_ENABLED = false;
    public static final double SDSDRIVETRAIN_STEER_STATOR_CURRENT_LIMIT = 80.0;

    public static final int SDSDRIVETRAIN_FEEDBACK_UPDATE_RATE_HZ = 100;
    public static final int SDSDRIVETRAIN_ERROR_UPDATE_RATE_HZ = 10;

    public static final boolean SDSDRIVETRAIN_SKIP_ANGLE_ON_ZERO_VELOCITY = true;
    public static final double SDSDRIVETRAIN_SKIP_ANGLE_ON_ZERO_DELTA = 0.001;
    public static final double SDSDRIVETRAIN_SKIP_OMEGA_ON_ZERO_DELTA = 0.36;

    public static final double SDSDRIVETRAIN_EXPONENTIAL = 2.0;
    public static final double SDSDRIVETRAIN_DEAD_ZONE_TURN = 0.1;
    public static final double SDSDRIVETRAIN_DEAD_ZONE_VELOCITY_X = 0.1;
    public static final double SDSDRIVETRAIN_DEAD_ZONE_VELOCITY_Y = 0.1;
    public static final double SDSDRIVETRAIN_DEAD_ZONE_TRIGGER_AB = 0.1;

    public static final double SDSDRIVETRAIN_ROTATION_A_MULTIPLIER = HardwareConstants.SDSDRIVETRAIN_HORIZONTAL_WHEEL_SEPERATION_DISTANCE / 2.0;
    public static final double SDSDRIVETRAIN_ROTATION_B_MULTIPLIER = HardwareConstants.SDSDRIVETRAIN_VERTICAL_WHEEL_SEPERATION_DISTANCE / 2.0;

    public static final double SDSDRIVETRAIN_MAX_MODULE_VELOCITY = TuningConstants.SDSDRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS * HardwareConstants.SDSDRIVETRAIN_DRIVE_MOTOR_VELOCITY_TO_INCHES_PER_SECOND; // max velocity in inches per second
    public static final double SDSDRIVETRAIN_MAX_VELOCITY = TuningConstants.SDSDRIVETRAIN_MAX_MODULE_VELOCITY; // max velocity in inches per second
    public static final double SDSDRIVETRAIN_SLOW_MODE_MAX_VELOCITY = 0.3 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY; // max velocity in inches per second
    public static final double SDSDRIVETRAIN_VELOCITY_TO_PERCENTAGE = 1.0 / TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY;
    public static final double SDSDRIVETRAIN_TURN_GOAL_VELOCITY = 10.0; // degrees per second for turn goal
    public static final double SDSDRIVETRAIN_TURN_SCALE = 1.6 * Math.PI; // radians per second
    public static final double SDSDRIVETRAIN_SLOW_MODE_TURN_SCALE = 0.3 * TuningConstants.SDSDRIVETRAIN_TURN_SCALE; // radians per second
    public static final double SDSDRIVETRAIN_STATIONARY_VELOCITY = 0.1;
    public static final double SDSDRIVETRAIN_TURN_APPROXIMATION_STATIONARY = 2.0; // number of degrees off at which point we give up trying to face an angle when uncommanded
    public static final double SDSDRIVETRAIN_TURN_APPROXIMATION = 1.0; // number of degrees off at which point we give up trying to face an angle when uncommanded
    public static final double SDSDRIVETRAIN_MAX_MODULE_PATH_VELOCITY = 0.85 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY; // up to x% of our max controllable speed
    public static final double SDSDRIVETRAIN_MAX_PATH_TURN_VELOCITY = 180.0; // in degrees per second
    public static final double SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY = 0.60 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY; // in inches per second
    public static final double SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION = 0.60 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY; // in inches per second per second
    public static final double SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_VELOCITY = 0.80 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY; // in inches per second
    public static final double SDSDRIVETRAIN_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION = 0.80 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY; // in inches per second per second
    public static final double SDSDRIVETRAIN_SUPER_MAX_PATH_TRANSLATIONAL_VELOCITY = 0.90 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY; // in inches per second
    public static final double SDSDRIVETRAIN_SUPER_MAX_PATH_TRANSLATIONAL_ACCELERATION = 1.6 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY; // in inches per second per second
    public static final double SDSDRIVETRAIN_SUPER_TRUE_MAX_PATH_TRANSLATIONAL_ACCELERATION = 0.90 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY; // in inches per second per second
    public static final double SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_VELOCITY = 0.60 * TuningConstants.SDSDRIVETRAIN_MAX_PATH_TURN_VELOCITY; // in degrees per second
    public static final double SDSDRIVETRAIN_MAX_PATH_ROTATIONAL_ACCELERATION = 0.75 * TuningConstants.SDSDRIVETRAIN_MAX_PATH_TURN_VELOCITY; // in degrees per second per second
    public static final double SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_VELOCITY = 0.50 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY; // in inches per second
    public static final double SDSDRIVETRAIN_MID_PATH_TRANSLATIONAL_ACCELERATION = 0.50 * TuningConstants.SDSDRIVETRAIN_MAX_VELOCITY; // in inches per second per second
    public static final double SDSDRIVETRAIN_LOW_PATH_TRANSLATIONAL_VELOCITY = TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_VELOCITY / 2.0; // in inches per second
    public static final double SDSDRIVETRAIN_LOW_PATH_TRANSLATIONAL_ACCELERATION = TuningConstants.SDSDRIVETRAIN_MAX_PATH_TRANSLATIONAL_ACCELERATION / 2.0; // in inches per second per second 

    //================================================== Coral EndEffector ==============================================================

    public static final boolean CORAL_INTAKE_MOTOR_INVERT_OUTPUT = true;
    public static final boolean CORAL_WRIST_MOTOR_INVERT_OUTPUT = false;
    public static final boolean CORAL_WRIST_MOTOR_INVERT_SENSOR = true;

    public static final boolean CORAL_WRIST_USE_PID = true;
    public static final boolean CORAL_WRIST_USE_TMP = false;

    public static final boolean CORAL_WRIST_POSITION_PID_WRAPPING_ENABLED = true;
    public static final double CORAL_WRIST_POSITION_PID_WRAPPING_MIN = 0.0;
    public static final double CORAL_WRIST_POSITION_PID_WRAPPING_MAX = 360.0;

    public static final int CORAL_WITHOUT_PID_SLOT = 0;
    public static final int CORAL_WITH_PID_SLOT = 1;

    // Coral PID settings (no TMP, without game piece)
    public static final double CORAL_WITHOUT_PID_WRIST_KP = 0.125;
    public static final double CORAL_WITHOUT_PID_WRIST_KI = 0.0;
    public static final double CORAL_WITHOUT_PID_WRIST_KD = 0.025;
    public static final double CORAL_WITHOUT_PID_WRIST_KF = 0.0;

    // Coral PID settings (no TMP, with game piece)
    public static final double CORAL_WITH_PID_WRIST_KP = TuningConstants.CORAL_WITHOUT_PID_WRIST_KP;
    public static final double CORAL_WITH_PID_WRIST_KI = TuningConstants.CORAL_WITHOUT_PID_WRIST_KI;
    public static final double CORAL_WITH_PID_WRIST_KD = TuningConstants.CORAL_WITHOUT_PID_WRIST_KD;
    public static final double CORAL_WITH_PID_WRIST_KF = TuningConstants.CORAL_WITHOUT_PID_WRIST_KF;

    // Coral PID settings (with TMP, without game piece)
    public static final double CORAL_WITHOUT_TMP_WRIST_KP = 0.0;
    public static final double CORAL_WITHOUT_TMP_WRIST_KI = 0.0;
    public static final double CORAL_WITHOUT_TMP_WRIST_KD = 0.0;
    public static final double CORAL_WITHOUT_TMP_WRIST_KF = 0.0;
    public static final double CORAL_WITHOUT_TMP_WRIST_CRUISE_VELOCITY = 100.0;
    public static final double CORAL_WITHOUT_TMP_WRIST_MAX_ACCELERATION = 75.0;

    // Coral PID settings (with TMP, with game piece)
    public static final double CORAL_WITH_TMP_WRIST_KP = 0.0;
    public static final double CORAL_WITH_TMP_WRIST_KI = 0.0;
    public static final double CORAL_WITH_TMP_WRIST_KD = 0.0;
    public static final double CORAL_WITH_TMP_WRIST_KF = 0.0;
    public static final double CORAL_WITH_TMP_WRIST_CRUISE_VELOCITY = 100.0;
    public static final double CORAL_WITH_TMP_WRIST_MAX_ACCELERATION = 75.0;

    public static final double CORAL_WRIST_MIN_OUTPUT = -1.0;
    public static final double CORAL_WRIST_MAX_OUTPUT = 1.0;

    public static final double CORAL_INTAKE_POWER = 0.39; // percent output
    public static final double CORAL_INTAKE_SLOW_POWER = 0.35; // percent output
    public static final double CORAL_REVERSE_POWER = -0.4; // percent output
    public static final double CORAL_REVERSE_SLOW_POWER = -0.25; // percent output
    public static final double CORAL_PLACE_POWER = 0.8; // percent output
    public static final double CORAL_PLACE_SLOW_POWER = 0.5; // percent output

    // Coral Power calculations
    public static final double CORAL_INTAKE_MIN_POWER_VALUE = 0.0; // in watts
    public static final double CORAL_INTAKE_MAX_POWER_VALUE = 40.0 * 12.0; // in watts
    public static final double CORAL_WRIST_MIN_POWER_VALUE = 0.0; // in watts 
    public static final double CORAL_WRIST_MAX_POWER_VALUE = 40.0 * 12.0; // in watts

    // Coral stall
    public static final boolean CORAL_USE_WRIST_STALL_MODE = false;
    public static final double CORAL_POWER_TRACKING_DURATION = 0.5;
    public static final double CORAL_POWER_SAMPLES_PER_LOOP = 1.0; // we may want to increase this if we find our update loop duration isn't very consistent...
    public static final double CORAL_POWER_SAMPLES_PER_SECOND = TuningConstants.LOOPS_PER_SECOND * TuningConstants.POWER_OVERCURRENT_SAMPLES_PER_LOOP;
    public static final double CORAL_VELOCITY_TRACKING_DURATION = TuningConstants.CORAL_POWER_TRACKING_DURATION;
    public static final double CORAL_VELOCITY_SAMPLES_PER_SECOND = TuningConstants.CORAL_POWER_SAMPLES_PER_SECOND;

    public static final double CORAL_WRIST_STALLED_POWER_THRESHOLD = 12.0 * 10.0; // in watts
    public static final double CORAL_WRIST_STALLED_VELOCITY_THRESHOLD = 1.0; // in degrees per second

    // Wrist min and max angle
    public static final double CORAL_WRIST_ANGLE_RANGE_START = 241.0;
    public static final double CORAL_WRIST_ANGLE_RANGE_END = 32.0;
    public static final double CORAL_WRIST_REGULAR_OUT_ANGLE = 0.0;
    public static final double CORAL_WRIST_L1_OUT_ANGLE = 330.0;
    public static final double CORAL_WRIST_STOW_ANGLE = 270;
    public static final double CORAL_WRIST_ABSOLUTE_ENCODER_OFFSET = 0.633105; // the offset in hardware between the encoder's 0 (based on magnetic north) and perfectly forward

    public static final double CORAL_WRIST_ANGLE_ADJUSTMENT_VELOCITY = 60.0; // in degrees per second

    public static final double CORAL_WRIST_POSITION_TOLERANCE = 1.0;

    public static final double CORAL_WRIST_DEAD_ZONE = 0.1;

    //================================================== Algae EndEffector ==============================================================

    public static final boolean ALGAE_INTAKE_MOTOR_INVERT_OUTPUT = true;
    public static final boolean ALGAE_WRIST_MOTOR_INVERT_OUTPUT = false;
    public static final boolean ALGAE_WRIST_MOTOR_INVERT_SENSOR = false;

    public static final boolean ALGAE_WRIST_USE_PID = true;
    public static final boolean ALGAE_WRIST_USE_TMP = false;

    public static final int ALGAE_WITHOUT_PID_SLOT = 0;
    public static final int ALGAE_WITH_PID_SLOT = 1;

    // ALGAE PID settings (no TMP, without game piece)
    public static final double ALGAE_WITHOUT_PID_WRIST_KP = 0.03;
    public static final double ALGAE_WITHOUT_PID_WRIST_KI = 0.0;
    public static final double ALGAE_WITHOUT_PID_WRIST_KD = 0.0;
    public static final double ALGAE_WITHOUT_PID_WRIST_KF = 0.0;

    // ALGAE PID settings (no TMP, with game piece)
    public static final double ALGAE_WITH_PID_WRIST_KP = 0.03;
    public static final double ALGAE_WITH_PID_WRIST_KI = 0.0;
    public static final double ALGAE_WITH_PID_WRIST_KD = 0.0;
    public static final double ALGAE_WITH_PID_WRIST_KF = 0.0;

    // ALGAE PID settings (with TMP, without game piece)
    public static final double ALGAE_WITHOUT_TMP_WRIST_KP = 0.0;
    public static final double ALGAE_WITHOUT_TMP_WRIST_KI = 0.0;
    public static final double ALGAE_WITHOUT_TMP_WRIST_KD = 0.0;
    public static final double ALGAE_WITHOUT_TMP_WRIST_KF = 0.0;
    public static final double ALGAE_WITHOUT_TMP_WRIST_CRUISE_VELOCITY = 100.0;
    public static final double ALGAE_WITHOUT_TMP_WRIST_MAX_ACCELERATION = 75.0;

    // ALGAE PID settings (with TMP, with game piece)
    public static final double ALGAE_WITH_TMP_WRIST_KP = 0.0;
    public static final double ALGAE_WITH_TMP_WRIST_KI = 0.0;
    public static final double ALGAE_WITH_TMP_WRIST_KD = 0.0;
    public static final double ALGAE_WITH_TMP_WRIST_KF = 0.0;
    public static final double ALGAE_WITH_TMP_WRIST_CRUISE_VELOCITY = 100.0;
    public static final double ALGAE_WITH_TMP_WRIST_MAX_ACCELERATION = 75.0;

    public static final double ALGAE_WRIST_MIN_OUTPUT = -0.5;
    public static final double ALGAE_WRIST_MAX_OUTPUT = 0.5;

    public static final double ALGAE_INTAKE_POWER = 0.5; // percent output
    public static final double ALGAE_HOLD_POWER = 0.25; // percent output
    public static final double ALGAE_OUTTAKE_POWER = -0.5; //percent output
    public static final double ALGAE_LAUNCH_POWER = -0.8;
    public static final boolean USE_ALGAE_HOLD_POWER = true;

    // ALGAE Power calculations
    public static final double ALGAE_INTAKE_MIN_POWER_VALUE = 0.0; // in watts
    public static final double ALGAE_INTAKE_MAX_POWER_VALUE = 40.0 * 12.0; // in watts
    public static final double ALGAE_WRIST_MIN_POWER_VALUE = 0.0; // in watts 
    public static final double ALGAE_WRIST_MAX_POWER_VALUE = 40.0 * 12.0; // in watts

    // ALGAE stall
    public static final boolean ALGAE_USE_WRIST_STALL_MODE = false;
    public static final double ALGAE_WRIST_POWER_TRACKING_DURATION = 0.5;
    public static final double ALGAE_INTAKE_POWER_TRACKING_DURATION = 0.25;
    public static final double ALGAE_POWER_SAMPLES_PER_LOOP = 1.0; // we may want to increase this if we find our update loop duration isn't very consistent...
    public static final double ALGAE_POWER_SAMPLES_PER_SECOND = TuningConstants.LOOPS_PER_SECOND * TuningConstants.POWER_OVERCURRENT_SAMPLES_PER_LOOP;
    public static final double ALGAE_VELOCITY_TRACKING_DURATION = TuningConstants.ALGAE_WRIST_POWER_TRACKING_DURATION;
    public static final double ALGAE_VELOCITY_SAMPLES_PER_SECOND = TuningConstants.ALGAE_POWER_SAMPLES_PER_SECOND;

    public static final double ALGAE_WRIST_STALLED_POWER_THRESHOLD = 12.0 * 10.0; // in watts
    public static final double ALGAE_WRIST_STALLED_VELOCITY_THRESHOLD = 1.0; // in degrees per second

    public static final double ALGAE_INTAKE_STALLED_POWER_THRESHOLD = 150.0; // in watts
    public static final double ALGAE_INTAKE_NO_LONGER_STALLED_POWER_THRESHOLD = 35.0; // in watts

    // Wrist min and max angle
    public static final double ALGAE_WRIST_ANGLE_RANGE_START = 145.0;
    public static final double ALGAE_WRIST_ANGLE_RANGE_END = 348.0;
    public static final double ALGAE_WRIST_ABSOLUTE_ENCODER_OFFSET = 0.897566666; // the offset in hardware between the encoder's 0 (based on magnetic north) and perfectly forward

    public static final double ALGAE_WRIST_PROCESSOR_ANGLE = 240.0;
    public static final double ALGAE_WRIST_BARGE_ANGLE = 315.0;
    public static final double ALGAE_WRIST_REEF_PICKUP_L23_ANGLE = 262.5;
    public static final double ALGAE_WRIST_REEF_PICKUP_L34_ANGLE = 282.5;
    public static final double ALGAE_WRIST_HOLD_ANGLE = 320.0;
    public static final double ALGAE_WRIST_STOW_ANGLE = 150.0;

    public static final double ALGAE_WRIST_ANGLE_ADJUSTMENT_VELOCITY = 45.0; // in degrees per second

    public static final double ALGAE_WRIST_POSITION_TOLERANCE = 4.0;

    public static final boolean SET_ALGAE_TO_IN_GAME_STOW = false;

    public static final double ALGAE_WRIST_NET_ANGLE = 0.0; // CHANGE

    public static final boolean ALGAE_WRIST_POSITION_PID_WRAPPING_ENABLED = true;
    public static final double ALGAE_WRIST_POSITION_PID_WRAPPING_MIN = 0.0; // TuningConstants.ALGAE_WRIST_ANGLE_RANGE_START;
    public static final double ALGAE_WRIST_POSITION_PID_WRAPPING_MAX = 360.0; // TuningConstants.ALGAE_WRIST_ANGLE_RANGE_END;

    //================================================== Elevator ==============================================================

    public static final boolean ELEVATOR_IN_PID_MODE = true;
    public static final boolean ELEVATOR_USE_MOTION_MAGIC = true;

    public static final double ELEVATOR_SETPOINT_MULTIPLIER = 0.6993006993006993;
    public static final double ELEVATOR_RESET_HEIGHT = 0.0;
    public static final double ELEVATOR_HOME_HEIGHT = 0.0;
    public static final double ELEVATOR_PROCESSOR_HEIGHT = TuningConstants.ELEVATOR_SETPOINT_MULTIPLIER * 0.0;
    public static final double ELEVATOR_CORAL_L1_HEIGHT = TuningConstants.ELEVATOR_SETPOINT_MULTIPLIER * 2.25;
    public static final double ELEVATOR_CORAL_L2_HEIGHT = TuningConstants.ELEVATOR_SETPOINT_MULTIPLIER * 25.2;
    public static final double ELEVATOR_CORAL_L3_HEIGHT = TuningConstants.ELEVATOR_SETPOINT_MULTIPLIER * 57.5;
    public static final double ELEVATOR_CORAL_L4_HEIGHT = TuningConstants.ELEVATOR_SETPOINT_MULTIPLIER * 105.5;
    public static final double ELEVATOR_ALGAE_L23_HEIGHT = TuningConstants.ELEVATOR_SETPOINT_MULTIPLIER * 0.0;
    public static final double ELEVATOR_ALGAE_L34_HEIGHT = TuningConstants.ELEVATOR_SETPOINT_MULTIPLIER * 18.75;
    public static final double ELEVATOR_ALGAE_BARGE_HEIGHT = TuningConstants.ELEVATOR_SETPOINT_MULTIPLIER * 105.5;

    // heights at which the coral end-effector (owl head) "clears" the pre-staged algae so that it can turn back to forward
    public static final double ELEVATOR_ALGAE_L23_CLEARING_HEIGHT = 40.0; 
    public static final double ELEVATOR_ALGAE_L34_CLEARING_HEIGHT = 65.0;

    // Gain Scheduling when elevator has game piece 
    public static final int ELEVATOR_GP_PID_SLOT = 0;

    // With Game Piece (GP) Motion Magic PID
    public static final double ELEVATOR_MOTION_MAGIC_GP_MM_PID_KP = 0.25 * (TuningConstants.ELEVATOR_USE_VOLTAGE_COMPENSATION ? TuningConstants.ELEVATOR_VOLTAGE_COMPENSATION : 1.0);
    public static final double ELEVATOR_MOTION_MAGIC_GP_MM_PID_KI = 0.0 * (TuningConstants.ELEVATOR_USE_VOLTAGE_COMPENSATION ? TuningConstants.ELEVATOR_VOLTAGE_COMPENSATION : 1.0);
    public static final double ELEVATOR_MOTION_MAGIC_GP_MM_PID_KD = 0.0 * (TuningConstants.ELEVATOR_USE_VOLTAGE_COMPENSATION ? TuningConstants.ELEVATOR_VOLTAGE_COMPENSATION : 1.0);
    public static final double ELEVATOR_MOTION_MAGIC_GP_MM_PID_KV = 0.0 * (TuningConstants.ELEVATOR_USE_VOLTAGE_COMPENSATION ? TuningConstants.ELEVATOR_VOLTAGE_COMPENSATION : 1.0);
    public static final double ELEVATOR_MOTION_MAGIC_GP_MM_PID_KS = 0.0 * (TuningConstants.ELEVATOR_USE_VOLTAGE_COMPENSATION ? TuningConstants.ELEVATOR_VOLTAGE_COMPENSATION : 1.0);
    public static final double ELEVATOR_MOTION_MAGIC_GP_MM_PID_CRUISE_VELOCITY = 120.0 * HardwareConstants.ELEVATOR_TICKS_PER_INCH;
    public static final double ELEVATOR_MOTION_MAGIC_GP_MM_PID_MAX_ACCELERATION = 400.0 * HardwareConstants.ELEVATOR_TICKS_PER_INCH;
    public static final double ELEVATOR_MOTION_MAGIC_GP_MM_PID_MAX_JERK = 3000.0 * HardwareConstants.ELEVATOR_TICKS_PER_INCH;

    // public static final double ELEVATOR_MOTION_MAGIC_GP_MM_PID_KP = 0.04 * (TuningConstants.ELEVATOR_USE_VOLTAGE_COMPENSATION ? TuningConstants.ELEVATOR_VOLTAGE_COMPENSATION : 1.0);
    // public static final double ELEVATOR_MOTION_MAGIC_GP_MM_PID_KI = 0.0 * (TuningConstants.ELEVATOR_USE_VOLTAGE_COMPENSATION ? TuningConstants.ELEVATOR_VOLTAGE_COMPENSATION : 1.0);
    // public static final double ELEVATOR_MOTION_MAGIC_GP_MM_PID_KD = 0.0 * (TuningConstants.ELEVATOR_USE_VOLTAGE_COMPENSATION ? TuningConstants.ELEVATOR_VOLTAGE_COMPENSATION : 1.0);
    // public static final double ELEVATOR_MOTION_MAGIC_GP_MM_PID_KV = 0.0 * (TuningConstants.ELEVATOR_USE_VOLTAGE_COMPENSATION ? TuningConstants.ELEVATOR_VOLTAGE_COMPENSATION : 1.0);
    // public static final double ELEVATOR_MOTION_MAGIC_GP_MM_PID_KS = 0.0 * (TuningConstants.ELEVATOR_USE_VOLTAGE_COMPENSATION ? TuningConstants.ELEVATOR_VOLTAGE_COMPENSATION : 1.0);
    // public static final double ELEVATOR_MOTION_MAGIC_GP_MM_PID_CRUISE_VELOCITY = 150.0 * HardwareConstants.ELEVATOR_TICKS_PER_INCH;
    // public static final double ELEVATOR_MOTION_MAGIC_GP_MM_PID_MAX_ACCELERATION = 450.0 * HardwareConstants.ELEVATOR_TICKS_PER_INCH;
    // public static final double ELEVATOR_MOTION_MAGIC_GP_MM_PID_MAX_JERK = 0.0;

    // Without Game Piece (NGP) Motion Magic PID
    public static final double ELEVATOR_MOTION_MAGIC_NGP_MM_PID_KP = TuningConstants.ELEVATOR_MOTION_MAGIC_GP_MM_PID_KP;
    public static final double ELEVATOR_MOTION_MAGIC_NGP_MM_PID_KI = TuningConstants.ELEVATOR_MOTION_MAGIC_GP_MM_PID_KI;
    public static final double ELEVATOR_MOTION_MAGIC_NGP_MM_PID_KD = TuningConstants.ELEVATOR_MOTION_MAGIC_GP_MM_PID_KD;
    public static final double ELEVATOR_MOTION_MAGIC_NGP_MM_PID_KV = TuningConstants.ELEVATOR_MOTION_MAGIC_GP_MM_PID_KV;
    public static final double ELEVATOR_MOTION_MAGIC_NGP_MM_PID_KS = TuningConstants.ELEVATOR_MOTION_MAGIC_GP_MM_PID_KS;
    public static final double ELEVATOR_MOTION_MAGIC_NGP_MM_PID_CRUISE_VELOCITY = TuningConstants.ELEVATOR_MOTION_MAGIC_GP_MM_PID_CRUISE_VELOCITY;
    public static final double ELEVATOR_MOTION_MAGIC_NGP_MM_PID_MAX_ACCELERATION = TuningConstants.ELEVATOR_MOTION_MAGIC_GP_MM_PID_MAX_ACCELERATION;
    public static final double ELEVATOR_MOTION_MAGIC_NGP_MM_PID_MAX_JERK = TuningConstants.ELEVATOR_MOTION_MAGIC_GP_MM_PID_MAX_JERK;
    
    // Gain Scheduling when elevator does not have game piece 
    public static final int ELEVATOR_NGP_PID_SLOT = 1;

    //With Game Piece (GP) Positional PID
    public static final double ELEVATOR_POSITION_GP_PID_KP = 0.04 * (TuningConstants.ELEVATOR_USE_VOLTAGE_COMPENSATION ? TuningConstants.ELEVATOR_VOLTAGE_COMPENSATION : 1.0);
    public static final double ELEVATOR_POSITION_GP_PID_KI = 0.0 * (TuningConstants.ELEVATOR_USE_VOLTAGE_COMPENSATION ? TuningConstants.ELEVATOR_VOLTAGE_COMPENSATION : 1.0);
    public static final double ELEVATOR_POSITION_GP_PID_KD = 0.01 * (TuningConstants.ELEVATOR_USE_VOLTAGE_COMPENSATION ? TuningConstants.ELEVATOR_VOLTAGE_COMPENSATION : 1.0);
    public static final double ELEVATOR_POSITION_GP_PID_KF = 0.0 * (TuningConstants.ELEVATOR_USE_VOLTAGE_COMPENSATION ? TuningConstants.ELEVATOR_VOLTAGE_COMPENSATION : 1.0);

    //Without Game Piece (GP) Positional PID
    public static final double ELEVATOR_POSITION_NGP_PID_KP = 0.04 * (TuningConstants.ELEVATOR_USE_VOLTAGE_COMPENSATION ? TuningConstants.ELEVATOR_VOLTAGE_COMPENSATION : 1.0);
    public static final double ELEVATOR_POSITION_NGP_PID_KI = 0.0 * (TuningConstants.ELEVATOR_USE_VOLTAGE_COMPENSATION ? TuningConstants.ELEVATOR_VOLTAGE_COMPENSATION : 1.0);
    public static final double ELEVATOR_POSITION_NGP_PID_KD = 0.01 * (TuningConstants.ELEVATOR_USE_VOLTAGE_COMPENSATION ? TuningConstants.ELEVATOR_VOLTAGE_COMPENSATION : 1.0);
    public static final double ELEVATOR_POSITION_NGP_PID_KF = 0.0 * (TuningConstants.ELEVATOR_USE_VOLTAGE_COMPENSATION ? TuningConstants.ELEVATOR_VOLTAGE_COMPENSATION : 1.0);

    public static final double ELEVATOR_SLOW_POWER = 0.05;

    // multiplier for PercentOutput mode
    public static final double ELEVATOR_UP_SPEED = 1.0;
    public static final double ELEVATOR_DOWN_SPEED = -1.0;

    public static final double ELEVATOR_ADJUSTMENT_MULTIPLIER = 10.0; // inches per second

    public static final boolean ELEVATOR_USE_GRAVITY_COMPENSATION = false;
    public static final double ELEVATOR_GRAVITY_COMPENSATION = 0.0 * (TuningConstants.ELEVATOR_USE_VOLTAGE_COMPENSATION ? TuningConstants.ELEVATOR_VOLTAGE_COMPENSATION : 1.0);

    // Elevator Protection
    public static final boolean ELEVATOR_USE_LIMIT_SWITCH = true;

    public static final double ELEVATOR_POSITION_TOLERANCE = 0.75;

    public static final double ELEVATOR_MIN_POWER_VALUE = 0.0;
    public static final double ELEVATOR_MAX_POWER_VALUE = 12.0 * 200.0;
    public static final double BATTERY_AVERAGE_EXPECTED_VOLTAGE = 12.0;

    public static final boolean ELEVATOR_USE_VOLTAGE_COMPENSATION = true;
    public static final double ELEVATOR_VOLTAGE_COMPENSATION = 12.0;
    public static final boolean ELEVATOR_SUPPLY_CURRENT_LIMITING_ENABLED = true;
    public static final double ELEVATOR_SUPPLY_CURRENT_MAX = 40.0;
    public static final double ELEVATOR_SUPPLY_TRIGGER_CURRENT = 50.0;
    public static final double ELEVATOR_SUPPLY_TRIGGER_DURATION = 0.1;
    public static final boolean ELEVATOR_STATOR_CURRENT_LIMITING_ENABLED = true;
    public static final double ELEVATOR_STATOR_CURRENT_LIMIT = 80.0;

    public static final int ELEVATOR_FEEDBACK_UPDATE_RATE_HZ = 100;
    public static final int ELEVATOR_ERROR_UPDATE_RATE_HZ = 10;
    public static final int ELEVATOR_OUTPUT_UPDATE_RATE_HZ = 100;
    public static final int ELEVATOR_FOLLOWER_OUTPUT_UPDATE_RATE_HZ = 100;

    public static final double ELEVATOR_MOTOR_POWER_MIN_DIFFERENCE = 0.5; // Watts
    public static final double ELEVATOR_POWER_DIFFERENCE = 0.50; // Percentage difference allowed between the two motors

    public static final double ELEVATOR_POWER_TRACKING_DURATION = 1.0;
    public static final double ELEVATOR_POWER_SAMPLES_PER_SECOND = TuningConstants.LOOPS_PER_SECOND;
    public static final double ELEVATOR_VELOCITY_TRACKING_DURATION = TuningConstants.ELEVATOR_POWER_TRACKING_DURATION;
    public static final double ELEVATOR_VELOCITY_SAMPLES_PER_SECOND = TuningConstants.ELEVATOR_POWER_SAMPLES_PER_SECOND;

    public static final double ELEVATOR_VELOCITY_AVERAGE_VALUE = 0.0;

    public static final boolean ELEVATOR_INVERT_SENSOR = false;
    public static final boolean ELEVATOR_INVERT_MOTOR = false;

    public static final boolean ELEVATOR_USE_STALL_MODE = true;
    public static final double ELEVATOR_POWER_STALLED_THRESHOLD = 100.0; // in watts
    public static final double ELEVATOR_VELOCITY_STALLED_THRESHOLD = 0.5;

    public static final double ELEVATOR_NET_HEIGHT = 0.0; // CHANGE LATER

    //================================================== Climber ==============================================================

    public static final boolean CLIMBER_USE_PID = true;

    public static final boolean CLIMBER_WINCH_INVERTED = false;
    public static final boolean CLIMBER_ELBOW_MOTOR_INVERTED = true;
    public static final boolean CLIMBER_ELBOW_ENCODER_INVERTED = true;

    public static final boolean CLIMBER_WINCH_BRAKE_AFTER_WINCHING = true;

    public static final boolean CLIMBER_USE_LIMIT_SWITCH = false;

    public static final double CLIMBER_WINCH_POWER = -0.9;
    public static final double CLIMBER_ELBOW_WINCHING_POWER = -0.35;

    public static final double CLIMBER_ELBOW_ABSOLUTE_ENCODER_OFFSET = (165.0) / 360.0;

    public static final double CLIMBER_ELBOW_SPARKMAX_PID_KP = 0.0;
    public static final double CLIMBER_ELBOW_SPARKMAX_PID_KI = 0.0;
    public static final double CLIMBER_ELBOW_SPARKMAX_PID_KD = 0.0;
    public static final double CLIMBER_ELBOW_SPARKMAX_PID_KF = 0.0;
    public static final double CLIMBER_ELBOW_SPARKMAX_PID_MIN_VALUE = -0.8;
    public static final double CLIMBER_ELBOW_SPARKMAX_PID_MAX_VALUE = 0.8;

    public static final double CLIMBER_ELBOW_ROBORIO_PID_KP = 0.02;
    public static final double CLIMBER_ELBOW_ROBORIO_PID_KI = 0.0;
    public static final double CLIMBER_ELBOW_ROBORIO_PID_KD = 0.001;
    public static final double CLIMBER_ELBOW_ROBORIO_PID_KF = 0.0;
    public static final double CLIMBER_ELBOW_ROBORIO_PID_KS = 1.0;
    public static final double CLIMBER_ELBOW_ROBORIO_PID_MIN_VALUE = -0.35;
    public static final double CLIMBER_ELBOW_ROBORIO_PID_MAX_VALUE = 0.35;

    public static final double CLIMBER_ELBOW_ANGLE_RANGE_START = 1.0; // degrees
    public static final double CLIMBER_ELBOW_ANGLE_RANGE_END = 160.0; // degrees
    public static final double CLIMBER_ELBOW_OUT_ANGLE = 156.0; // degrees

    public static final double CLIMBER_ELBOW_SUPER_DOWN_ANGLE = 35.0; // degrees
    public static final double CLIMBER_ELBOW_DOWN_ANGLE = 56.5; // degrees
    public static final double CLIMBER_ELBOW_INITIAL_ANGLE = 25.0; // degrees
    public static final double CLIMBER_ELBOW_POSITION_TOLERANCE = 2.0; // degrees

    public static final double CLIMBER_ELBOW_ANGLE_ADJUSTMENT_VELOCITY = 15.0; // degrees per second

    public static final boolean CLIMBER_ELBOW_USE_STALL_MODE = false;

    public static final double CLIMBER_ELBOW_STALLED_POWER_THRESHOLD = 0;
    public static final double CLIMBER_ELBOW_STALLED_VELOCITY_THRESHOLD = 0;

    public static final double CLIMBER_ELBOW_POWER_TRACKING_DURATION = 0.5;
    public static final double CLIMBER_ELBOW_POWER_SAMPLES_PER_LOOP = 1.0; // we may want to increase this if we find our update loop duration isn't very consistent...
    public static final double CLIMBER_ELBOW_POWER_SAMPLES_PER_SECOND = TuningConstants.LOOPS_PER_SECOND * TuningConstants.POWER_OVERCURRENT_SAMPLES_PER_LOOP;
    public static final double CLIMBER_ELBOW_MIN_POWER_VALUE = 0.0; // in watts
    public static final double CLIMBER_ELBOW_MAX_POWER_VALUE = 240.0; // in watts
    public static final double CLIMBER_ELBOW_VELOCITY_TRACKING_DURATION = TuningConstants.CLIMBER_ELBOW_POWER_TRACKING_DURATION;
    public static final double CLIMBER_ELBOW_VELOCITY_SAMPLES_PER_SECOND = TuningConstants.CLIMBER_ELBOW_POWER_SAMPLES_PER_SECOND;
    // need to adjust
    public static final double CLIMBER_ELBOW_DESIRE_DIFFERENCE_THRESHOLD = 5.0;
}
