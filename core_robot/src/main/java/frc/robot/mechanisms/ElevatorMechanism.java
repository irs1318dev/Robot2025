package frc.robot.mechanisms;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.lib.driver.IDriver;
import frc.lib.filters.FloatingAverageCalculator;
import frc.lib.helpers.Helpers;
import frc.lib.mechanisms.IMechanism;
import frc.lib.mechanisms.LoggingManager;
import frc.lib.robotprovider.IDigitalInput;
import frc.lib.robotprovider.ILogger;
import frc.lib.robotprovider.IRobotProvider;
import frc.lib.robotprovider.ITalonFX;
import frc.lib.robotprovider.ITimer;
import frc.lib.robotprovider.MotorNeutralMode;
import frc.lib.robotprovider.RobotMode;
import frc.lib.robotprovider.TalonFXControlMode;
import frc.robot.ElectronicsConstants;
import frc.robot.HardwareConstants;
import frc.robot.LoggingKey;
import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;

@Singleton
public class ElevatorMechanism implements IMechanism
{
    private final ILogger logger;
    private final IDriver driver;

    private final ITimer timer;
    private double prevTime;

    private final PowerManager powerManager;

    private final CoralEndEffectorMechanism coralIntake;
    private final AlgaeIntakeMechanism algaeIntake;

    private final ITalonFX elevatorMotor;
    private final ITalonFX elevatorFollowerMotor;
    private final TalonFXControlMode pidControlMode;
    private double elevatorDesiredPosition;
    private boolean inSimpleMode;

    private double elevatorVelocity;
    private double elevatorPosition; 
    private double elevatorError;

    private final IDigitalInput topLimitSwitch;
    private final IDigitalInput bottomLimitSwitch;
    private boolean topSwitchPressed;
    private boolean bottomSwitchPressed;

    private boolean elevatorStalled;
    private double elevatorDesiredPositionChangedTime;

    private final FloatingAverageCalculator elevatorMasterPowerAverageCalculator;
    private final FloatingAverageCalculator elevatorFollowerPowerAverageCalculator;
    private double elevatorMasterPowerAverage;
    private double elevatorFollowerPowerAverage;
    private double elevatorPowerAverage;

    private final FloatingAverageCalculator elevatorVelocityAverageCalculator;
    private double elevatorVelocityAverage;

    @Inject
    public ElevatorMechanism(
        IRobotProvider provider,
        IDriver driver,
        ITimer timer,
        PowerManager powerManager,
        CoralEndEffectorMechanism coralIntake,
        AlgaeIntakeMechanism algaeIntake,
        LoggingManager logger)
    {
        this.driver = driver;
        this.timer = timer;
        this.powerManager = powerManager;
        this.coralIntake = coralIntake;
        this.algaeIntake = algaeIntake;
        this.logger = logger;

        this.pidControlMode = TuningConstants.ELEVATOR_USE_MOTION_MAGIC
            ? TalonFXControlMode.MotionMagicPosition
            : TalonFXControlMode.Position;

        this.inSimpleMode = !TuningConstants.ELEVATOR_IN_PID_MODE;

        this.elevatorMotor = provider.getTalonFX(ElectronicsConstants.ELEVATOR_MOTOR_CAN_ID, ElectronicsConstants.CANIVORE_NAME);
        this.elevatorMotor.setMotorOutputSettings(false, MotorNeutralMode.Brake);
        this.elevatorMotor.setVoltageCompensation(TuningConstants.ELEVATOR_USE_VOLTAGE_COMPENSATION, TuningConstants.ELEVATOR_VOLTAGE_COMPENSATION);
        this.elevatorMotor.setCurrentLimit(
            TuningConstants.ELEVATOR_SUPPLY_CURRENT_LIMITING_ENABLED,
            TuningConstants.ELEVATOR_SUPPLY_CURRENT_MAX,
            TuningConstants.ELEVATOR_SUPPLY_TRIGGER_CURRENT,
            TuningConstants.ELEVATOR_SUPPLY_TRIGGER_DURATION,
            TuningConstants.ELEVATOR_STATOR_CURRENT_LIMITING_ENABLED,
            TuningConstants.ELEVATOR_STATOR_CURRENT_LIMIT);
        this.elevatorMotor.setFeedbackUpdateRate(TuningConstants.ELEVATOR_FEEDBACK_UPDATE_RATE_HZ);
        this.elevatorMotor.setErrorUpdateRate(TuningConstants.ELEVATOR_ERROR_UPDATE_RATE_HZ);
        this.elevatorMotor.setOutputUpdateRate(TuningConstants.ELEVATOR_OUTPUT_UPDATE_RATE_HZ);
        this.elevatorMotor.setControlMode(this.inSimpleMode ? TalonFXControlMode.PercentOutput : this.pidControlMode);
        this.elevatorMotor.setPosition(HardwareConstants.ELEVATOR_START_HEIGHT * HardwareConstants.ELEVATOR_TICKS_PER_INCH);

        this.elevatorFollowerMotor = provider.getTalonFX(ElectronicsConstants.ELEVATOR_FOLLOWER_MOTOR_CAN_ID, ElectronicsConstants.CANIVORE_NAME);
        this.elevatorFollowerMotor.setMotorOutputSettings(false, MotorNeutralMode.Brake);
        this.elevatorFollowerMotor.setVoltageCompensation(TuningConstants.ELEVATOR_USE_VOLTAGE_COMPENSATION, TuningConstants.ELEVATOR_VOLTAGE_COMPENSATION);
        this.elevatorFollowerMotor.setCurrentLimit(
            TuningConstants.ELEVATOR_SUPPLY_CURRENT_LIMITING_ENABLED,
            TuningConstants.ELEVATOR_SUPPLY_CURRENT_MAX,
            TuningConstants.ELEVATOR_SUPPLY_TRIGGER_CURRENT,
            TuningConstants.ELEVATOR_SUPPLY_TRIGGER_DURATION,
            TuningConstants.ELEVATOR_STATOR_CURRENT_LIMITING_ENABLED,
            TuningConstants.ELEVATOR_STATOR_CURRENT_LIMIT);
        this.elevatorFollowerMotor.setOutputUpdateRate(TuningConstants.ELEVATOR_FOLLOWER_OUTPUT_UPDATE_RATE_HZ);
        this.elevatorFollowerMotor.follow(this.elevatorMotor);

        this.elevatorFollowerPowerAverageCalculator = new FloatingAverageCalculator(this.timer, TuningConstants.ELEVATOR_MIN_POWER_VALUE, TuningConstants.ELEVATOR_MAX_POWER_VALUE, TuningConstants.ELEVATOR_POWER_TRACKING_DURATION, TuningConstants.ELEVATOR_POWER_SAMPLES_PER_SECOND);
        this.elevatorMasterPowerAverageCalculator = new FloatingAverageCalculator(this.timer, TuningConstants.ELEVATOR_MIN_POWER_VALUE, TuningConstants.ELEVATOR_MAX_POWER_VALUE, TuningConstants.ELEVATOR_POWER_TRACKING_DURATION, TuningConstants.ELEVATOR_POWER_SAMPLES_PER_SECOND);
        this.elevatorVelocityAverageCalculator = new FloatingAverageCalculator(this.timer, TuningConstants.ELEVATOR_VELOCITY_TRACKING_DURATION, TuningConstants.ELEVATOR_VELOCITY_SAMPLES_PER_SECOND);

        if (TuningConstants.ELEVATOR_USE_MOTION_MAGIC)
        {
            this.elevatorMotor.setMotionMagicPIDVS(
                TuningConstants.ELEVATOR_MOTION_MAGIC_NGP_MM_PID_KP,
                TuningConstants.ELEVATOR_MOTION_MAGIC_NGP_MM_PID_KI,
                TuningConstants.ELEVATOR_MOTION_MAGIC_NGP_MM_PID_KD,
                TuningConstants.ELEVATOR_MOTION_MAGIC_NGP_MM_PID_KV,
                TuningConstants.ELEVATOR_MOTION_MAGIC_NGP_MM_PID_KS,
                TuningConstants.ELEVATOR_MOTION_MAGIC_NGP_MM_PID_CRUISE_VELOCITY,
                TuningConstants.ELEVATOR_MOTION_MAGIC_NGP_MM_PID_MAX_ACCELERATION,
                TuningConstants.ELEVATOR_MOTION_MAGIC_NGP_MM_PID_MAX_JERK,
                TuningConstants.ELEVATOR_NGP_PID_SLOT);
            this.elevatorMotor.setMotionMagicPIDVS(
                TuningConstants.ELEVATOR_MOTION_MAGIC_GP_MM_PID_KP,
                TuningConstants.ELEVATOR_MOTION_MAGIC_GP_MM_PID_KI,
                TuningConstants.ELEVATOR_MOTION_MAGIC_GP_MM_PID_KD,
                TuningConstants.ELEVATOR_MOTION_MAGIC_GP_MM_PID_KV,
                TuningConstants.ELEVATOR_MOTION_MAGIC_GP_MM_PID_KS,
                TuningConstants.ELEVATOR_MOTION_MAGIC_GP_MM_PID_CRUISE_VELOCITY,
                TuningConstants.ELEVATOR_MOTION_MAGIC_GP_MM_PID_MAX_ACCELERATION,
                TuningConstants.ELEVATOR_MOTION_MAGIC_GP_MM_PID_MAX_JERK,
                TuningConstants.ELEVATOR_GP_PID_SLOT);
        }
        else
        {
            this.elevatorMotor.setPIDF(
                TuningConstants.ELEVATOR_POSITION_NGP_PID_KP,
                TuningConstants.ELEVATOR_POSITION_NGP_PID_KI,
                TuningConstants.ELEVATOR_POSITION_NGP_PID_KD,
                TuningConstants.ELEVATOR_POSITION_NGP_PID_KF,
                TuningConstants.ELEVATOR_NGP_PID_SLOT);
            this.elevatorMotor.setPIDF(
                TuningConstants.ELEVATOR_POSITION_GP_PID_KP,
                TuningConstants.ELEVATOR_POSITION_GP_PID_KI,
                TuningConstants.ELEVATOR_POSITION_GP_PID_KD,
                TuningConstants.ELEVATOR_POSITION_GP_PID_KF,
                TuningConstants.ELEVATOR_GP_PID_SLOT);
        }

        this.elevatorMotor.setSelectedSlot(TuningConstants.ELEVATOR_GP_PID_SLOT);

        // Tell the motor controllers to optimize their canbus utilization so they don't log signals frequently that we don't care about
        // this.elevatorMotor.optimizeCanbus();
        // this.elevatorFollowerMotor.optimizeCanbus();

        this.topLimitSwitch = provider.getDigitalInput(ElectronicsConstants.ELEVATOR_TOP_LIMIT_SWITCH_DIO);
        this.topLimitSwitch.setInverted(ElectronicsConstants.ELEVATOR_TOP_LIMIT_SWITCH_INVERTED);
        this.bottomLimitSwitch = provider.getDigitalInput(ElectronicsConstants.ELEVATOR_BOTTOM_LIMIT_SWITCH_DIO);
        this.bottomLimitSwitch.setInverted(ElectronicsConstants.ELEVATOR_BOTTOM_LIMIT_SWITCH_INVERTED);

        this.elevatorStalled = false;
    }

    @Override
    public void readSensors()
    {
        // collect power information from power manager
        double batteryVoltage = this.powerManager.getBatteryVoltage();
        double elevatorMasterCurrent = this.powerManager.getCurrent(ElectronicsConstants.ELEVATOR_MOTOR_PDH_CHANNEL);
        double elevatorFollowerCurrent = this.powerManager.getCurrent(ElectronicsConstants.ELEVATOR_MOTOR_FOLLOWER_PDH_CHANNEL);

        // Calculates average power for each elevator motor
        this.elevatorMasterPowerAverage = this.elevatorMasterPowerAverageCalculator.update(elevatorMasterCurrent * batteryVoltage);
        this.elevatorFollowerPowerAverage = this.elevatorFollowerPowerAverageCalculator.update(elevatorFollowerCurrent * batteryVoltage);

        // Calculates average power for the elevator motors (combined)
        this.elevatorPowerAverage = 0.5 * (this.elevatorMasterPowerAverage + this.elevatorFollowerPowerAverage);

        this.logger.logNumber(LoggingKey.ElevatorMasterPowerAverage, this.elevatorMasterPowerAverage);
        this.logger.logNumber(LoggingKey.ElevatorFollowerPowerAverage, this.elevatorFollowerPowerAverage);
        this.logger.logNumber(LoggingKey.ElevatorPowerAverage, this.elevatorPowerAverage);

        // collect sensor values from elevator motor and 
        this.elevatorPosition = this.elevatorMotor.getPosition() * HardwareConstants.ELEVATOR_TICK_DISTANCE; // ticks * (inches / tick) = inches
        this.elevatorVelocity = this.elevatorMotor.getVelocity() * HardwareConstants.ELEVATOR_TICK_DISTANCE;
        this.elevatorError = this.elevatorMotor.getError();

        this.logger.logNumber(LoggingKey.ElevatorPosition, this.elevatorPosition);
        this.logger.logNumber(LoggingKey.ElevatorVelocity, this.elevatorVelocity);
        this.logger.logNumber(LoggingKey.ElevatorError, this.elevatorError);

        this.topSwitchPressed = this.topLimitSwitch.get();
        this.bottomSwitchPressed = this.bottomLimitSwitch.get();

        this.logger.logBoolean(LoggingKey.ElevatorTopSwitch, this.topSwitchPressed);
        this.logger.logBoolean(LoggingKey.ElevatorBottomSwitch, this.bottomSwitchPressed);

        this.elevatorVelocityAverage = this.elevatorVelocityAverageCalculator.update(this.elevatorVelocity);

        this.logger.logNumber(LoggingKey.ElevatorVelocityAverage, this.elevatorVelocityAverage);

        // Check for power discrepancy between the master and follower motors for the elevator
        boolean isElevatorMotorPowerDiscrepancy = false;
        if (this.elevatorPowerAverage > TuningConstants.ELEVATOR_MOTOR_POWER_MIN_DIFFERENCE)
        {
            isElevatorMotorPowerDiscrepancy = Math.abs(this.elevatorMasterPowerAverage - this.elevatorFollowerPowerAverage) / this.elevatorPowerAverage >= TuningConstants.ELEVATOR_POWER_DIFFERENCE;
        }

        this.logger.logBoolean(LoggingKey.ElevatorPowerDiscrepancy, isElevatorMotorPowerDiscrepancy);
    }

    @Override
    public void update(RobotMode mode)
    {
        double currTime = this.timer.get();
        double elapsedTime = currTime - this.prevTime;

        if (driver.getDigital(DigitalOperation.CoalWristResetPosition))
        {
            elevatorPosition = 0.0;
            elevatorMotor.setPosition(0.0);
        }

        if (!this.inSimpleMode && this.driver.getDigital(DigitalOperation.ElevatorDisablePID))
        {
            this.inSimpleMode = true;
            this.elevatorMotor.setControlMode(TalonFXControlMode.PercentOutput);
        }
        else if (this.inSimpleMode && this.driver.getDigital(DigitalOperation.ElevatorEnablePID))
        {
            this.elevatorMotor.setControlMode(this.pidControlMode);
            if (this.coralIntake.hasGamePiece() ||
                this.algaeIntake.hasGamePiece())
            {
                this.elevatorMotor.setSelectedSlot(TuningConstants.ELEVATOR_GP_PID_SLOT);
            }
            else
            {
                this.elevatorMotor.setSelectedSlot(TuningConstants.ELEVATOR_NGP_PID_SLOT);
            }

            this.inSimpleMode = false;

            this.elevatorDesiredPosition = this.elevatorPosition;
            this.elevatorDesiredPositionChangedTime = currTime;
            this.elevatorStalled = false;
        }

        boolean useSimpleMode;

        double elevatorPowerAdjustment = this.driver.getAnalog(AnalogOperation.ElevatorPower);
        double elevatorMotorPower = 0.0;
        if (this.inSimpleMode)
        {
            useSimpleMode = true;
            elevatorMotorPower = elevatorPowerAdjustment;

            this.elevatorDesiredPositionChangedTime = currTime;
            this.elevatorStalled = false;
        }
        else
        {
            if (elevatorPowerAdjustment != 0.0)
            {
                useSimpleMode = true;

                elevatorMotorPower = elevatorPowerAdjustment;
                if (this.elevatorPosition < HardwareConstants.ELEVATOR_MIN_HEIGHT)
                {
                    this.elevatorMotor.setPosition(HardwareConstants.ELEVATOR_MIN_HEIGHT);
                    this.elevatorPosition = HardwareConstants.ELEVATOR_MIN_HEIGHT;
                }
                else if (this.elevatorPosition > HardwareConstants.ELEVATOR_MAX_HEIGHT)
                {
                    this.elevatorMotor.setPosition(HardwareConstants.ELEVATOR_MAX_HEIGHT);
                    this.elevatorPosition = HardwareConstants.ELEVATOR_MAX_HEIGHT;
                }

                this.elevatorDesiredPosition = this.elevatorPosition;
            }
            else
            {
                useSimpleMode = false;

                double newDesiredElevatorPosition = this.driver.getAnalog(AnalogOperation.ElevatorDesiredPosition);
                double newElevatorPositionAdjustment = this.driver.getAnalog(AnalogOperation.ElevatorPositionAdjustment) * TuningConstants.ELEVATOR_ADJUSTMENT_MULTIPLIER * elapsedTime;

                if (newDesiredElevatorPosition != TuningConstants.MAGIC_NULL_VALUE)
                {
                    this.elevatorDesiredPositionChangedTime = currTime;
                    this.elevatorStalled = false;

                    this.elevatorDesiredPosition = newDesiredElevatorPosition;
                }
                else if (newElevatorPositionAdjustment != 0.0)
                {
                    this.elevatorDesiredPositionChangedTime = currTime;
                    this.elevatorStalled = false;

                    this.elevatorDesiredPosition += newElevatorPositionAdjustment;
                }

                if (newDesiredElevatorPosition != TuningConstants.MAGIC_NULL_VALUE || newElevatorPositionAdjustment != 0.0)
                {
                    double clampedDesiredElevatorPosition = Helpers.enforceRange(this.elevatorDesiredPosition, HardwareConstants.ELEVATOR_MIN_HEIGHT, HardwareConstants.ELEVATOR_MAX_HEIGHT);
                    this.elevatorDesiredPosition = clampedDesiredElevatorPosition;
                }
            }
        }

        if (TuningConstants.ELEVATOR_USE_LIMIT_SWITCH)
        {
            // if (this.topSwitchPressed)
            // {
            //     this.elevatorPosition = HardwareConstants.ELEVATOR_MAX_HEIGHT;
            //     if (this.elevatorDesiredPosition > this.elevatorPosition)
            //     {
            //         this.elevatorDesiredPosition = this.elevatorPosition;
            //     }

            //     this.elevatorMotor.setPosition(this.elevatorPosition * HardwareConstants.ELEVATOR_TICKS_PER_INCH);
            // }

            if (this.bottomSwitchPressed)
            {
                if (this.elevatorDesiredPosition <= this.elevatorPosition || this.elevatorPosition < HardwareConstants.ELEVATOR_MIN_HEIGHT)
                {
                    this.elevatorPosition = HardwareConstants.ELEVATOR_MIN_HEIGHT;
                    this.elevatorDesiredPosition = this.elevatorPosition;
                    this.elevatorMotor.setPosition(this.elevatorPosition * HardwareConstants.ELEVATOR_TICKS_PER_INCH);
                }
            }
        }

        if (TuningConstants.ELEVATOR_USE_STALL_MODE)
        {
            if (currTime > this.elevatorDesiredPositionChangedTime + TuningConstants.ELEVATOR_VELOCITY_TRACKING_DURATION &&
                this.elevatorPowerAverage >= TuningConstants.ELEVATOR_POWER_STALLED_THRESHOLD &&
                Math.abs(this.elevatorVelocityAverage) <= TuningConstants.ELEVATOR_VELOCITY_STALLED_THRESHOLD)
            {
                this.elevatorStalled = true;
            }
        }

        double elevatorOutput;
        if (this.elevatorStalled)
        {
            elevatorOutput = 0.0;
            this.elevatorMotor.stop();
        }
        else if (useSimpleMode)
        {
            elevatorOutput = elevatorMotorPower;
            this.elevatorMotor.set(TalonFXControlMode.PercentOutput, elevatorMotorPower);
        }
        else
        {
            if (TuningConstants.ELEVATOR_USE_GRAVITY_COMPENSATION)
            {
                this.elevatorMotor.set(this.pidControlMode, this.elevatorDesiredPosition * HardwareConstants.ELEVATOR_TICKS_PER_INCH, TuningConstants.ELEVATOR_GRAVITY_COMPENSATION);
            }
            else
            {
                this.elevatorMotor.set(this.pidControlMode, this.elevatorDesiredPosition * HardwareConstants.ELEVATOR_TICKS_PER_INCH);
            }

            elevatorOutput = this.elevatorMotor.getOutput();
        }

        this.logger.logNumber(LoggingKey.ElevatorDesiredPosition, this.elevatorDesiredPosition);
        this.logger.logBoolean(LoggingKey.ElevatorStallMode, this.elevatorStalled);

        this.logger.logNumber(LoggingKey.ElevatorOutput, elevatorOutput);

        this.prevTime = currTime;
    }

    @Override
    public void stop()
    {
        this.elevatorMotor.stop();

        this.elevatorVelocity = 0.0;
        this.elevatorPosition = 0.0;
        this.elevatorError = 0.0;
        this.elevatorPosition = 0.0;
    }

    public double getPosition()
    {
        return this.elevatorPosition;
    }

    public double getCurrentElevatorDesiredPosition()
    {
        return this.elevatorDesiredPosition;
    }
    
    public boolean getTopLimitSwitch()
    {
        return this.topSwitchPressed;
    }

    public boolean getBottomLimitSwitch()
    {
        return this.bottomSwitchPressed;
    }
}
