package frc.robot.mechanisms;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.lib.driver.IDriver;
import frc.lib.filters.BooleanThresholdFilter;
import frc.lib.filters.FloatingAverageCalculator;
import frc.lib.helpers.ExceptionHelpers;
import frc.lib.helpers.Helpers;
import frc.lib.mechanisms.IMechanism;
import frc.lib.mechanisms.LoggingManager;
import frc.lib.robotprovider.IDigitalInput;
import frc.lib.robotprovider.IRobotProvider;
import frc.lib.robotprovider.ISparkMax;
import frc.lib.robotprovider.ITalonSRX;
import frc.lib.robotprovider.ITimer;
import frc.lib.robotprovider.MotorNeutralMode;
import frc.lib.robotprovider.RobotMode;
import frc.lib.robotprovider.SparkControlMode;
import frc.lib.robotprovider.SparkMotorType;
import frc.lib.robotprovider.SparkSignalType;
import frc.lib.robotprovider.TalonSRXControlMode;
import frc.robot.ElectronicsConstants;
import frc.robot.HardwareConstants;
import frc.robot.LoggingKey;
import frc.robot.TuningConstants;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;

@Singleton
public class CoralEndEffectorMechanism implements IMechanism
{
    private final IDriver driver;
    private final ITimer timer;

    private final LoggingManager logger;
    private final PowerManager powerManager;

    private final IDigitalInput throughBeamSensor;
    private final BooleanThresholdFilter throughBeamFilter;
    private boolean throughBeamBroken;

    // Motors
    private final ISparkMax wristMotor;
    private final ITalonSRX intakeMotor;

    private double wristDesiredAngle; 
    private double wristAngle;
    private double wristVelocity;

    private final FloatingAverageCalculator wristPowerAverageCalculator;
    private double wristPowerAverage;

    private final FloatingAverageCalculator wristVelocityAverageCalculator;
    private double wristVelocityAverage;

    private final FloatingAverageCalculator intakePowerAverageCalculator;
    private double intakePowerAverage;

    private boolean inSimpleMode;

    private boolean isWristStalled;
    private double wristDesiredAngleChangedTime;

    private double prevTime;

    @Inject
    public CoralEndEffectorMechanism(
        IRobotProvider provider,
        IDriver driver,
        ITimer timer,
        LoggingManager logger,
        PowerManager powerManager)
    {
        this.driver = driver;
        this.timer = timer;
        this.logger = logger;
        this.powerManager = powerManager;

        // Motors
        this.intakeMotor = provider.getTalonSRX(ElectronicsConstants.CORAL_INTAKE_MOTOR_CAN_ID);
        this.intakeMotor.setMotorOutputSettings(TuningConstants.CORAL_INTAKE_MOTOR_INVERT_OUTPUT, MotorNeutralMode.Brake);
        this.intakeMotor.setControlMode(TalonSRXControlMode.PercentOutput);

        this.wristMotor = provider.getSparkMax(ElectronicsConstants.CORAL_WRIST_MOTOR_CAN_ID, SparkMotorType.Brushed);
        this.wristMotor.configureMotorOutputSettings(TuningConstants.CORAL_WRIST_MOTOR_INVERT_OUTPUT, MotorNeutralMode.Brake);
        this.wristMotor.configureAbsoluteEncoder();
        this.wristMotor.configureAbsoluteOffset(TuningConstants.CORAL_WRIST_ABSOLUTE_ENCODER_OFFSET);
        this.wristMotor.configureInvertSensor(TuningConstants.CORAL_WRIST_MOTOR_INVERT_SENSOR);
        this.wristMotor.configurePositionConversionFactor(HardwareConstants.CORAL_INTAKE_MOTOR_TICK_DISTANCE);
        this.wristMotor.configureVelocityConversionFactor(HardwareConstants.CORAL_INTAKE_MOTOR_TICK_DISTANCE);
        this.wristMotor.configureSignals(SparkSignalType.AbsoluteEncoderPosition, 10);
        this.wristMotor.configureSignals(SparkSignalType.AbsoluteEncoderVelocity, 10);

        if (TuningConstants.CORAL_WRIST_USE_PID)
        {
            if (TuningConstants.CORAL_WRIST_USE_TMP)
            {
                this.wristMotor.configurePIDFMAXMotion(
                    TuningConstants.CORAL_WITHOUT_TMP_WRIST_KP,
                    TuningConstants.CORAL_WITHOUT_TMP_WRIST_KI,
                    TuningConstants.CORAL_WITHOUT_TMP_WRIST_KD,
                    TuningConstants.CORAL_WITHOUT_TMP_WRIST_KF,
                    0.0,
                    TuningConstants.CORAL_WITHOUT_TMP_WRIST_CRUISE_VELOCITY,
                    TuningConstants.CORAL_WITHOUT_TMP_WRIST_MAX_ACCELERATION,
                    TuningConstants.CORAL_WRIST_MIN_OUTPUT,
                    TuningConstants.CORAL_WRIST_MAX_OUTPUT,
                    TuningConstants.CORAL_WITHOUT_PID_SLOT);

                this.wristMotor.configurePIDFMAXMotion(
                    TuningConstants.CORAL_WITH_TMP_WRIST_KP,
                    TuningConstants.CORAL_WITH_TMP_WRIST_KI,
                    TuningConstants.CORAL_WITH_TMP_WRIST_KD,
                    TuningConstants.CORAL_WITH_TMP_WRIST_KF,
                    0.0,
                    TuningConstants.CORAL_WITH_TMP_WRIST_CRUISE_VELOCITY,
                    TuningConstants.CORAL_WITH_TMP_WRIST_MAX_ACCELERATION,
                    TuningConstants.CORAL_WRIST_MIN_OUTPUT,
                    TuningConstants.CORAL_WRIST_MAX_OUTPUT,
                    TuningConstants.CORAL_WITH_PID_SLOT);

                this.wristMotor.setControlMode(SparkControlMode.MAXMotionPosition);
            }
            else
            {
                this.wristMotor.configurePIDF(
                    TuningConstants.CORAL_WITHOUT_PID_WRIST_KP,
                    TuningConstants.CORAL_WITHOUT_PID_WRIST_KI,
                    TuningConstants.CORAL_WITHOUT_PID_WRIST_KD,
                    TuningConstants.CORAL_WITHOUT_PID_WRIST_KF,
                    TuningConstants.CORAL_WRIST_MIN_OUTPUT,
                    TuningConstants.CORAL_WRIST_MAX_OUTPUT,
                    TuningConstants.CORAL_WITHOUT_PID_SLOT);

                this.wristMotor.configurePIDF(
                    TuningConstants.CORAL_WITH_PID_WRIST_KP,
                    TuningConstants.CORAL_WITH_PID_WRIST_KI,
                    TuningConstants.CORAL_WITH_PID_WRIST_KD,
                    TuningConstants.CORAL_WITH_PID_WRIST_KF,
                    TuningConstants.CORAL_WRIST_MIN_OUTPUT,
                    TuningConstants.CORAL_WRIST_MAX_OUTPUT,
                    TuningConstants.CORAL_WITH_PID_SLOT);

                this.wristMotor.setControlMode(SparkControlMode.Position);
            }

            this.wristMotor.configurePositionPIDWrappingSettings(
                TuningConstants.CORAL_WRIST_POSITION_PID_WRAPPING_ENABLED,
                TuningConstants.CORAL_WRIST_POSITION_PID_WRAPPING_MIN,
                TuningConstants.CORAL_WRIST_POSITION_PID_WRAPPING_MAX);
            this.inSimpleMode = false;
        }
        else
        {
            this.wristMotor.setControlMode(SparkControlMode.PercentOutput);
            this.inSimpleMode = true;
        }

        this.wristMotor.setSelectedSlot(TuningConstants.CORAL_WITH_PID_SLOT);
        this.wristMotor.applyConfiguration(true);

        this.wristAngle = this.wristMotor.getPosition(); 
        this.wristDesiredAngle = TuningConstants.CORAL_WRIST_STOW_ANGLE;
        this.wristVelocity = 0.0;

        // Sensors
        this.throughBeamSensor = provider.getDigitalInput(ElectronicsConstants.CORAL_THROUGH_BEAM_SENSOR_INPUT_DIO_CHANNEL);
        this.throughBeamSensor.setInverted(ElectronicsConstants.CORAL_THROUGH_BEAM_SENSOR_INVERTED);
        this.throughBeamFilter = new BooleanThresholdFilter(2);
        this.throughBeamBroken = true;

        // Stall
        this.wristDesiredAngleChangedTime = 0.0;
        this.isWristStalled = false;

        this.intakePowerAverageCalculator = new FloatingAverageCalculator(this.timer, TuningConstants.CORAL_INTAKE_MIN_POWER_VALUE, TuningConstants.CORAL_INTAKE_MAX_POWER_VALUE, TuningConstants.CORAL_POWER_TRACKING_DURATION, TuningConstants.CORAL_POWER_SAMPLES_PER_SECOND);
        this.wristPowerAverageCalculator = new FloatingAverageCalculator(this.timer, TuningConstants.CORAL_WRIST_MIN_POWER_VALUE, TuningConstants.CORAL_WRIST_MAX_POWER_VALUE, TuningConstants.CORAL_POWER_TRACKING_DURATION, TuningConstants.CORAL_POWER_SAMPLES_PER_SECOND);
        this.wristVelocityAverageCalculator = new FloatingAverageCalculator(this.timer, TuningConstants.CORAL_VELOCITY_TRACKING_DURATION, TuningConstants.CORAL_VELOCITY_SAMPLES_PER_SECOND);

        this.wristPowerAverage = 0.0;
        this.wristVelocityAverage = 0.0;
        this.intakePowerAverage = 0.0;
    }

    @Override
    public void readSensors()
    {
        // wrist sensors
        this.wristAngle = this.wristMotor.getPosition();
        this.wristVelocity = this.wristMotor.getVelocity();

        this.logger.logNumber(LoggingKey.CoralEndEffectorWristAngle, this.wristAngle);
        this.logger.logNumber(LoggingKey.CoralEndEffectorWristVelocity, this.wristVelocity);
        this.logger.logNumber(LoggingKey.CoralEndEffectorWristVelocityAverage, this.wristVelocityAverage);

        // through-beam sensor
        boolean throughBeamBroken = this.throughBeamSensor.get();
        this.throughBeamBroken = this.throughBeamFilter.update(throughBeamBroken);

        this.logger.logBoolean(LoggingKey.CoralEndEffectorThroughBeamBroken, this.throughBeamBroken);

        // power information
        double batteryVoltage = this.powerManager.getBatteryVoltage();
        double intakeCurrent = this.powerManager.getCurrent(ElectronicsConstants.CORAL_INTAKE_PDH_CHANNEL);
        double wristCurrent = this.powerManager.getCurrent(ElectronicsConstants.CORAL_WRIST_PDH_CHANNEL);

        // Calculates power, velocity averages
        this.intakePowerAverage = this.intakePowerAverageCalculator.update(intakeCurrent * batteryVoltage);
        this.wristPowerAverage = this.wristPowerAverageCalculator.update(wristCurrent * batteryVoltage);

        this.logger.logNumber(LoggingKey.CoralEndEffectorIntakePowerAverage, this.intakePowerAverage);
        this.logger.logNumber(LoggingKey.CoralEndEffectorWristPowerAverage, this.wristPowerAverage);

        this.wristVelocityAverage = this.wristVelocityAverageCalculator.update(this.wristVelocity);
        this.logger.logNumber(LoggingKey.CoralEndEffectorWristVelocityAverage, this.wristVelocityAverage);
    }

    @Override
    public void update(RobotMode mode)
    {
        double currTime = this.timer.get();
        double elapsedTime = currTime - this.prevTime;

        double intakePower;
        if (this.driver.getDigital(DigitalOperation.CoralIntake))
        {
            intakePower = TuningConstants.CORAL_INTAKE_POWER;
        }
        else if (this.driver.getDigital(DigitalOperation.CoralIntakeSlow))
        {
            intakePower = TuningConstants.CORAL_INTAKE_SLOW_POWER;
        }
        else if (this.driver.getDigital(DigitalOperation.CoralReverse))
        {
            intakePower = TuningConstants.CORAL_REVERSE_POWER;
        }
        else if (this.driver.getDigital(DigitalOperation.CoralReverseSlow))
        {
            intakePower = TuningConstants.CORAL_REVERSE_SLOW_POWER;
        }
        else if (this.driver.getDigital(DigitalOperation.CoralPlace))
        {
            intakePower = TuningConstants.CORAL_PLACE_POWER;
        }
        else if (this.driver.getDigital(DigitalOperation.CoralPlaceSlow))
        {
            intakePower = TuningConstants.CORAL_PLACE_SLOW_POWER;
        }
        else
        {
            intakePower = 0.0;
        }

        this.intakeMotor.set(intakePower);
        this.logger.logNumber(LoggingKey.CoralEndEffectorIntakeOutput, intakePower);

        int desiredSlot;
        if (this.throughBeamBroken)
        {
            desiredSlot = TuningConstants.CORAL_WITH_PID_SLOT;
        }
        else
        {
            desiredSlot = TuningConstants.CORAL_WITHOUT_PID_SLOT;
        }

        this.wristMotor.setSelectedSlot(desiredSlot);

        if (this.driver.getDigital(DigitalOperation.CoralWristEnableSimpleMode))
        {
            this.inSimpleMode = true;
        }
        else if (this.driver.getDigital(DigitalOperation.CoralWristDisableSimpleMode))
        {
            this.inSimpleMode = true;
            this.wristDesiredAngle = Helpers.enforceAbsoluteAngleRange(this.wristAngle, TuningConstants.CORAL_WRIST_ANGLE_RANGE_START, TuningConstants.CORAL_WRIST_ANGLE_RANGE_END);
        }

        boolean useSimpleMode = this.inSimpleMode;

        double wristPower = this.driver.getAnalog(AnalogOperation.CoralWristPowerA) + this.driver.getAnalog(AnalogOperation.CoralWristPowerB);
        if (this.inSimpleMode)
        {
            if (wristPower == 0.0)
            {
                // fall back to using angle adjustment control when in simple mode...
                wristPower = this.driver.getAnalog(AnalogOperation.CoralWristAdjustAngleLeft) + this.driver.getAnalog(AnalogOperation.CoralWristAdjustAngleRight);
            }

            this.wristDesiredAngleChangedTime = currTime;
            this.isWristStalled = false;
        }
        else
        {
            if (wristPower != 0.0)
            {
                useSimpleMode = true;

                this.wristDesiredAngle = Helpers.enforceAbsoluteAngleRange(this.wristAngle, TuningConstants.CORAL_WRIST_ANGLE_RANGE_START, TuningConstants.CORAL_WRIST_ANGLE_RANGE_END);

                this.wristDesiredAngleChangedTime = currTime;
                this.isWristStalled = false;
            }
            else
            {
                double coralWristAngleAdjustment = this.driver.getAnalog(AnalogOperation.CoralWristAdjustAngleLeft) + this.driver.getAnalog(AnalogOperation.CoralWristAdjustAngleRight);;
                double newDesiredWristAngle = this.driver.getAnalog(AnalogOperation.CoralWristDesiredAngle);

                if (newDesiredWristAngle != TuningConstants.MAGIC_NULL_VALUE)
                {
                    this.wristDesiredAngle = newDesiredWristAngle;

                    this.wristDesiredAngleChangedTime = currTime;
                    this.isWristStalled = false;
                }
                else if (coralWristAngleAdjustment != 0.0)
                {
                    double adjustmentAmount = coralWristAngleAdjustment * TuningConstants.CORAL_WRIST_ANGLE_ADJUSTMENT_VELOCITY * elapsedTime;
                    this.wristDesiredAngle = Helpers.updateAngleRange360(this.wristDesiredAngle + adjustmentAmount);

                    this.wristDesiredAngleChangedTime = currTime;
                    this.isWristStalled = false;
                }

                if (newDesiredWristAngle != TuningConstants.MAGIC_NULL_VALUE || coralWristAngleAdjustment != 0.0)
                {
                    // Clamp the coralWristMotorDesiredAngle to min and max angles using the Helpers.EnforceRange() function
                    double clampedDesiredElevatorPosition = Helpers.enforceAbsoluteAngleRange(this.wristDesiredAngle, TuningConstants.CORAL_WRIST_ANGLE_RANGE_START, TuningConstants.CORAL_WRIST_ANGLE_RANGE_END);
                    if (clampedDesiredElevatorPosition != this.wristDesiredAngle)
                    {
                        // System.out.println("clamped");
                    }

                    this.wristDesiredAngle = clampedDesiredElevatorPosition;
                }
            }
        }

        this.logger.logNumber(LoggingKey.CoralEndEffectorWristDesiredAngle, this.wristDesiredAngle);

        if (TuningConstants.CORAL_USE_WRIST_STALL_MODE)
        {
            if (currTime >= this.wristDesiredAngleChangedTime + TuningConstants.CORAL_VELOCITY_TRACKING_DURATION &&
                this.wristPowerAverage >= TuningConstants.CORAL_WRIST_STALLED_POWER_THRESHOLD &&
                Math.abs(this.wristVelocityAverage) <= TuningConstants.CORAL_WRIST_STALLED_VELOCITY_THRESHOLD)
            {
                this.isWristStalled = true;
            }
        }

        // Actually setting the motor power based on whether simple mode or not
        double wristOutput;
        if (this.isWristStalled)
        {
            wristOutput = 0.0;
            this.wristMotor.stop();
        }
        else if (useSimpleMode)
        {
            wristOutput = wristPower;
            this.wristMotor.set(SparkControlMode.PercentOutput, wristPower);
        }
        else
        {
            ExceptionHelpers.Assert(
                Helpers.withinAbsoluteAngleRange(this.wristDesiredAngle, TuningConstants.CORAL_WRIST_ANGLE_RANGE_START, TuningConstants.CORAL_WRIST_ANGLE_RANGE_END),
                "Expect our setpoint (%f) to be within the desired angle range (%f, %f)",
                this.wristDesiredAngle,
                TuningConstants.CORAL_WRIST_ANGLE_RANGE_START,
                TuningConstants.CORAL_WRIST_ANGLE_RANGE_END);

            this.wristMotor.set(
                TuningConstants.CORAL_WRIST_USE_TMP ? SparkControlMode.MAXMotionPosition : SparkControlMode.Position,
                this.wristDesiredAngle);

            wristOutput = this.wristMotor.getOutput();
        }

        this.logger.logNumber(LoggingKey.CoralEndEffectorWristOutput, wristOutput);

        // Reset prevTime at end of update() loop
        this.prevTime = currTime;
    }

    @Override
    public void stop()
    {
        this.wristMotor.stop();
        this.intakeMotor.stop();
    }

    public boolean hasGamePiece()
    {
        return this.throughBeamBroken;
    }

    public double getAngle()
    {
        return this.wristAngle;
    }
}

