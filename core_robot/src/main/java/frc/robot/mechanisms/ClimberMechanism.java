package frc.robot.mechanisms;

import frc.lib.controllers.PIDHandler;
import frc.lib.driver.IDriver;
import frc.lib.filters.FloatingAverageCalculator;
import frc.lib.helpers.ExceptionHelpers;
import frc.lib.helpers.Helpers;
import frc.lib.mechanisms.IMechanism;
import frc.lib.mechanisms.LoggingManager;
import frc.lib.robotprovider.*;
import frc.robot.ElectronicsConstants;
import frc.robot.HardwareConstants;
import frc.robot.TuningConstants;
import frc.robot.LoggingKey;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class ClimberMechanism implements IMechanism
{
    private final IDriver driver;
    private final ILogger logger;
    private final ITimer timer;

    private final PowerManager powerManager;

    private final ISparkMax winchMotor;
    private final ISparkMax elbowMotor;

    private final IDigitalInput limitSwitch;

    private final PIDHandler elbowPidHandler;
    private final IDutyCycleEncoder elbowEncoder;

    private Double elbowAngle;
    private Double elbowVelocity;

    private boolean isLimitSwitchPressed;

    private boolean inSimpleMode;
    private double elbowDesiredAngle;

    private boolean isElbowStalled;
    private double elbowDesiredAngleChangedTime;

    private boolean hasSwitchedToBrakeMode;

    private FloatingAverageCalculator elbowVelocityAverageCalculator;
    private FloatingAverageCalculator elbowPowerAverageCalculator;
    private Double elbowVelocityAverage;
    private double elbowPowerAverage;

    private double prevTime;

    @Inject
    public ClimberMechanism(IRobotProvider provider, IDriver driver, LoggingManager logger, ITimer timer, PowerManager powerManager)
    {
        this.driver = driver;
        this.logger = logger;
        this.timer = timer;

        this.powerManager = powerManager;

        this.winchMotor = provider.getSparkMax(ElectronicsConstants.CLIMBER_WINCH_MOTOR_CAN_ID, SparkMotorType.Brushless);
        this.winchMotor.configureMotorOutputSettings(TuningConstants.CLIMBER_WINCH_INVERTED, MotorNeutralMode.Coast);
        this.winchMotor.applyConfiguration(true);
        this.winchMotor.setControlMode(SparkControlMode.PercentOutput);

        this.elbowMotor = provider.getSparkMax(ElectronicsConstants.CLIMBER_ELBOW_MOTOR_CAN_ID, SparkMotorType.Brushed);
        this.elbowMotor.configureMotorOutputSettings(TuningConstants.CLIMBER_ELBOW_MOTOR_INVERTED, MotorNeutralMode.Brake);
        if (ElectronicsConstants.CLIMBER_ABSOLUTE_ENCODER_ROUTED_TO_ROBORIO)
        {
            this.elbowMotor.setControlMode(SparkControlMode.PercentOutput);

            this.elbowEncoder = provider.getDutyCycleEncoder(ElectronicsConstants.CLIMBER_ELBOW_ENCODER_DIO_CHANNEL);
            this.elbowEncoder.setDutyCycleRange(ElectronicsConstants.REV_THROUGHBORE_ENCODER_DUTY_CYCLE_MIN, ElectronicsConstants.REV_THROUGHBORE_ENCODER_DUTY_CYCLE_MAX);
            this.elbowEncoder.setDistancePerRotation(HardwareConstants.CLIMBER_ELBOW_ABSOLUTE_ENCODER_TICK_DISTANCE);
            this.elbowEncoder.setPositionOffset(TuningConstants.CLIMBER_ELBOW_ABSOLUTE_ENCODER_OFFSET);
            this.elbowEncoder.setInverted(TuningConstants.CLIMBER_ELBOW_ENCODER_INVERTED);

            this.elbowPidHandler =
                new PIDHandler(
                    TuningConstants.CLIMBER_ELBOW_ROBORIO_PID_KP,
                    TuningConstants.CLIMBER_ELBOW_ROBORIO_PID_KI,
                    TuningConstants.CLIMBER_ELBOW_ROBORIO_PID_KD,
                    TuningConstants.CLIMBER_ELBOW_ROBORIO_PID_KF,
                    TuningConstants.CLIMBER_ELBOW_ROBORIO_PID_KS,
                    TuningConstants.CLIMBER_ELBOW_ROBORIO_PID_MIN_VALUE,
                    TuningConstants.CLIMBER_ELBOW_ROBORIO_PID_MAX_VALUE,
                    this.timer);
        }
        else
        {
            this.elbowMotor.configureAbsoluteEncoder();
            this.elbowMotor.configureAbsoluteOffset(TuningConstants.CLIMBER_ELBOW_ABSOLUTE_ENCODER_OFFSET);
            this.elbowMotor.configureInvertSensor(TuningConstants.CLIMBER_ELBOW_ENCODER_INVERTED);
            this.elbowMotor.configurePositionConversionFactor(HardwareConstants.CLIMBER_ELBOW_ABSOLUTE_ENCODER_TICK_DISTANCE);
            this.elbowMotor.configureSignals(SparkSignalType.AbsoluteEncoderPosition, 10);
            this.elbowMotor.configureSignals(SparkSignalType.AbsoluteEncoderVelocity, 10);
            this.elbowMotor.configurePIDF(
                TuningConstants.CLIMBER_ELBOW_SPARKMAX_PID_KP,
                TuningConstants.CLIMBER_ELBOW_SPARKMAX_PID_KI,
                TuningConstants.CLIMBER_ELBOW_SPARKMAX_PID_KD,
                TuningConstants.CLIMBER_ELBOW_SPARKMAX_PID_KF,
                TuningConstants.CLIMBER_ELBOW_SPARKMAX_PID_MIN_VALUE,
                TuningConstants.CLIMBER_ELBOW_SPARKMAX_PID_MAX_VALUE,
                0);

            this.elbowMotor.setControlMode(SparkControlMode.Position);
            this.elbowEncoder = null;
            this.elbowPidHandler = null;
        }

        this.elbowMotor.applyConfiguration(true);

        this.limitSwitch = provider.getDigitalInput(ElectronicsConstants.CLIMBER_DOWN_LIMIT_SWITCH_DIO_CHANNEL);

        this.elbowAngle = this.elbowEncoder.getAbsolutePosition();
        this.elbowVelocity = null;
        this.elbowDesiredAngle = TuningConstants.CLIMBER_ELBOW_INITIAL_ANGLE;

        this.isLimitSwitchPressed = false;
        if (TuningConstants.CLIMBER_USE_LIMIT_SWITCH)
        {
            this.isLimitSwitchPressed = this.limitSwitch.get();
        }

        this.elbowPowerAverageCalculator = new FloatingAverageCalculator(this.timer, TuningConstants.CLIMBER_ELBOW_MIN_POWER_VALUE, TuningConstants.CLIMBER_ELBOW_MAX_POWER_VALUE, TuningConstants.CLIMBER_ELBOW_POWER_TRACKING_DURATION, TuningConstants.CLIMBER_ELBOW_POWER_SAMPLES_PER_SECOND);
        this.elbowVelocityAverageCalculator = new FloatingAverageCalculator(this.timer, TuningConstants.CLIMBER_ELBOW_VELOCITY_TRACKING_DURATION, TuningConstants.CLIMBER_ELBOW_VELOCITY_SAMPLES_PER_SECOND);
        this.elbowVelocityAverage = 0.0;
        this.elbowPowerAverage = 0.0;

        this.inSimpleMode = !TuningConstants.CLIMBER_USE_PID;

        this.hasSwitchedToBrakeMode = false;
    }

    @Override
    public void readSensors()
    {
        boolean limitSwitchPressed = this.limitSwitch.get();

        this.logger.logBoolean(LoggingKey.ClimberElbowLimitSwitch, limitSwitchPressed);
        if (TuningConstants.CLIMBER_USE_LIMIT_SWITCH)
        {
            this.isLimitSwitchPressed = limitSwitchPressed;
        }

        if (ElectronicsConstants.CLIMBER_ABSOLUTE_ENCODER_ROUTED_TO_ROBORIO)
        {
            this.elbowAngle = this.elbowEncoder.getAbsolutePosition();
        }
        else
        {
            this.elbowAngle = this.elbowMotor.getPosition();
            this.elbowVelocity = this.elbowMotor.getVelocity();
            this.elbowVelocityAverage = this.elbowVelocityAverageCalculator.update(this.elbowVelocity);
        }

        this.logger.logNumber(LoggingKey.ClimberElbowAngle, this.elbowAngle);
        this.logger.logNumber(LoggingKey.ClimberElbowVelocity, this.elbowVelocity);
        this.logger.logNumber(LoggingKey.ClimberElbowVelocityAverage, this.elbowVelocityAverage);

        double batteryVoltage = this.powerManager.getBatteryVoltage();
        double elbowCurrent = this.powerManager.getCurrent(ElectronicsConstants.CLIMBER_ELBOW_PDH_CHANNEL);
        this.elbowPowerAverage = this.elbowPowerAverageCalculator.update(batteryVoltage * elbowCurrent);
        this.logger.logNumber(LoggingKey.ClimberElbowPowerAverage, this.elbowPowerAverage);
    }

    @Override
    public void update(RobotMode mode)
    {
        double currTime = this.timer.get();
        double elapsedTime = currTime - this.prevTime;

        if (this.driver.getDigital(DigitalOperation.ClimberElbowEnableSimpleMode) ||
            this.elbowAngle == null)
        {
            this.inSimpleMode = true;
        }
        else if (this.driver.getDigital(DigitalOperation.ClimberElbowDisableSimpleMode))
        {
            this.inSimpleMode = false;
            this.elbowDesiredAngle = Helpers.enforceAbsoluteAngleRange(this.elbowAngle, TuningConstants.CLIMBER_ELBOW_ANGLE_RANGE_START, TuningConstants.CLIMBER_ELBOW_ANGLE_RANGE_END);
        }

        boolean useSimpleMode = this.inSimpleMode;

        double elbowPower = this.driver.getAnalog(AnalogOperation.ClimberElbowPower);
        if (this.inSimpleMode)
        {
            if (elbowPower == 0.0)
            {
                // fall back to using angle adjustment control when in simple mode...
                elbowPower = this.driver.getAnalog(AnalogOperation.ClimberElbowAngleAdjustment);
            }

            this.elbowDesiredAngleChangedTime = currTime;
            this.isElbowStalled = false;
        }
        else
        {
            if (elbowPower != 0.0)
            {
                useSimpleMode = true;

                this.elbowDesiredAngle = Helpers.enforceAbsoluteAngleRange(this.elbowAngle, TuningConstants.CLIMBER_ELBOW_ANGLE_RANGE_START, TuningConstants.CLIMBER_ELBOW_ANGLE_RANGE_END);

                this.elbowDesiredAngleChangedTime = currTime;
                this.isElbowStalled = false;
            }
            else
            {
                double elbowAngleAdjustment = this.driver.getAnalog(AnalogOperation.ClimberElbowAngleAdjustment);
                double newDesiredElbowAngle = this.driver.getAnalog(AnalogOperation.ClimberElbowDesiredAngle);

                if (newDesiredElbowAngle != TuningConstants.MAGIC_NULL_VALUE)
                {
                    this.elbowDesiredAngle = newDesiredElbowAngle;

                    this.elbowDesiredAngleChangedTime = currTime;
                    this.isElbowStalled = false;
                }
                else if (elbowAngleAdjustment != 0.0)
                {
                    this.elbowDesiredAngle = Helpers.updateAngleRange360(this.elbowDesiredAngle + elbowAngleAdjustment * TuningConstants.CLIMBER_ELBOW_ANGLE_ADJUSTMENT_VELOCITY * elapsedTime);

                    this.elbowDesiredAngleChangedTime = currTime;
                    this.isElbowStalled = false;
                }

                if (newDesiredElbowAngle != TuningConstants.MAGIC_NULL_VALUE || elbowAngleAdjustment != 0.0)
                {
                    // Clamp the coralWristMotorDesiredAngle to min and max angles using the Helpers.EnforceRange() function
                    double clampedDesiredElevatorPosition = Helpers.enforceAbsoluteAngleRange(this.elbowDesiredAngle, TuningConstants.CLIMBER_ELBOW_ANGLE_RANGE_START, TuningConstants.CLIMBER_ELBOW_ANGLE_RANGE_END);
                    this.elbowDesiredAngle = clampedDesiredElevatorPosition;
                }
            }
        }

        this.logger.logNumber(LoggingKey.ClimberElbowDesiredAngle, this.elbowDesiredAngle);

        if (TuningConstants.CLIMBER_ELBOW_USE_STALL_MODE)
        {
            if (currTime >= this.elbowDesiredAngleChangedTime + TuningConstants.CLIMBER_ELBOW_VELOCITY_TRACKING_DURATION &&
                this.elbowPowerAverage >= TuningConstants.CLIMBER_ELBOW_STALLED_POWER_THRESHOLD &&
                Math.abs(this.elbowVelocityAverage) <= TuningConstants.CLIMBER_ELBOW_STALLED_VELOCITY_THRESHOLD)
            {
                this.isElbowStalled = true;
            }
        }

        this.logger.logBoolean(LoggingKey.ClimberElbowStalled, this.isElbowStalled);

        // Actually setting the motor power based on whether simple mode or not
        double elbowOutput;
        if (this.isElbowStalled)
        {
            elbowOutput = 0.0;
            this.elbowMotor.stop();
        }
        else if (useSimpleMode)
        {
            elbowOutput = elbowPower;
            this.elbowMotor.set(SparkControlMode.PercentOutput, elbowPower);
        }
        else
        {
            ExceptionHelpers.Assert(
                Helpers.withinAbsoluteAngleRange(this.elbowDesiredAngle, TuningConstants.CLIMBER_ELBOW_ANGLE_RANGE_START, TuningConstants.CLIMBER_ELBOW_ANGLE_RANGE_END),
                "Expect our setpoint (%f) to be within the desired angle range (%f, %f)",
                this.elbowDesiredAngle,
                TuningConstants.CLIMBER_ELBOW_ANGLE_RANGE_START,
                TuningConstants.CLIMBER_ELBOW_ANGLE_RANGE_END);

            if (ElectronicsConstants.CLIMBER_ABSOLUTE_ENCODER_ROUTED_TO_ROBORIO)
            {
                elbowPower = this.elbowPidHandler.calculatePosition(this.elbowDesiredAngle, this.elbowAngle);
                this.elbowMotor.set(elbowPower);
                elbowOutput = elbowPower;
            }
            else
            {
                this.elbowMotor.set(SparkControlMode.Position, this.elbowDesiredAngle);
                elbowOutput = this.elbowMotor.getOutput();
            }
        }

        this.logger.logNumber(LoggingKey.ClimberElbowOutput, elbowOutput);

        // winch
        double winchPowerAdjustment = TuningConstants.ZERO;
        if (this.driver.getDigital(DigitalOperation.ClimberWinchClimb) &&
            !this.isLimitSwitchPressed)
        {
            winchPowerAdjustment = TuningConstants.CLIMBER_WINCH_POWER;
            if (!this.hasSwitchedToBrakeMode &&
                TuningConstants.CLIMBER_WINCH_BRAKE_AFTER_WINCHING)
            {
                this.winchMotor.configureMotorOutputSettings(TuningConstants.CLIMBER_WINCH_INVERTED, MotorNeutralMode.Brake);
                this.winchMotor.applyConfiguration(false);
                this.hasSwitchedToBrakeMode = true;
            }
        }

        this.winchMotor.set(winchPowerAdjustment);
        this.logger.logNumber(LoggingKey.ClimberWinchOutput, winchPowerAdjustment);

        // Reset prevTime at end of update() loop
        this.prevTime = currTime;
    }

    @Override
    public void stop()
    {
        this.winchMotor.stop();
        this.elbowMotor.stop();
    }

    public boolean getLimitSwitchStatus()
    {
        return this.isLimitSwitchPressed;
    }

    public Double getAngle()
    {
        return this.elbowAngle;
    }
}
