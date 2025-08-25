package frc.robot.mechanisms;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.lib.driver.IDriver;
import frc.lib.filters.FloatingAverageCalculator;
import frc.lib.helpers.ExceptionHelpers;
import frc.lib.helpers.Helpers;
import frc.lib.mechanisms.IMechanism;
import frc.lib.mechanisms.LoggingManager;
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
public class AlgaeIntakeMechanism implements IMechanism
{
    private final IDriver driver;
    private final ITimer timer;

    private final LoggingManager logger;
    private final PowerManager powerManager;

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

    private boolean hasAlgae;
    private double intakeStartedTime;

    private boolean inSimpleMode;

    private boolean isWristStalled;
    private double wristDesiredAngleChangedTime;

    private double prevTime;

    @Inject
    public AlgaeIntakeMechanism(
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
        this.intakeMotor = provider.getTalonSRX(ElectronicsConstants.ALGAE_INTAKE_MOTOR_CAN_ID);
        this.intakeMotor.setMotorOutputSettings(TuningConstants.ALGAE_INTAKE_MOTOR_INVERT_OUTPUT, MotorNeutralMode.Brake);
        this.intakeMotor.setControlMode(TalonSRXControlMode.PercentOutput);

        this.wristMotor = provider.getSparkMax(ElectronicsConstants.ALGAE_WRIST_MOTOR_CAN_ID, SparkMotorType.Brushed);
        this.wristMotor.configureMotorOutputSettings(TuningConstants.ALGAE_WRIST_MOTOR_INVERT_OUTPUT, MotorNeutralMode.Brake);
        this.wristMotor.configureAbsoluteEncoder();
        this.wristMotor.configureAbsoluteOffset(TuningConstants.ALGAE_WRIST_ABSOLUTE_ENCODER_OFFSET);
        this.wristMotor.configureInvertSensor(TuningConstants.ALGAE_WRIST_MOTOR_INVERT_SENSOR);
        this.wristMotor.configurePositionConversionFactor(HardwareConstants.ALGAE_INTAKE_MOTOR_TICK_DISTANCE);
        this.wristMotor.configureVelocityConversionFactor(HardwareConstants.ALGAE_INTAKE_MOTOR_TICK_DISTANCE);
        this.wristMotor.configureSignals(SparkSignalType.AbsoluteEncoderPosition, 10);
        this.wristMotor.configureSignals(SparkSignalType.AbsoluteEncoderVelocity, 10);

        if (TuningConstants.ALGAE_WRIST_USE_PID)
        {
            if (TuningConstants.ALGAE_WRIST_USE_TMP)
            {
                this.wristMotor.configurePIDFMAXMotion(
                    TuningConstants.ALGAE_WITHOUT_TMP_WRIST_KP,
                    TuningConstants.ALGAE_WITHOUT_TMP_WRIST_KI,
                    TuningConstants.ALGAE_WITHOUT_TMP_WRIST_KD,
                    TuningConstants.ALGAE_WITHOUT_TMP_WRIST_KF,
                    0.0,
                    TuningConstants.ALGAE_WITHOUT_TMP_WRIST_CRUISE_VELOCITY,
                    TuningConstants.ALGAE_WITHOUT_TMP_WRIST_MAX_ACCELERATION,
                    TuningConstants.ALGAE_WRIST_MIN_OUTPUT,
                    TuningConstants.ALGAE_WRIST_MAX_OUTPUT,
                    TuningConstants.ALGAE_WITHOUT_PID_SLOT);

                this.wristMotor.configurePIDFMAXMotion(
                    TuningConstants.ALGAE_WITH_TMP_WRIST_KP,
                    TuningConstants.ALGAE_WITH_TMP_WRIST_KI,
                    TuningConstants.ALGAE_WITH_TMP_WRIST_KD,
                    TuningConstants.ALGAE_WITH_TMP_WRIST_KF,
                    0.0,
                    TuningConstants.ALGAE_WITH_TMP_WRIST_CRUISE_VELOCITY,
                    TuningConstants.ALGAE_WITH_TMP_WRIST_MAX_ACCELERATION,
                    TuningConstants.ALGAE_WRIST_MIN_OUTPUT,
                    TuningConstants.ALGAE_WRIST_MAX_OUTPUT,
                    TuningConstants.ALGAE_WITH_PID_SLOT);

                this.wristMotor.setControlMode(SparkControlMode.MAXMotionPosition);
            }
            else
            {
                this.wristMotor.configurePIDF(
                    TuningConstants.ALGAE_WITHOUT_PID_WRIST_KP,
                    TuningConstants.ALGAE_WITHOUT_PID_WRIST_KI,
                    TuningConstants.ALGAE_WITHOUT_PID_WRIST_KD,
                    TuningConstants.ALGAE_WITHOUT_PID_WRIST_KF,
                    TuningConstants.ALGAE_WRIST_MIN_OUTPUT,
                    TuningConstants.ALGAE_WRIST_MAX_OUTPUT,
                    TuningConstants.ALGAE_WITHOUT_PID_SLOT);

                this.wristMotor.configurePIDF(
                    TuningConstants.ALGAE_WITH_PID_WRIST_KP,
                    TuningConstants.ALGAE_WITH_PID_WRIST_KI,
                    TuningConstants.ALGAE_WITH_PID_WRIST_KD,
                    TuningConstants.ALGAE_WITH_PID_WRIST_KF,
                    TuningConstants.ALGAE_WRIST_MIN_OUTPUT,
                    TuningConstants.ALGAE_WRIST_MAX_OUTPUT,
                    TuningConstants.ALGAE_WITH_PID_SLOT);

                this.wristMotor.setControlMode(SparkControlMode.Position);
            }

            this.wristMotor.configurePositionPIDWrappingSettings(
                TuningConstants.ALGAE_WRIST_POSITION_PID_WRAPPING_ENABLED,
                TuningConstants.ALGAE_WRIST_POSITION_PID_WRAPPING_MIN,
                TuningConstants.ALGAE_WRIST_POSITION_PID_WRAPPING_MAX);
            this.inSimpleMode = false;
        }
        else
        {
            this.wristMotor.setControlMode(SparkControlMode.PercentOutput);
            this.inSimpleMode = true;
        }

        this.wristMotor.setSelectedSlot(TuningConstants.ALGAE_WITH_PID_SLOT);
        this.wristMotor.applyConfiguration(true);

        this.wristAngle = this.wristMotor.getPosition(); 
        this.wristDesiredAngle = TuningConstants.ALGAE_WRIST_STOW_ANGLE;
        this.wristVelocity = 0.0;

        this.hasAlgae = false;
        this.intakeStartedTime = 0.0;

        // Stall
        this.wristDesiredAngleChangedTime = TuningConstants.MAGIC_NULL_VALUE;
        this.isWristStalled = false;

        this.intakePowerAverageCalculator = new FloatingAverageCalculator(this.timer, TuningConstants.ALGAE_INTAKE_MIN_POWER_VALUE, TuningConstants.ALGAE_INTAKE_MAX_POWER_VALUE, TuningConstants.ALGAE_INTAKE_POWER_TRACKING_DURATION, TuningConstants.ALGAE_POWER_SAMPLES_PER_SECOND);
        this.wristPowerAverageCalculator = new FloatingAverageCalculator(this.timer, TuningConstants.ALGAE_WRIST_MIN_POWER_VALUE, TuningConstants.ALGAE_WRIST_MAX_POWER_VALUE, TuningConstants.ALGAE_WRIST_POWER_TRACKING_DURATION, TuningConstants.ALGAE_POWER_SAMPLES_PER_SECOND);
        this.wristVelocityAverageCalculator = new FloatingAverageCalculator(this.timer, TuningConstants.ALGAE_VELOCITY_TRACKING_DURATION, TuningConstants.ALGAE_VELOCITY_SAMPLES_PER_SECOND);

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

        this.logger.logNumber(LoggingKey.AlgaeEndEffectorWristAngle, this.wristAngle);
        this.logger.logNumber(LoggingKey.AlgaeEndEffectorWristVelocity, this.wristVelocity);
        this.logger.logNumber(LoggingKey.AlgaeEndEffectorWristVelocityAverage, this.wristVelocityAverage);

        // power information
        double batteryVoltage = this.powerManager.getBatteryVoltage();
        double intakeCurrent = this.powerManager.getCurrent(ElectronicsConstants.ALGAE_INTAKE_PDH_CHANNEL);
        double wristCurrent = this.powerManager.getCurrent(ElectronicsConstants.ALGAE_WRIST_PDH_CHANNEL);

        // Calculates power, velocity averages
        this.intakePowerAverage = this.intakePowerAverageCalculator.update(intakeCurrent * batteryVoltage);
        this.wristPowerAverage = this.wristPowerAverageCalculator.update(wristCurrent * batteryVoltage);

        this.logger.logNumber(LoggingKey.AlgaeEndEffectorIntakePowerAverage, this.intakePowerAverage);
        this.logger.logNumber(LoggingKey.AlgaeEndEffectorWristPowerAverage, this.wristPowerAverage);

        this.wristVelocityAverage = this.wristVelocityAverageCalculator.update(this.wristVelocity);
        this.logger.logNumber(LoggingKey.AlgaeEndEffectorWristVelocityAverage, this.wristVelocityAverage);
    }

    @Override
    public void update(RobotMode mode)
    {
        double currTime = this.timer.get();
        double elapsedTime = currTime - this.prevTime;
        double intakePower = 0.0;

        if (this.driver.getDigital(DigitalOperation.AlgaeIntake))
        {
            intakePower = TuningConstants.ALGAE_INTAKE_POWER;
            if (this.intakeStartedTime == TuningConstants.MAGIC_NULL_VALUE)
            {
                this.intakeStartedTime = currTime;
            }
            else if (this.intakeStartedTime + TuningConstants.ALGAE_INTAKE_POWER_TRACKING_DURATION <= currTime)
            {
                this.hasAlgae = this.intakePowerAverage > TuningConstants.ALGAE_INTAKE_STALLED_POWER_THRESHOLD;
            }
        }
        else if (this.driver.getDigital(DigitalOperation.AlgaeOuttake))
        {
            intakePower = TuningConstants.ALGAE_OUTTAKE_POWER;
            this.intakeStartedTime = TuningConstants.MAGIC_NULL_VALUE;
            this.hasAlgae = false;
        }
        else if (this.driver.getDigital(DigitalOperation.AlgaeLaunch))
        {
            intakePower = TuningConstants.ALGAE_LAUNCH_POWER;
            this.intakeStartedTime = TuningConstants.MAGIC_NULL_VALUE;
            this.hasAlgae = false;
        }
        else
        {
            if (this.hasAlgae &&
                this.intakePowerAverage < TuningConstants.ALGAE_INTAKE_NO_LONGER_STALLED_POWER_THRESHOLD)
            {
                this.hasAlgae = false;
            }

            if (TuningConstants.USE_ALGAE_HOLD_POWER && this.hasAlgae)
            {
                intakePower = TuningConstants.ALGAE_HOLD_POWER;
            }
            else
            {
                intakePower = 0.0;
            }

            this.intakeStartedTime = TuningConstants.MAGIC_NULL_VALUE;
        }

        this.logger.logBoolean(LoggingKey.AlgaeEndEffectorHasAlgae, this.hasAlgae);

        this.intakeMotor.set(intakePower);
        this.logger.logNumber(LoggingKey.AlgaeEndEffectorIntakeOutput, intakePower);

        int desiredSlot;
        if (this.hasAlgae)
        {
            desiredSlot = TuningConstants.ALGAE_WITH_PID_SLOT;
        }
        else
        {
            desiredSlot = TuningConstants.ALGAE_WITHOUT_PID_SLOT;
        }

        this.wristMotor.setSelectedSlot(desiredSlot);

        if (this.driver.getDigital(DigitalOperation.AlgaeWristEnableSimpleMode))
        {
            this.inSimpleMode = true;
        }
        else if (this.driver.getDigital(DigitalOperation.AlgaeWristDisableSimpleMode))
        {
            this.inSimpleMode = false;
        }

        boolean useSimpleMode = this.inSimpleMode;

        double wristPower = this.driver.getAnalog(AnalogOperation.AlgaeWristPower);
        if (this.inSimpleMode)
        {
            if (wristPower == 0.0)
            {
                // fall back to using angle adjustment control when in simple mode...
                wristPower = this.driver.getAnalog(AnalogOperation.AlgaeWristAngleAdjustment);
            }

            this.wristDesiredAngleChangedTime = currTime;
            this.isWristStalled = false;
        }
        else
        {
            if (wristPower != 0.0)
            {
                useSimpleMode = true;

                this.wristDesiredAngleChangedTime = currTime;
                this.isWristStalled = false;
            }
            else
            {
                double algaeWristAngleAdjustment = this.driver.getAnalog(AnalogOperation.AlgaeWristAngleAdjustment);
                double newDesiredWristAngle = this.driver.getAnalog(AnalogOperation.AlgaeWristDesiredAngle);

                if (newDesiredWristAngle != TuningConstants.MAGIC_NULL_VALUE)
                {
                    this.wristDesiredAngle = newDesiredWristAngle;

                    this.wristDesiredAngleChangedTime = currTime;
                    this.isWristStalled = false;
                }
                else if (algaeWristAngleAdjustment != 0.0)
                {
                    double angleChange = algaeWristAngleAdjustment * TuningConstants.ALGAE_WRIST_ANGLE_ADJUSTMENT_VELOCITY * elapsedTime;
                    this.wristDesiredAngle = Helpers.updateAngleRange360(this.wristDesiredAngle + angleChange);

                    this.wristDesiredAngleChangedTime = currTime;
                    this.isWristStalled = false;
                }

                if (newDesiredWristAngle != TuningConstants.MAGIC_NULL_VALUE || algaeWristAngleAdjustment != 0.0)
                {
                    // Clamp the algaeWristMotorDesiredAngle to min and max angles using the Helpers.EnforceRange() function
                    double clampedDesiredElevatorPosition = Helpers.enforceAbsoluteAngleRange(this.wristDesiredAngle, TuningConstants.ALGAE_WRIST_ANGLE_RANGE_START, TuningConstants.ALGAE_WRIST_ANGLE_RANGE_END);
                    this.wristDesiredAngle = clampedDesiredElevatorPosition;
                }
            }
        }

        this.logger.logNumber(LoggingKey.AlgaeEndEffectorWristDesiredAngle, this.wristDesiredAngle);

        if (TuningConstants.ALGAE_USE_WRIST_STALL_MODE)
        {
            if (currTime >= this.wristDesiredAngleChangedTime + TuningConstants.ALGAE_VELOCITY_TRACKING_DURATION &&
                this.wristPowerAverage >= TuningConstants.ALGAE_WRIST_STALLED_POWER_THRESHOLD &&
                Math.abs(this.wristVelocityAverage) <= TuningConstants.ALGAE_WRIST_STALLED_VELOCITY_THRESHOLD)
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
                Helpers.withinAbsoluteAngleRange(this.wristDesiredAngle, TuningConstants.ALGAE_WRIST_ANGLE_RANGE_START, TuningConstants.ALGAE_WRIST_ANGLE_RANGE_END),
                "Expect our setpoint (%f) to be within the desired angle range (%f, %f)",
                this.wristDesiredAngle,
                TuningConstants.ALGAE_WRIST_ANGLE_RANGE_START,
                TuningConstants.ALGAE_WRIST_ANGLE_RANGE_END);

            this.wristMotor.set(
                TuningConstants.ALGAE_WRIST_USE_TMP ? SparkControlMode.MAXMotionPosition : SparkControlMode.Position,
                this.wristDesiredAngle);

            wristOutput = this.wristMotor.getOutput();
        }

        this.logger.logNumber(LoggingKey.AlgaeEndEffectorWristOutput, wristOutput);

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
        return this.hasAlgae;
    }

    public double getAngle()
    {
        return this.wristAngle;
    }
}

