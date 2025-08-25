package frc.lib.robotprovider;

import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.*;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.lib.helpers.ExceptionHelpers;

import com.revrobotics.spark.config.SparkMaxConfig;

abstract class SparkBaseWrapper implements ISparkBase
{
    final SparkBase wrappedObject;

    private final String sparkMotorId;

    private SparkMaxConfig config;
    private double zeroOffset; // recorded because we may need to update this when setting position.

    private SparkClosedLoopController closedLoopController;
    private boolean useAbsoluteEncoder;
    private RelativeEncoder wrappedRelativeEncoder;
    private AbsoluteEncoder wrappedAbsoluteEncoder;
    private SparkLimitSwitch wrappedFwdLimitSwitch;
    private SparkLimitSwitch wrappedRevLimitSwitch;

    private SparkControlMode currentMode;
    private ClosedLoopSlot selectedSlot;

    SparkBaseWrapper(SparkBase sparkBase, String sparkMotorId)
    {
        this.wrappedObject = sparkBase;
        this.sparkMotorId = sparkMotorId;

        this.config = new SparkMaxConfig();

        this.currentMode = SparkControlMode.PercentOutput;
        this.useAbsoluteEncoder = false;
    }

    public void set(double value)
    {
        this.set(this.currentMode, value, 0.0);
    }

    public void set(double value, double feedForward)
    {
        this.set(this.currentMode, value, feedForward);
    }

    public void set(SparkControlMode controlMode, double value)
    {
        this.set(controlMode, value, 0.0);
    }

    public void set(SparkControlMode controlMode, double value, double feedForward)
    {
        if (controlMode == SparkControlMode.PercentOutput)
        {
            this.wrappedObject.set(value);
            return;
        }

        this.ensureClosedLoopController();

        ControlType controlType;
        switch (controlMode)
        {
            case Position:
                controlType = ControlType.kPosition;
                break;

            case MAXMotionPosition:
                controlType = ControlType.kMAXMotionPositionControl;
                break;

            case MAXMotionVelocity:
                controlType = ControlType.kMAXMotionVelocityControl;
                break;

            case Velocity:
                controlType = ControlType.kVelocity;
                break;

            case Voltage:
                controlType = ControlType.kVoltage;
                break;

            default:
            case PercentOutput:
                throw new RuntimeException(String.format("%s: unexpected control mode %s", this.sparkMotorId, controlMode));
        }

        RevErrorCodeHelper.printError(
            this.closedLoopController.setReference(value, controlType, selectedSlot, feedForward, ArbFFUnits.kPercentOut),
            this.sparkMotorId,
            "SparkBaseWrapper.set");
    }

    public void stop()
    {
        this.wrappedObject.stopMotor();
    }

    public void setControlMode(SparkControlMode mode)
    {
        this.currentMode = mode;
    }

    public double getPosition()
    {
        if (this.useAbsoluteEncoder)
        {
            return this.wrappedAbsoluteEncoder.getPosition();
        }
        else // if (!this.useAbsoluteEncoder)
        {
            return this.wrappedRelativeEncoder.getPosition();
        }
    }

    public double getVelocity()
    {
        // NOTE: In 2024 and before, SparkMAX Absolute encoder provided velocity in Rotations per Second,
        // but SparkMAX Relative/Alternative encoder provides velocity in Rotations per Minute.
        // According to documentation, all encoder types use Rotations per Minute in 2025.
        if (this.useAbsoluteEncoder)
        {
            return this.wrappedAbsoluteEncoder.getVelocity();
        }
        else // if (!this.useAbsoluteEncoder)
        {
            return this.wrappedRelativeEncoder.getVelocity();
        }
    }

    public double getOutput()
    {
        return this.wrappedObject.getAppliedOutput();
    }

    public boolean getForwardLimitSwitchStatus()
    {
        if (this.wrappedFwdLimitSwitch == null)
        {
            return false;
        }

        return this.wrappedFwdLimitSwitch.isPressed();
    }

    public boolean getReverseLimitSwitchStatus()
    {
        if (this.wrappedRevLimitSwitch == null)
        {
            return false;
        }

        return this.wrappedRevLimitSwitch.isPressed();
    }

    public void reset()
    {
        if (!this.useAbsoluteEncoder)
        {
            this.setPosition(0.0);
        }
    }

    public void setPosition(double position)
    {
        if (this.useAbsoluteEncoder)
        {
            // update zero offset to current location
            double currentPosition = this.zeroOffset + this.wrappedAbsoluteEncoder.getPosition();
            this.config.absoluteEncoder.zeroOffset(currentPosition - position); // Note: for absolute encoders, we will need to apply configuration after a setPosition calls
        }
        else // if (!this.useAbsoluteEncoder)
        {
            RevErrorCodeHelper.printError(
                this.wrappedRelativeEncoder.setPosition(position),
                this.sparkMotorId,
                "SparkBaseWrapper.setPosition");
        }
    }

    public void applyConfiguration(boolean persistant)
    {
        this.wrappedObject.configure(this.config, ResetMode.kResetSafeParameters, persistant ? PersistMode.kPersistParameters : PersistMode.kNoPersistParameters);
    }

    public void configureAbsoluteEncoder()
    {
        this.useAbsoluteEncoder = true;
        this.wrappedAbsoluteEncoder = this.wrappedObject.getAbsoluteEncoder();
    }

    public void configureRelativeEncoder()
    {
        this.useAbsoluteEncoder = false;
        this.wrappedRelativeEncoder = this.wrappedObject.getEncoder();
    }

    public void configureRelativeEncoder(int resolution)
    {
        this.config.encoder.countsPerRevolution(resolution);
        this.useAbsoluteEncoder = false;
        this.wrappedRelativeEncoder = this.wrappedObject.getEncoder();
    }

    public void configureFollow(ISparkBase sparkBase)
    {
        if (sparkBase instanceof SparkBaseWrapper)
        {
            this.config.follow(((SparkBaseWrapper)sparkBase).wrappedObject);
        }

        if (sparkBase instanceof SparkFlexWrapper)
        {
            this.config.follow(((SparkFlexWrapper)sparkBase).wrappedObject);
        }
    }

    public void configureSignals(SparkSignalType signalType, boolean alwaysOn)
    {
        switch (signalType)
        {
            case Faults:
                this.config.signals.faultsAlwaysOn(alwaysOn);
                break;

            case Warnings:
                this.config.signals.warningsAlwaysOn(alwaysOn);
                break;

            case PrimaryEncoderVelocity:
                this.config.signals.primaryEncoderVelocityAlwaysOn(alwaysOn);
                break;

            case PrimaryEncoderPosition:
                this.config.signals.primaryEncoderPositionAlwaysOn(alwaysOn);
                break;

            case AnalogVoltage:
                this.config.signals.analogVoltageAlwaysOn(alwaysOn);
                break;

            case AnalogVelocity:
                this.config.signals.analogVelocityAlwaysOn(alwaysOn);
                break;

            case AnalogPosition:
                this.config.signals.analogPositionAlwaysOn(alwaysOn);
                break;

            case ExternalOrAltEncoderVelocity:
                this.config.signals.externalOrAltEncoderVelocityAlwaysOn(alwaysOn);
                break;

            case ExternalOrAltEncoderPosition:
                this.config.signals.externalOrAltEncoderPositionAlwaysOn(alwaysOn);
                break;

            case AbsoluteEncoderVelocity:
                this.config.signals.absoluteEncoderVelocityAlwaysOn(alwaysOn);
                break;

            case AbsoluteEncoderPosition:
                this.config.signals.absoluteEncoderPositionAlwaysOn(alwaysOn);
                break;

            case IAccumulation:
                this.config.signals.iAccumulationAlwaysOn(alwaysOn);
                break;

            default:
                ExceptionHelpers.Assert(false, "unable to confiugure alwaysOn for signal type %s", signalType);
                break;
        }
    }

    public void configureSignals(SparkSignalType signalType, int periodMs)
    {
        switch (signalType)
        {
            case AppliedOutput:
                this.config.signals.appliedOutputPeriodMs(periodMs);
                break;

            case BusVoltage:
                this.config.signals.busVoltagePeriodMs(periodMs);
                break;

            case OutputCurrent:
                this.config.signals.outputCurrentPeriodMs(periodMs);
                break;

            case MotorTemperature:
                this.config.signals.motorTemperaturePeriodMs(periodMs);
                break;

            case Limits:
                this.config.signals.limitsPeriodMs(periodMs);
                break;

            case Faults:
                this.config.signals.faultsPeriodMs(periodMs);
                break;

            case Warnings:
                this.config.signals.warningsPeriodMs(periodMs);
                break;

            case PrimaryEncoderVelocity:
                this.config.signals.primaryEncoderVelocityPeriodMs(periodMs);
                break;

            case PrimaryEncoderPosition:
                this.config.signals.primaryEncoderPositionPeriodMs(periodMs);
                break;

            case AnalogVoltage:
                this.config.signals.analogVoltagePeriodMs(periodMs);
                break;

            case AnalogVelocity:
                this.config.signals.analogVelocityPeriodMs(periodMs);
                break;

            case AnalogPosition:
                this.config.signals.analogPositionPeriodMs(periodMs);
                break;

            case ExternalOrAltEncoderVelocity:
                this.config.signals.externalOrAltEncoderVelocity(periodMs);
                break;

            case ExternalOrAltEncoderPosition:
                this.config.signals.externalOrAltEncoderPosition(periodMs);
                break;

            case AbsoluteEncoderVelocity:
                this.config.signals.absoluteEncoderVelocityPeriodMs(periodMs);
                break;

            case AbsoluteEncoderPosition:
                this.config.signals.absoluteEncoderPositionPeriodMs(periodMs);
                break;

            case IAccumulation:
                this.config.signals.iAccumulationPeriodMs(periodMs);
                break;

            default:
                ExceptionHelpers.Assert(false, "unable to confiugure period (ms) for signal type %s", signalType);
                break;
        }
    }

    /**
     * Set the sampling depth of the velocity calculation process of the encoder. This value sets the number of samples in the average for velocity readings. This value must be either 1, 2, 4, or 8 (default)
     */
    public void configureEncoderAverageDepth(int depth)
    {
        this.config.encoder.uvwAverageDepth(depth);
    }

    /**
     * Set the position measurement period used to calculate the velocity of the encoder. This value is in units of milliseconds and must be in a range [8, 64]. The default value is 32ms.
     * The basic formula to calculate velocity is change in position / change in time. This parameter sets the change in time for measurement.
     */
    public void configureVelocityMeasurementPeriod(int periodMs)
    {
        // only supported for relative encoders
        if (!this.useAbsoluteEncoder)
        {
            this.config.encoder.uvwMeasurementPeriod(periodMs);
        }
    }

    public void setSelectedSlot(int slotId)
    {
        this.selectedSlot = SparkBaseWrapper.getSlot(slotId);
    }

    public void configurePIDF(double p, double i, double d, double f, int slotId)
    {
        ClosedLoopSlot slot = SparkBaseWrapper.getSlot(slotId);
        this.config.closedLoop.p(p, slot);
        this.config.closedLoop.i(i, slot);
        this.config.closedLoop.d(d, slot);
        this.config.closedLoop.velocityFF(f, slot);
    }

    public void configurePIDF(double p, double i, double d, double f, double minOutput, double maxOutput, int slotId)
    {
        ClosedLoopSlot slot = SparkBaseWrapper.getSlot(slotId);
        this.config.closedLoop.p(p, slot);
        this.config.closedLoop.i(i, slot);
        this.config.closedLoop.d(d, slot);
        this.config.closedLoop.velocityFF(f, slot);
        this.config.closedLoop.outputRange(minOutput, maxOutput, slot);
    }

    public void configurePIDF(double p, double i, double d, double f, double izone, int slotId)
    {
        ClosedLoopSlot slot = SparkBaseWrapper.getSlot(slotId);
        this.config.closedLoop.p(p, slot);
        this.config.closedLoop.i(i, slot);
        this.config.closedLoop.d(d, slot);
        this.config.closedLoop.velocityFF(f, slot);
        this.config.closedLoop.iZone(izone, slot);
    }

    public void configurePIDF(double p, double i, double d, double f, double izone, double minOutput, double maxOutput, int slotId)
    {
        ClosedLoopSlot slot = SparkBaseWrapper.getSlot(slotId);
        this.config.closedLoop.p(p, slot);
        this.config.closedLoop.i(i, slot);
        this.config.closedLoop.d(d, slot);
        this.config.closedLoop.velocityFF(f, slot);
        this.config.closedLoop.iZone(izone, slot);
        this.config.closedLoop.outputRange(minOutput, maxOutput, slot);
    }

    public void configurePIDFMAXMotion(double p, double i, double d, double f, double izone, double velocity, double acceleration, int slotId)
    {
        ClosedLoopSlot slot = SparkBaseWrapper.getSlot(slotId);
        this.config.closedLoop.p(p, slot);
        this.config.closedLoop.i(i, slot);
        this.config.closedLoop.d(d, slot);
        this.config.closedLoop.velocityFF(f, slot);
        this.config.closedLoop.iZone(izone, slot);
        this.config.closedLoop.maxMotion.maxVelocity(velocity, slot);
        this.config.closedLoop.maxMotion.maxAcceleration(acceleration, slot);
        this.config.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, slot);
    }

    public void configurePIDFMAXMotion(double p, double i, double d, double f, double izone, double velocity, double acceleration, double minOutput, double maxOutput, int slotId)
    {
        ClosedLoopSlot slot = SparkBaseWrapper.getSlot(slotId);
        this.config.closedLoop.p(p, slot);
        this.config.closedLoop.i(i, slot);
        this.config.closedLoop.d(d, slot);
        this.config.closedLoop.velocityFF(f, slot);
        this.config.closedLoop.iZone(izone, slot);
        this.config.closedLoop.outputRange(minOutput, maxOutput, slot);
        this.config.closedLoop.maxMotion.maxVelocity(velocity, slot);
        this.config.closedLoop.maxMotion.maxAcceleration(acceleration, slot);
        this.config.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, slot);
    }

    public void configureForwardLimitSwitch(boolean enabled, boolean normallyOpen)
    {
        LimitSwitchConfig.Type polarity = LimitSwitchConfig.Type.kNormallyClosed;
        if (normallyOpen)
        {
            polarity = LimitSwitchConfig.Type.kNormallyOpen;
        }

        this.config.limitSwitch.forwardLimitSwitchEnabled(enabled);
        this.config.limitSwitch.forwardLimitSwitchType(polarity);

        this.wrappedFwdLimitSwitch = this.wrappedObject.getReverseLimitSwitch();
    }

    public void configureReverseLimitSwitch(boolean enabled, boolean normallyOpen)
    {
        LimitSwitchConfig.Type polarity = LimitSwitchConfig.Type.kNormallyClosed;
        if (normallyOpen)
        {
            polarity = LimitSwitchConfig.Type.kNormallyOpen;
        }

        this.config.limitSwitch.reverseLimitSwitchEnabled(enabled);
        this.config.limitSwitch.reverseLimitSwitchType(polarity);

        this.wrappedRevLimitSwitch = this.wrappedObject.getReverseLimitSwitch();
    }

    public void configureMotorOutputSettings(boolean invert, MotorNeutralMode neutralMode)
    {
        this.config.inverted(invert);

        IdleMode mode;
        if (neutralMode == MotorNeutralMode.Brake)
        {
            mode = IdleMode.kBrake;
        }
        else
        {
            mode = IdleMode.kCoast;
        }

        this.config.idleMode(mode);
    }

    public void configureInvertSensor(boolean invert)
    {
        if (this.useAbsoluteEncoder)
        {
            this.config.absoluteEncoder.inverted(invert);
        }
        else // if (!this.useAbsoluteEncoder)
        {
            this.config.encoder.inverted(invert);
        }
    }

    public void configureSmartCurrentLimit(int stallLimit, int freeLimit, int limitRPM)
    {
        this.config.smartCurrentLimit(stallLimit, freeLimit, limitRPM);
    }

    public void configureSecondaryCurrentLimit(double limit, int chopCycles)
    {
        this.config.secondaryCurrentLimit(limit, chopCycles);
    }

    public void configureAbsoluteOffset(double zeroOffset)
    {
        if (this.useAbsoluteEncoder)
        {
            this.zeroOffset = zeroOffset;
            this.config.absoluteEncoder.zeroOffset(zeroOffset);
        }
    }

    public void configurePositionConversionFactor(double ratio)
    {
        if (this.useAbsoluteEncoder)
        {
            this.config.absoluteEncoder.positionConversionFactor(ratio);
        }
        else // if (!this.useAbsoluteEncoder)
        {
            this.config.encoder.positionConversionFactor(ratio);
        }
    }

    public void configureVelocityConversionFactor(double ratio)
    {
        if (this.useAbsoluteEncoder)
        {
            this.config.absoluteEncoder.velocityConversionFactor(ratio);
        }
        else // if (!this.useAbsoluteEncoder)
        {
            this.config.encoder.velocityConversionFactor(ratio);
        }
    }

    public void configurePositionPIDWrappingSettings(boolean enable, double minInput, double maxInput)
    {
        this.ensureClosedLoopController();

        this.config.closedLoop.positionWrappingEnabled(enable);
        this.config.closedLoop.positionWrappingInputRange(minInput, maxInput);
    }

    protected static com.revrobotics.spark.SparkLowLevel.MotorType getMotorType(SparkMotorType motorType)
    {
        com.revrobotics.spark.SparkLowLevel.MotorType type = com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
        switch (motorType)
        {
            case Brushed:
                type = com.revrobotics.spark.SparkLowLevel.MotorType.kBrushed;
                break;

            case Brushless:
                type = com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
                break;
        }

        return type;
    }

    private static ClosedLoopSlot getSlot(int slotId)
    {
        switch (slotId)
        {
            case 0:
                return ClosedLoopSlot.kSlot0;
            case 1:
                return ClosedLoopSlot.kSlot1;
            case 2:
                return ClosedLoopSlot.kSlot2;
            case 3:
                return ClosedLoopSlot.kSlot3;

            default:
                throw new RuntimeException(String.format("Unexpected slotId %d", slotId));
        }
    }

    private void ensureClosedLoopController()
    {
        if (this.closedLoopController == null)
        {
            this.closedLoopController = this.wrappedObject.getClosedLoopController();
            if (this.useAbsoluteEncoder)
            {
                this.config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
            }
            else
            {
                this.config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
            }
        }
    }
}
