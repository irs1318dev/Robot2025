package frc.robot.mechanisms;

import frc.robot.*;
import frc.lib.driver.IDriver;
import frc.lib.helpers.Helpers;
import frc.lib.mechanisms.IMechanism;
import frc.lib.mechanisms.LoggingManager;
import frc.lib.robotprovider.*;
import frc.robot.driver.*;

import java.util.List;
import java.util.Optional;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Offboard Vision manager.
 */
@Singleton
public class OffboardVisionManager implements IMechanism
{
    public static final DigitalOperation[] PossibleVisionOperations =
    {
        DigitalOperation.VisionFindAnyAprilTag,
        DigitalOperation.VisionFindSpecificAprilTag,
        DigitalOperation.VisionFindAbsolutePosition,
        DigitalOperation.VisionFindReefTagsOnly,
        DigitalOperation.VisionFindReefRightFarTagOnly,
        DigitalOperation.VisionFindReefLeftFarTagOnly,
        DigitalOperation.VisionFindReefRightCloseTagOnly,
        DigitalOperation.VisionFindReefLeftCloseTagOnly,
        DigitalOperation.VisionFindReefCenterCloseTagOnly,
        DigitalOperation.VisionFindReefCenterFarTagOnly,
    };

    private final IDriver driver;
    private final ILogger logger;
    private final IDriverStation ds;

    private final IDoubleSubscriber atrXOffsetSubscriber;
    private final IDoubleSubscriber atrYOffsetSubscriber;
    private final IDoubleSubscriber atrZOffsetSubscriber;
    private final IDoubleSubscriber atrYawSubscriber;
    private final IDoubleSubscriber atrPitchSubscriber;
    private final IDoubleSubscriber atrRollSubscriber;
    private final IDoubleSubscriber atrIdSubscriber;
    private final IDoubleSubscriber atrScoreSubscriber;

    private final IDoubleSubscriber atlXOffsetSubscriber;
    private final IDoubleSubscriber atlYOffsetSubscriber;
    private final IDoubleSubscriber atlZOffsetSubscriber;
    private final IDoubleSubscriber atlYawSubscriber;
    private final IDoubleSubscriber atlPitchSubscriber;
    private final IDoubleSubscriber atlRollSubscriber;
    private final IDoubleSubscriber atlIdSubscriber;
    private final IDoubleSubscriber atlScoreSubscriber;

    private final IDoubleSubscriber abslXOffsetSubscriber;
    private final IDoubleSubscriber abslYOffsetSubscriber;
    private final IDoubleSubscriber abslZOffsetSubscriber;
    private final IDoubleSubscriber abslRollAngleSubscriber;
    private final IDoubleSubscriber abslPitchAngleSubscriber;
    private final IDoubleSubscriber abslYawAngleSubscriber;
    private final IDoubleSubscriber abslTagIdSubscriber;
    private final IDoubleSubscriber abslDecisionMarginSubscriber;
    private final IDoubleSubscriber abslErrorSubscriber;

    private final IDoubleSubscriber absrXOffsetSubscriber;
    private final IDoubleSubscriber absrYOffsetSubscriber;
    private final IDoubleSubscriber absrZOffsetSubscriber;
    private final IDoubleSubscriber absrRollAngleSubscriber;
    private final IDoubleSubscriber absrPitchAngleSubscriber;
    private final IDoubleSubscriber absrYawAngleSubscriber;
    private final IDoubleSubscriber absrTagIdSubscriber;
    private final IDoubleSubscriber absrDecisionMarginSubscriber;
    private final IDoubleSubscriber absrErrorSubscriber;

    private final IDoubleSubscriber ledModeSubscriber;
    private final IDoubleSubscriber ledValueSubscriber;

    private final IDoubleSubscriber heartbeatSubscriber;

    private Double atXOffset;
    private Double atYOffset;
    private Double atZOffset;
    private Double atYaw;
    private Double atPitch;
    private Double atRoll;
    private Integer atId;

    private Double absXOffset;
    private Double absYOffset;
    private Double absZOffset;
    private Double absRoll;
    private Double absPitch;
    private Double absYaw;
    private Integer absId;
    private Double absDecisionMargin;
    private Double absError;

    private boolean enableVision;
    private List<Integer> prevTargets;

    private int missedHeartbeats;
    private double prevHeartbeat;

    /**
     * Initializes a new OffboardVisionManager
     * @param driver for obtaining operations
     * @param logger for logging to smart dashboard
     * @param provider for obtaining electronics objects
     */
    @Inject
    public OffboardVisionManager(IDriver driver, LoggingManager logger, IRobotProvider provider)
    {
        this.driver = driver;
        this.logger = logger;
        this.ds = provider.getDriverStation();

        INetworkTableProvider networkTable = provider.getNetworkTableProvider();

        this.atrXOffsetSubscriber = networkTable.getDoubleSubscriber("atr.xOffset", TuningConstants.MAGIC_NULL_VALUE);
        this.atrYOffsetSubscriber = networkTable.getDoubleSubscriber("atr.yOffset", TuningConstants.MAGIC_NULL_VALUE);
        this.atrZOffsetSubscriber = networkTable.getDoubleSubscriber("atr.zOffset", TuningConstants.MAGIC_NULL_VALUE);
        this.atrYawSubscriber = networkTable.getDoubleSubscriber("atr.yawAngle", TuningConstants.MAGIC_NULL_VALUE);
        this.atrPitchSubscriber = networkTable.getDoubleSubscriber("atr.pitchAngle", TuningConstants.MAGIC_NULL_VALUE);
        this.atrRollSubscriber = networkTable.getDoubleSubscriber("atr.rollAngle", TuningConstants.MAGIC_NULL_VALUE);
        this.atrIdSubscriber = networkTable.getDoubleSubscriber("atr.tagId", (int)TuningConstants.MAGIC_NULL_VALUE);
        this.atrScoreSubscriber = networkTable.getDoubleSubscriber("atr.tagScore", TuningConstants.MAGIC_NULL_VALUE);

        this.atlXOffsetSubscriber = networkTable.getDoubleSubscriber("atl.xOffset", TuningConstants.MAGIC_NULL_VALUE);
        this.atlYOffsetSubscriber = networkTable.getDoubleSubscriber("atl.yOffset", TuningConstants.MAGIC_NULL_VALUE);
        this.atlZOffsetSubscriber = networkTable.getDoubleSubscriber("atl.zOffset", TuningConstants.MAGIC_NULL_VALUE);
        this.atlYawSubscriber = networkTable.getDoubleSubscriber("atl.yawAngle", TuningConstants.MAGIC_NULL_VALUE);
        this.atlPitchSubscriber = networkTable.getDoubleSubscriber("atl.pitchAngle", TuningConstants.MAGIC_NULL_VALUE);
        this.atlRollSubscriber = networkTable.getDoubleSubscriber("atl.rollAngle", TuningConstants.MAGIC_NULL_VALUE);
        this.atlIdSubscriber = networkTable.getDoubleSubscriber("atl.tagId", (int)TuningConstants.MAGIC_NULL_VALUE);
        this.atlScoreSubscriber = networkTable.getDoubleSubscriber("atl.tagScore", TuningConstants.MAGIC_NULL_VALUE);

        this.abslXOffsetSubscriber = networkTable.getDoubleSubscriber("absl.xOffset", TuningConstants.MAGIC_NULL_VALUE);
        this.abslYOffsetSubscriber = networkTable.getDoubleSubscriber("absl.yOffset", TuningConstants.MAGIC_NULL_VALUE);
        this.abslZOffsetSubscriber = networkTable.getDoubleSubscriber("absl.zOffset", TuningConstants.MAGIC_NULL_VALUE);
        this.abslRollAngleSubscriber = networkTable.getDoubleSubscriber("absl.rollAngle", TuningConstants.MAGIC_NULL_VALUE);
        this.abslPitchAngleSubscriber = networkTable.getDoubleSubscriber("absl.pitchAngle", TuningConstants.MAGIC_NULL_VALUE);
        this.abslYawAngleSubscriber = networkTable.getDoubleSubscriber("absl.yawAngle", TuningConstants.MAGIC_NULL_VALUE);
        this.abslTagIdSubscriber = networkTable.getDoubleSubscriber("absl.tagId", TuningConstants.MAGIC_NULL_VALUE);
        this.abslDecisionMarginSubscriber = networkTable.getDoubleSubscriber("absl.decisionMargin", TuningConstants.MAGIC_NULL_VALUE);
        this.abslErrorSubscriber = networkTable.getDoubleSubscriber("absl.error", TuningConstants.MAGIC_NULL_VALUE);

        this.absrXOffsetSubscriber = networkTable.getDoubleSubscriber("absr.xOffset", TuningConstants.MAGIC_NULL_VALUE);
        this.absrYOffsetSubscriber = networkTable.getDoubleSubscriber("absr.yOffset", TuningConstants.MAGIC_NULL_VALUE);
        this.absrZOffsetSubscriber = networkTable.getDoubleSubscriber("absr.zOffset", TuningConstants.MAGIC_NULL_VALUE);
        this.absrRollAngleSubscriber = networkTable.getDoubleSubscriber("absr.rollAngle", TuningConstants.MAGIC_NULL_VALUE);
        this.absrPitchAngleSubscriber = networkTable.getDoubleSubscriber("absr.pitchAngle", TuningConstants.MAGIC_NULL_VALUE);
        this.absrYawAngleSubscriber = networkTable.getDoubleSubscriber("absr.yawAngle", TuningConstants.MAGIC_NULL_VALUE);
        this.absrTagIdSubscriber = networkTable.getDoubleSubscriber("absr.tagId", TuningConstants.MAGIC_NULL_VALUE);
        this.absrDecisionMarginSubscriber = networkTable.getDoubleSubscriber("absr.decisionMargin", TuningConstants.MAGIC_NULL_VALUE);
        this.absrErrorSubscriber = networkTable.getDoubleSubscriber("absr.error", TuningConstants.MAGIC_NULL_VALUE);

        this.heartbeatSubscriber = networkTable.getDoubleSubscriber("bellypan.heartbeat", 0);

        this.ledModeSubscriber = networkTable.getDoubleSubscriber("led.mode", TuningConstants.MAGIC_NULL_VALUE);
        this.ledValueSubscriber = networkTable.getDoubleSubscriber("led.value", TuningConstants.MAGIC_NULL_VALUE);

        this.enableVision = false;
        this.prevTargets = null;

        this.missedHeartbeats = 0;
        this.prevHeartbeat = 0.0;
    }

    /**
     * read all the sensors for the mechanism that we will use in macros/autonomous mode and record their values
     */
    @Override
    public void readSensors()
    {
        double atrXOffset = this.atrXOffsetSubscriber.get();
        double atrYOffset = this.atrYOffsetSubscriber.get();
        double atrZOffset = this.atrZOffsetSubscriber.get();
        double atrYaw = this.atrYawSubscriber.get();
        double atrPitch = this.atrPitchSubscriber.get();
        double atrRoll = this.atrRollSubscriber.get();
        int atrId = (int)this.atrIdSubscriber.get();
        double atrScore = this.atrScoreSubscriber.get();

        double atlXOffset = this.atlXOffsetSubscriber.get();
        double atlYOffset = this.atlYOffsetSubscriber.get();
        double atlZOffset = this.atlZOffsetSubscriber.get();
        double atlYaw = this.atlYawSubscriber.get();
        double atlPitch = this.atlPitchSubscriber.get();
        double atlRoll = this.atlRollSubscriber.get();
        int atlId = (int)this.atlIdSubscriber.get();
        double atlScore = this.atlScoreSubscriber.get();

        double abslXOffset = this.abslXOffsetSubscriber.get();
        double abslYOffset = this.abslYOffsetSubscriber.get();
        double abslZOffset = this.abslZOffsetSubscriber.get();
        double abslRoll = this.abslRollAngleSubscriber.get();
        double abslPitch = this.abslPitchAngleSubscriber.get();
        double abslYaw = this.abslYawAngleSubscriber.get();
        int abslId = (int)this.abslTagIdSubscriber.get();
        double abslDecisionMargin = this.abslDecisionMarginSubscriber.get();
        double abslError = this.abslErrorSubscriber.get();

        double absrXOffset = this.absrXOffsetSubscriber.get();
        double absrYOffset = this.absrYOffsetSubscriber.get();
        double absrZOffset = this.absrZOffsetSubscriber.get();
        double absrRoll = this.absrRollAngleSubscriber.get();
        double absrPitch = this.absrPitchAngleSubscriber.get();
        double absrYaw = this.absrYawAngleSubscriber.get();
        int absrId = (int)this.absrTagIdSubscriber.get();
        double absrDecisionMargin = this.absrDecisionMarginSubscriber.get();
        double absrError = this.absrErrorSubscriber.get();

        double newHeartbeat = this.heartbeatSubscriber.get();
        if (!Helpers.roughEquals(this.prevHeartbeat, newHeartbeat, 0.5))
        {
            this.missedHeartbeats = 0;
        }
        else
        {
            this.missedHeartbeats++;
        }

        this.prevHeartbeat = newHeartbeat;
        this.logger.logNumber(LoggingKey.OffboardVisionMissedHeartbeats, this.missedHeartbeats);

        boolean missedHeartbeatExceedsThreshold = this.missedHeartbeats > TuningConstants.VISION_MISSED_HEARTBEAT_THRESHOLD;
        this.logger.logBoolean(LoggingKey.OffboardVisionExcessiveMissedHeartbeats, missedHeartbeatExceedsThreshold);

        // reset if we couldn't find the april tag
        this.atXOffset = null;
        this.atYOffset = null;
        this.atZOffset = null;
        this.atYaw = null;
        this.atPitch = null;
        this.atRoll = null;
        this.atId = null;

        this.absXOffset = null;
        this.absYOffset = null;
        this.absZOffset = null;
        this.absRoll = null;
        this.absPitch = null;
        this.absYaw = null;
        this.absId = null;
        this.absDecisionMargin = null;
        this.absError = null;

        if (!missedHeartbeatExceedsThreshold && this.enableVision)
        {
            // since these conditions are used in multiple places, I put them in memory for conciseness
            boolean atrFound = atrXOffset != TuningConstants.MAGIC_NULL_VALUE &&
                atrYOffset != TuningConstants.MAGIC_NULL_VALUE &&
                atrZOffset != TuningConstants.MAGIC_NULL_VALUE &&
                (this.prevTargets == null || this.prevTargets.contains(atrId));

            boolean atlFound = atlXOffset != TuningConstants.MAGIC_NULL_VALUE &&
                atlYOffset != TuningConstants.MAGIC_NULL_VALUE &&
                atlZOffset != TuningConstants.MAGIC_NULL_VALUE &&
                (this.prevTargets == null || this.prevTargets.contains(atlId));

            if (TuningConstants.VISION_USE_SCORE && atrFound && atlFound)
            {
                if (atlScore > atrScore)
                {
                    this.atXOffset = atlXOffset;
                    this.atYOffset = atlYOffset;
                    this.atZOffset = atlZOffset;
                    this.atYaw = atlYaw;
                    this.atPitch = atlPitch;
                    this.atRoll = atlRoll;
                    this.atId = atlId;
                }
                else
                {
                    this.atXOffset = atrXOffset;
                    this.atYOffset = atrYOffset;
                    this.atZOffset = atrZOffset;
                    this.atYaw = atrYaw;
                    this.atPitch = atrPitch;
                    this.atRoll = atrRoll;
                    this.atId = atrId;
                }
            }
            else if (atrFound)
            {
                this.atXOffset = atrXOffset;
                this.atYOffset = atrYOffset;
                this.atZOffset = atrZOffset;
                this.atYaw = atrYaw;
                this.atPitch = atrPitch;
                this.atRoll = atrRoll;
                this.atId = atrId;
            }
            else if (atlFound)
            {
                this.atXOffset = atlXOffset;
                this.atYOffset = atlYOffset;
                this.atZOffset = atlZOffset;
                this.atYaw = atlYaw;
                this.atPitch = atlPitch;
                this.atRoll = atlRoll;
                this.atId = atlId;
            }

            if (absrXOffset != TuningConstants.MAGIC_NULL_VALUE &&
                absrYOffset != TuningConstants.MAGIC_NULL_VALUE &&
                absrZOffset != TuningConstants.MAGIC_NULL_VALUE)
            {
                this.absXOffset = absrXOffset;
                this.absYOffset = absrYOffset;
                this.absZOffset = absrZOffset;
                this.absYaw = absrYaw;
                this.absPitch = absrPitch;
                this.absRoll = absrRoll;
                this.absId = absrId;
            }
            else if (abslXOffset != TuningConstants.MAGIC_NULL_VALUE &&
                abslYOffset != TuningConstants.MAGIC_NULL_VALUE &&
                abslZOffset != TuningConstants.MAGIC_NULL_VALUE)
            {
                this.absXOffset = abslXOffset;
                this.absYOffset = abslYOffset;
                this.absZOffset = abslZOffset;
                this.absYaw = abslYaw;
                this.absPitch = abslPitch;
                this.absRoll = abslRoll;
                this.absId = abslId;
            }
        }

        this.logger.logNumber(LoggingKey.OffboardVisionAprilTagXOffset, this.atXOffset);
        this.logger.logNumber(LoggingKey.OffboardVisionAprilTagYOffset, this.atYOffset);
        this.logger.logNumber(LoggingKey.OffboardVisionAprilTagZOffset, this.atZOffset);
        this.logger.logNumber(LoggingKey.OffboardVisionAprilTagYaw, this.atYaw);
        this.logger.logNumber(LoggingKey.OffboardVisionAprilTagPitch, this.atPitch);
        this.logger.logNumber(LoggingKey.OffboardVisionAprilTagRoll, this.atRoll);
        this.logger.logInteger(LoggingKey.OffboardVisionAprilTagId, this.atId);
    }

    @Override
    public void update(RobotMode mode)
    {
        this.enableVision = !this.driver.getDigital(DigitalOperation.VisionForceDisable);
        boolean enableVideoStream = mode == RobotMode.Test || this.driver.getDigital(DigitalOperation.VisionEnableStream);

        Optional<Alliance> alliance = this.ds.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;

        int visionMode = 0;
        List<Integer> desiredTargets = null;
        String desiredTargetsString = null;
        if (this.driver.getDigital(DigitalOperation.VisionFindReefTagsOnly))
        {
            visionMode = 1;
            if (isRed)
            {
                desiredTargets = TuningConstants.VISION_REEF_RED_APRILTAGS;
                desiredTargetsString = TuningConstants.VISION_REEF_RED_STRING;
            }
            else
            {
                desiredTargets = TuningConstants.VISION_REEF_BLUE_APRILTAGS;
                desiredTargetsString = TuningConstants.VISION_REEF_BLUE_STRING;
            }
        }
        else if (this.driver.getDigital(DigitalOperation.VisionFindReefLeftFarTagOnly))
        {
            visionMode = 1;
            if (isRed)
            {
                desiredTargets = TuningConstants.VISION_REEF_LEFT_FAR_RED_APRILTAGS;
                desiredTargetsString = TuningConstants.VISION_REEF_LEFT_FAR_RED_STRING;
            }
            else
            {
                desiredTargets = TuningConstants.VISION_REEF_LEFT_FAR_BLUE_APRILTAGS;
                desiredTargetsString = TuningConstants.VISION_REEF_LEFT_FAR_BLUE_STRING;
            }
        }
        else if (this.driver.getDigital(DigitalOperation.VisionFindReefRightFarTagOnly))
        {
            visionMode = 1;
            if (isRed)
            {
                desiredTargets = TuningConstants.VISION_REEF_RIGHT_FAR_RED_APRILTAGS;
                desiredTargetsString = TuningConstants.VISION_REEF_RIGHT_FAR_RED_STRING;
            }
            else
            {
                desiredTargets = TuningConstants.VISION_REEF_RIGHT_FAR_BLUE_APRILTAGS;
                desiredTargetsString = TuningConstants.VISION_REEF_RIGHT_FAR_BLUE_STRING;
            }
        }
        else if (this.driver.getDigital(DigitalOperation.VisionFindReefLeftCloseTagOnly))
        {
            visionMode = 1;
            if (isRed)
            {
                desiredTargets = TuningConstants.VISION_REEF_LEFT_CLOSE_RED_APRILTAGS;
                desiredTargetsString = TuningConstants.VISION_REEF_LEFT_CLOSE_RED_STRING;
            }
            else
            {
                desiredTargets = TuningConstants.VISION_REEF_LEFT_CLOSE_BLUE_APRILTAGS;
                desiredTargetsString = TuningConstants.VISION_REEF_LEFT_CLOSE_BLUE_STRING;
            }
        }
        else if (this.driver.getDigital(DigitalOperation.VisionFindReefRightCloseTagOnly))
        {
            visionMode = 1;
            if (isRed)
            {
                desiredTargets = TuningConstants.VISION_REEF_RIGHT_CLOSE_RED_APRILTAGS;
                desiredTargetsString = TuningConstants.VISION_REEF_RIGHT_CLOSE_RED_STRING;
            }
            else
            {
                desiredTargets = TuningConstants.VISION_REEF_RIGHT_CLOSE_BLUE_APRILTAGS;
                desiredTargetsString = TuningConstants.VISION_REEF_RIGHT_CLOSE_BLUE_STRING;
            }
        }
        else if (this.driver.getDigital(DigitalOperation.VisionFindReefCenterCloseTagOnly))
        {
            visionMode = 1;
            if (isRed)
            {
                desiredTargets = TuningConstants.VISION_REEF_CENTER_CLOSE_RED_APRILTAGS;
                desiredTargetsString = TuningConstants.VISION_REEF_CENTER_CLOSE_RED_STRING;
            }
            else
            {
                desiredTargets = TuningConstants.VISION_REEF_CENTER_CLOSE_BLUE_APRILTAGS;
                desiredTargetsString = TuningConstants.VISION_REEF_CENTER_CLOSE_BLUE_STRING;
            }
        }
        else if (this.driver.getDigital(DigitalOperation.VisionFindReefCenterFarTagOnly))
        {
            visionMode = 1;
            if (isRed)
            {
                desiredTargets = TuningConstants.VISION_REEF_CENTER_FAR_RED_APRILTAGS;
                desiredTargetsString = TuningConstants.VISION_REEF_CENTER_FAR_RED_STRING;
            }
            else
            {
                desiredTargets = TuningConstants.VISION_REEF_CENTER_FAR_BLUE_APRILTAGS;
                desiredTargetsString = TuningConstants.VISION_REEF_CENTER_FAR_BLUE_STRING;
            }
        }
        else if (this.driver.getDigital(DigitalOperation.VisionFindAnyAprilTag))
        {
            visionMode = 1;
        }
        else if (this.driver.getDigital(DigitalOperation.VisionFindAbsolutePosition))
        {
            visionMode = 2;
        }

        this.prevTargets = desiredTargets;
        this.logger.logBoolean(LoggingKey.OffboardVisionEnableStream, enableVideoStream);
        this.logger.logInteger(LoggingKey.OffboardVisionMode, visionMode);
        this.logger.logString(LoggingKey.OffboardVisionDesiredTarget, desiredTargetsString);
    }

    @Override
    public void stop()
    {
        this.enableVision = false;
        this.prevTargets = null;
        this.logger.logBoolean(LoggingKey.OffboardVisionEnableStream, false);
        this.logger.logInteger(LoggingKey.OffboardVisionMode, 0);
        this.logger.logString(LoggingKey.OffboardVisionDesiredTarget, "");
    }

    public Double getAprilTagXOffset()
    {
        return this.atXOffset;
    }

    public Double getAprilTagYOffset()
    {
        return this.atYOffset;
    }

    public Double getAprilTagZOffset()
    {
        return this.atZOffset;
    }

    public Double getAprilTagYaw()
    {
        return this.atYaw;
    }

    public Double getAprilTagPitch()
    {
        return this.atPitch;
    }

    public Double getAprilTagRoll()
    {
        return this.atRoll;
    }

    public Integer getAprilTagId()
    {
        return this.atId;
    }

    public Double getAbsolutePositionX()
    {
        return this.absXOffset;
    }

    public Double getAbsolutePositionY()
    {
        return this.absYOffset;
    }

    public Double getAbsolutePositionZ()
    {
        return this.absZOffset;
    }

    public Double getAbsolutePositionYaw()
    {
        return this.absYaw;
    }

    public Double getAbsolutePositionPitch()
    {
        return this.absPitch;
    }

    public Double getAbsolutePositionRoll()
    {
        return this.absRoll;
    }

    public Integer getAbsolutePositionTagId()
    {
        return this.absId;
    }

    public Double getAbsolutePositionDecisionMargin()
    {
        return this.absDecisionMargin;
    }

    public Double getAbsolutePositionError()
    {
        return this.absError;
    }
}