package frc.robot.driver;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import frc.lib.driver.TrajectoryManager;
import frc.lib.robotprovider.MotorType;
import frc.lib.robotprovider.PathPlannerWrapper;

public class PathPlannerTrajectoryTests
{
    @BeforeEach
    public void setup()
    {
        // assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    }

    @Test
    public void verifyTrajectoryGeneration()
    {
        // TrajectoryManager trajectoryManager = new TrajectoryManager();
        // PathPlannerWrapper pathPlanner = PathPlannerWrapper.Instance;
        // pathPlanner.configureRobot(
        //    120.0,
        //    1.0,
        //    3.95,
        //    120.0,
        //    1.0,
        //    36000.0 / 5880.0,
        //    MotorType.KrakenX60,
        //    1,
        //    40,
        //    12.0,
        //    12.0);

        // PathPlannerTrajectoryGenerator.generateTrajectories(trajectoryManager, pathPlanner);
    }
}