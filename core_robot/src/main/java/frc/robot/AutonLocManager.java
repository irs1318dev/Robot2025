package frc.robot;
import java.util.Optional;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.lib.helpers.Helpers;
import frc.lib.robotprovider.Alliance;
import frc.lib.robotprovider.IDriverStation;
import frc.lib.robotprovider.IRobotProvider;
import frc.lib.robotprovider.Point2d;

@Singleton
public class AutonLocManager
{
    private boolean isRed;
    private IDriverStation driverStation;
    private double distanceFromReef;
    private double angledReefXDistance;
    private double angledReefYDistance;

    private double angledReefXDistanceOnBumper;
    private double angledReefYDistanceOnBumper;

    private double angledCoralStationXDistanceOnBumper;
    private double angledCoralStationYDistanceOnBumper;

    private double angledCoralStationXDistanceOnBumperFake;
    private double angledCoralStationYDistanceOnBumperFake;

    private double angledCoralStationXDistanceOnBumperReal;
    private double angledCoralStationYDistanceOnBumperReal;

    private double angledCoralStationXDistance;
    private double angledCoralStationYDistance;

    private double distanceFromCoralStation;

    // Processer
    public Point2d P;

    // Intermediary Processer
    public Point2d IP;
    
    // Intermediary General
    public Point2d IG1;
    public Point2d IG2;
    public Point2d IG3;
    public Point2d IG4;
    public Point2d IG5;
    public Point2d IG6;

    // Ground Pieces
    public Point2d GPT;
    public Point2d GPM;
    public Point2d GPB;

    // Coral Station
    public Point2d CT;
    public Point2d CB;

    public Point2d CTF;
    public Point2d CBF;

    public Point2d CTR;
    public Point2d CBR;
    
    // Intermediary Coral Station
    public Point2d ICT;
    public Point2d ICB;



    // Scoring (reef)
    public Point2d S1;
    public Point2d S2;
    public Point2d S3;
    public Point2d S4;
    public Point2d S5;
    public Point2d S6;
    public Point2d S7;
    public Point2d S8;
    public Point2d S9;
    public Point2d S10;
    public Point2d S11;
    public Point2d S12;
    public Point2d S13;
    public Point2d S14;
    public Point2d S15;
    public Point2d S16;
    public Point2d S17;
    public Point2d S18;

    // Intermediary Scoring (reef)
    public Point2d IS1;
    public Point2d IS2;
    public Point2d IS3;
    public Point2d IS4;
    public Point2d IS5;
    public Point2d IS6;
    public Point2d IS7;
    public Point2d IS8;
    public Point2d IS9;
    public Point2d IS10;
    public Point2d IS11;
    public Point2d IS12;
    public Point2d IS13;
    public Point2d IS14;
    public Point2d IS15;
    public Point2d IS16;
    public Point2d IS17;
    public Point2d IS18;

    // Starting position
    public Point2d B1;
    public Point2d B2;
    public Point2d B3;

    public Point2d B1a;
    public Point2d B2a;
    public Point2d B3a;

    public Point2d B1b;
    public Point2d B3b;

    // Intermediary Starting Position
    public Point2d IB1;
    public Point2d IB3;

    // Intermediates from Pieces
    public Point2d IPT;
    public Point2d IPB;
    public Point2d IPM;

    // Net Points
    public Point2d N;

    public AutonLocManager(boolean isRed)
    {
        this.isRed = isRed;
        this.setValues();
    }

    @Inject
    public AutonLocManager(IRobotProvider provider) 
    {
        this.driverStation = provider.getDriverStation();
    }

    public void updateAlliance()
    {
        Optional<Alliance> alliance = this.driverStation.getAlliance();
        this.isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
        this.setValues();
    }

    public boolean getRedUpdateAlliance()
    {
        Optional<Alliance> alliance = this.driverStation.getAlliance();
        this.isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
        this.setValues();
        return isRed;
    }

    public double getOrientationOrHeading(double orientationOrHeading)
    {
        return AutonLocManager.getOrientationOrHeading(this.isRed, orientationOrHeading);
    }

    public boolean getIsRed()
    {
        return this.isRed;
    }

    private void setValues()
    {
        // x:       Red alliance side is positive, Blue alliance side is negative
        // y:       away from scoring table is postive, towards scoring table is negative
        // angle:   blue alliance forwards is 0 degrees, red alliance forwards is 180 degrees

        this.distanceFromReef = 30;
        this.distanceFromCoralStation = 20;

        this.angledReefXDistance = (this.distanceFromReef + HardwareConstants.ROBOT_HALF_SIDE_LENGTH) * Helpers.cosd(60);
        this.angledReefYDistance = (this.distanceFromReef + HardwareConstants.ROBOT_HALF_SIDE_LENGTH) * Helpers.sind(60);

        this.angledReefXDistanceOnBumper = (HardwareConstants.ROBOT_HALF_SIDE_LENGTH) * Helpers.cosd(60);
        this.angledReefYDistanceOnBumper = (HardwareConstants.ROBOT_HALF_SIDE_LENGTH) * Helpers.sind(60);

        this.angledCoralStationXDistanceOnBumper = (HardwareConstants.ROBOT_HALF_SIDE_LENGTH - 2.5) * Helpers.cosd(54);
        this.angledCoralStationYDistanceOnBumper = (HardwareConstants.ROBOT_HALF_SIDE_LENGTH - 2.5) * Helpers.sind(54);

        this.angledCoralStationXDistanceOnBumperFake = (HardwareConstants.ROBOT_HALF_SIDE_LENGTH - 15.0) * Helpers.cosd(54);
        this.angledCoralStationYDistanceOnBumperFake = (HardwareConstants.ROBOT_HALF_SIDE_LENGTH - 15.0) * Helpers.sind(54);

        this.angledCoralStationXDistanceOnBumperReal = (HardwareConstants.ROBOT_HALF_SIDE_LENGTH) * Helpers.cosd(54);
        this.angledCoralStationYDistanceOnBumperReal = (HardwareConstants.ROBOT_HALF_SIDE_LENGTH) * Helpers.sind(54);

        this.angledCoralStationXDistance = (this.distanceFromCoralStation + HardwareConstants.ROBOT_HALF_SIDE_LENGTH - 6) * Helpers.cosd(54);
        this.angledCoralStationYDistance = (this.distanceFromCoralStation + HardwareConstants.ROBOT_HALF_SIDE_LENGTH - 6) * Helpers.sind(54);

        //Y field length: 323in
        //X field length: 653in

        this.P = new Point2d(getNegatedPosition(isRed, -109.7), getNegatedPosition(isRed, -158.5 + HardwareConstants.ROBOT_HALF_SIDE_LENGTH));
        this.IP = new Point2d(getNegatedPosition(isRed, -109.7), getNegatedPosition(isRed, -158.5 + this.distanceFromReef + HardwareConstants.ROBOT_HALF_SIDE_LENGTH));

        this.GPB = new Point2d(getNegatedPosition(isRed, -290 + HardwareConstants.ROBOT_HALF_SIDE_LENGTH - 2), getNegatedPosition(isRed, -72));
        this.GPB = new Point2d(getNegatedPosition(isRed, -290 + HardwareConstants.ROBOT_HALF_SIDE_LENGTH - 2), getNegatedPosition(isRed, 0));
        this.GPB = new Point2d(getNegatedPosition(isRed, -290 + HardwareConstants.ROBOT_HALF_SIDE_LENGTH - 2), getNegatedPosition(isRed, 72));

        this.CT = new Point2d(getNegatedPosition(isRed, -311.9 + this.angledCoralStationXDistanceOnBumper), getNegatedPosition(isRed, -132.6 + this.angledCoralStationYDistanceOnBumper));
        this.CB = new Point2d(getNegatedPosition(isRed, -311.9 + this.angledCoralStationXDistanceOnBumper), getNegatedPosition(isRed, 132.6 - this.angledCoralStationYDistanceOnBumper));
        this.CTF = new Point2d(getNegatedPosition(isRed, -311.9 + this.angledCoralStationXDistanceOnBumperFake), getNegatedPosition(isRed, -132.6 + this.angledCoralStationYDistanceOnBumperFake));
        this.CBF = new Point2d(getNegatedPosition(isRed, -311.9 + this.angledCoralStationXDistanceOnBumperFake), getNegatedPosition(isRed, 132.6 - this.angledCoralStationYDistanceOnBumperFake));
        this.CTR = new Point2d(getNegatedPosition(isRed, -311.9 + this.angledCoralStationXDistanceOnBumperReal), getNegatedPosition(isRed, -132.6 + this.angledCoralStationYDistanceOnBumperReal));
        this.CBR = new Point2d(getNegatedPosition(isRed, -311.9 + this.angledCoralStationXDistanceOnBumperReal), getNegatedPosition(isRed, 132.6 - this.angledCoralStationYDistanceOnBumperReal));

        this.ICT = new Point2d(getNegatedPosition(isRed, -311.9 + this.angledCoralStationXDistance), getNegatedPosition(isRed, -132.6 + this.angledCoralStationYDistance));
        this.ICB = new Point2d(getNegatedPosition(isRed, -311.9 + this.angledCoralStationXDistance), getNegatedPosition(isRed, 132.6 - this.angledCoralStationYDistance));

        this.IG1 = new Point2d(getNegatedPosition(isRed, -161), getNegatedPosition(isRed, -120.0));
        this.IG2 = new Point2d(getNegatedPosition(isRed, -161), getNegatedPosition(isRed, -80));
        this.IG3 = new Point2d(getNegatedPosition(isRed, -161), getNegatedPosition(isRed, -40));
        this.IG4 = new Point2d(getNegatedPosition(isRed, -161), getNegatedPosition(isRed, 40));
        this.IG5 = new Point2d(getNegatedPosition(isRed, -161), getNegatedPosition(isRed, 80));
        this.IG6 = new Point2d(getNegatedPosition(isRed, -161), getNegatedPosition(isRed, 120));

        this.S1 = new Point2d(getNegatedPosition(isRed, -136 + HardwareConstants.ROBOT_HALF_SIDE_LENGTH), getNegatedPosition(isRed, -6.7)); 
        this.S2 = new Point2d(getNegatedPosition(isRed, -136 + HardwareConstants.ROBOT_HALF_SIDE_LENGTH), getNegatedPosition(isRed, 2.0)); 
        this.S3 = new Point2d(getNegatedPosition(isRed, -136 + HardwareConstants.ROBOT_HALF_SIDE_LENGTH), getNegatedPosition(isRed, 6.7));
        this.S4 = new Point2d(getNegatedPosition(isRed, -146.5 + this.angledReefXDistanceOnBumper), getNegatedPosition(isRed, 25 + this.angledReefYDistanceOnBumper));
        this.S5 = new Point2d(getNegatedPosition(isRed, -152.3 + this.angledReefXDistanceOnBumper), getNegatedPosition(isRed, 28.4 + this.angledReefYDistanceOnBumper));
        this.S6 = new Point2d(getNegatedPosition(isRed, -158.3 + this.angledReefXDistanceOnBumper), getNegatedPosition(isRed, 31.7 + this.angledReefYDistanceOnBumper));
        this.S7 = new Point2d(getNegatedPosition(isRed, -179.3 - this.angledReefXDistanceOnBumper), getNegatedPosition(isRed, 31.7 + this.angledReefYDistanceOnBumper));
        this.S8 = new Point2d(getNegatedPosition(isRed, -185 - this.angledReefXDistanceOnBumper), getNegatedPosition(isRed, 28.4 + this.angledReefYDistanceOnBumper));
        this.S9 = new Point2d(getNegatedPosition(isRed, -190.9 - this.angledReefXDistanceOnBumper), getNegatedPosition(isRed, 25 + this.angledReefYDistanceOnBumper));
        this.S10 = new Point2d(getNegatedPosition(isRed, -202 - HardwareConstants.ROBOT_HALF_SIDE_LENGTH), getNegatedPosition(isRed, 6.7)); 
        this.S11 = new Point2d(getNegatedPosition(isRed, -202 - HardwareConstants.ROBOT_HALF_SIDE_LENGTH), getNegatedPosition(isRed, 0)); 
        this.S12 = new Point2d(getNegatedPosition(isRed, -202 - HardwareConstants.ROBOT_HALF_SIDE_LENGTH), getNegatedPosition(isRed, -6.7)); 
        this.S13 = new Point2d(getNegatedPosition(isRed, -190.9 - this.angledReefXDistanceOnBumper), getNegatedPosition(isRed, -25 - this.angledReefYDistanceOnBumper));
        this.S14 = new Point2d(getNegatedPosition(isRed, -185 - this.angledReefXDistanceOnBumper), getNegatedPosition(isRed, -28.4 - this.angledReefYDistanceOnBumper));
        this.S15 = new Point2d(getNegatedPosition(isRed, -179.3 - this.angledReefXDistanceOnBumper), getNegatedPosition(isRed, -31.7 - this.angledReefYDistanceOnBumper));
        this.S16 = new Point2d(getNegatedPosition(isRed, -158.3 + this.angledReefXDistanceOnBumper), getNegatedPosition(isRed, -31.7 - this.angledReefYDistanceOnBumper));
        this.S17 = new Point2d(getNegatedPosition(isRed, -152.3 + this.angledReefXDistanceOnBumper), getNegatedPosition(isRed, -28.4 - this.angledReefYDistanceOnBumper));
        this.S18 = new Point2d(getNegatedPosition(isRed, -146.5 + this.angledReefXDistanceOnBumper), getNegatedPosition(isRed, -25 - this.angledReefYDistanceOnBumper));

        this.IS1 = new Point2d(getNegatedPosition(isRed, -136 + HardwareConstants.ROBOT_HALF_SIDE_LENGTH + this.distanceFromReef), getNegatedPosition(isRed, -6.7)); 
        this.IS2 = new Point2d(getNegatedPosition(isRed, -136 + HardwareConstants.ROBOT_HALF_SIDE_LENGTH + this.distanceFromReef), getNegatedPosition(isRed, 2.0)); 
        this.IS3 = new Point2d(getNegatedPosition(isRed, -136 + HardwareConstants.ROBOT_HALF_SIDE_LENGTH + this.distanceFromReef), getNegatedPosition(isRed, 6.7));
        this.IS4 = new Point2d(getNegatedPosition(isRed, -146.5 + this.angledReefXDistance), getNegatedPosition(isRed, 25 + this.angledReefYDistance));
        this.IS5 = new Point2d(getNegatedPosition(isRed, -152.3 + this.angledReefXDistance), getNegatedPosition(isRed, 28.4 + this.angledReefYDistance));
        this.IS6 = new Point2d(getNegatedPosition(isRed, -158.3 + this.angledReefXDistance), getNegatedPosition(isRed, 31.7 + this.angledReefYDistance));
        this.IS7 = new Point2d(getNegatedPosition(isRed, -179.3 - this.angledReefXDistance), getNegatedPosition(isRed, 31.7 + this.angledReefYDistance));
        this.IS8 = new Point2d(getNegatedPosition(isRed, -185 - this.angledReefXDistance), getNegatedPosition(isRed, 28.4 + this.angledReefYDistance));
        this.IS9 = new Point2d(getNegatedPosition(isRed, -190.9 - this.angledReefXDistance), getNegatedPosition(isRed, 25 + this.angledReefYDistance));
        this.IS10 = new Point2d(getNegatedPosition(isRed, -202 - HardwareConstants.ROBOT_HALF_SIDE_LENGTH + this.distanceFromReef), getNegatedPosition(isRed, 6.7)); 
        this.IS11 = new Point2d(getNegatedPosition(isRed, -202 - HardwareConstants.ROBOT_HALF_SIDE_LENGTH + this.distanceFromReef), getNegatedPosition(isRed, 0)); 
        this.IS12 = new Point2d(getNegatedPosition(isRed, -202 - HardwareConstants.ROBOT_HALF_SIDE_LENGTH + this.distanceFromReef), getNegatedPosition(isRed, -6.7)); 
        this.IS13 = new Point2d(getNegatedPosition(isRed, -190.9 - this.angledReefXDistance), getNegatedPosition(isRed, -25 - this.angledReefYDistance));
        this.IS14 = new Point2d(getNegatedPosition(isRed, -185 - this.angledReefXDistance), getNegatedPosition(isRed, -28.4 - this.angledReefYDistance));
        this.IS15 = new Point2d(getNegatedPosition(isRed, -179.3 - this.angledReefXDistance), getNegatedPosition(isRed, -31.7 - this.angledReefYDistance));
        this.IS16 = new Point2d(getNegatedPosition(isRed, -158.3 + this.angledReefXDistance), getNegatedPosition(isRed, -31.7 - this.angledReefYDistance));
        this.IS17 = new Point2d(getNegatedPosition(isRed, -152.2 + this.angledReefXDistance), getNegatedPosition(isRed, -28.4 - this.angledReefYDistance));
        this.IS18 = new Point2d(getNegatedPosition(isRed, -146.5 + this.angledReefXDistance), getNegatedPosition(isRed, -25 - this.angledReefYDistance));
        
        this.B1 = new Point2d(getNegatedPosition(isRed, -46 - HardwareConstants.ROBOT_HALF_SIDE_LENGTH), getNegatedPosition(isRed, 158.5 - HardwareConstants.ROBOT_HALF_SIDE_LENGTH));
        this.B2 = new Point2d(getNegatedPosition(isRed, -46 - HardwareConstants.ROBOT_HALF_SIDE_LENGTH), getNegatedPosition(isRed, 0));
        this.B3 = new Point2d(getNegatedPosition(isRed, -46 - HardwareConstants.ROBOT_HALF_SIDE_LENGTH), getNegatedPosition(isRed, -158.5 + HardwareConstants.ROBOT_HALF_SIDE_LENGTH));

        double robotLengthwiseDistance = Math.sqrt(2.0 * Math.pow(HardwareConstants.ROBOT_FULL_SIDE_LENGTH, 2.0));
        double robotHalfLengthwiseDistance = Math.sqrt(2.0 * Math.pow(HardwareConstants.ROBOT_HALF_SIDE_LENGTH, 2.0));
        this.B1a = new Point2d(getNegatedPosition(isRed, -46 - robotLengthwiseDistance + 1), getNegatedPosition(isRed, 158.5 - robotHalfLengthwiseDistance));
        this.B2a = new Point2d(getNegatedPosition(isRed, -46 - HardwareConstants.ROBOT_FULL_SIDE_LENGTH + 1), getNegatedPosition(isRed, 0));
        this.B3a = new Point2d(getNegatedPosition(isRed, -46 - robotLengthwiseDistance + 1), getNegatedPosition(isRed, -158.5 + robotHalfLengthwiseDistance));

        this.B1b = new Point2d(getNegatedPosition(isRed, -46 - HardwareConstants.ROBOT_HALF_SIDE_LENGTH), getNegatedPosition(isRed, 83));
        this.B3b = new Point2d(getNegatedPosition(isRed, -46 - HardwareConstants.ROBOT_HALF_SIDE_LENGTH), getNegatedPosition(isRed, -83));

        this.IB1 = new Point2d(getNegatedPosition(isRed, -56 - HardwareConstants.ROBOT_FULL_SIDE_LENGTH), getNegatedPosition(isRed, 148.0 - HardwareConstants.ROBOT_HALF_SIDE_LENGTH));
        this.IB3 = new Point2d(getNegatedPosition(isRed, -56 - HardwareConstants.ROBOT_FULL_SIDE_LENGTH), getNegatedPosition(isRed, -148.0 + HardwareConstants.ROBOT_HALF_SIDE_LENGTH));

        this.IPB = new Point2d(getNegatedPosition(isRed, -250), getNegatedPosition(isRed, 72));
        this.IPM = new Point2d(getNegatedPosition(isRed, -250), getNegatedPosition(isRed, 0));
        this.IPT = new Point2d(getNegatedPosition(isRed, -250), getNegatedPosition(isRed, -72));

        this.N = new Point2d(getNegatedPosition(isRed, -56), getNegatedPosition(isRed, 60));
    }

    private static double getOrientationOrHeading(boolean isRed, double orientationOrHeading)
    {
        if (isRed)
        {
            return 180.0 + orientationOrHeading;
        }
        else
        {
            return orientationOrHeading;
        }
    }

    private static double getNegatedPosition(boolean isRed, double position)
    {
        if (isRed)
        {
            return -1.0 * position;
        }
        else
        {
            return position;
        }
    }
    
}
