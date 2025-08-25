package frc.lib.robotprovider;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.helpers.Helpers;

public class Field2dWrapper implements IField2d
{
    private final Field2d wrappedObject;

    private Field2dWrapper(Field2d wrappedObject)
    {
        this.wrappedObject = wrappedObject;
    }

    public static IField2d create(String name)
    {
        Field2d field = new Field2d();
        SmartDashboard.putData(name, field);
        return new Field2dWrapper(field);
    }

    @Override
    public void setRobotPose(double xPos, double yPos, double yaw) 
    {
        this.wrappedObject.setRobotPose(
            xPos * Helpers.METERS_PER_INCH,
            yPos * Helpers.METERS_PER_INCH,
            Rotation2d.fromDegrees(yaw));
    }

    @Override
    public frc.lib.robotprovider.Pose2d getRobotPose() 
    {
        edu.wpi.first.math.geometry.Pose2d pose2d = this.wrappedObject.getRobotPose();

        return new Pose2d(
            pose2d.getX() * Helpers.INCHES_PER_METER,
            pose2d.getY() * Helpers.INCHES_PER_METER,
            pose2d.getRotation().getDegrees());
    }
}
