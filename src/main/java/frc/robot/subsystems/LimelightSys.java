/*package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LimelightSys extends SubsystemBase {

    private double[] botPoseArray;  

    private double xPos;
    private double yPos;
    private final double BLUE_TARGET_X_POS = 1.5;
    private final double BLUE_TARGET_Y_POS = 5.5;
    private final double RED_TARGET_X_POS = 15.5;
    private final double RED_TARGET_Y_POS = 5.5;
    // private double rawAngle;
    private double distance;
    private double angle;

    private Supplier<Rotation2d> robotHeading;

    public LimelightSys(Supplier<Rotation2d> robotHeading) {
        this.robotHeading = robotHeading;        
    }

    @Override
    public void periodic() {
        botPoseArray = LimelightHelpers.getBotPose_wpiBlue("limelight-shooter");

        this.xPos = botPoseArray[0];
        this.yPos = botPoseArray[1];
        this.distance = calculateDistance();
        this.angle = calculateAngle(robotHeading.get());
        SmartDashboard.putNumber("xPos", xPos);
        SmartDashboard.putNumber("yPos", yPos);
        SmartDashboard.putNumber("distance", distance);

        getRotValue();
        getXValue();
    }

    private double calculateAngle(Rotation2d robotHeading) {
        double heading = robotHeading.getDegrees() % 360;

        if (heading > 180) {
            heading -= 360;
        }
        heading *= -1;


        double angle = 0;
        SmartDashboard.putNumber("heading", heading);

        if(DriverStation.getAlliance().get() == Alliance.Blue) {
            angle = Math.atan((yPos - BLUE_TARGET_Y_POS) / (xPos - BLUE_TARGET_X_POS));
        }
        else {
            angle = Math.atan((yPos - RED_TARGET_Y_POS) / (xPos - RED_TARGET_X_POS));
        }
        angle = Math.toDegrees(angle);
        // rawAngle = angle;
        angle += heading;
        SmartDashboard.putNumber("Angle" , angle);
        return angle;
    }

    private double calculateDistance() {

        double distance = 0;

        if(DriverStation.getAlliance().get() == Alliance.Blue) {
            distance = Math.sqrt(((BLUE_TARGET_X_POS - xPos)*(BLUE_TARGET_X_POS - xPos)) - ((BLUE_TARGET_Y_POS - yPos)*(BLUE_TARGET_Y_POS - yPos)));
        } 
        else {
            distance = Math.sqrt(((xPos - RED_TARGET_X_POS)*(xPos - RED_TARGET_X_POS)) - ((yPos - RED_TARGET_Y_POS)*(yPos - RED_TARGET_Y_POS)));
        }
        return distance;
    }

    public double getRotValue() {
        SmartDashboard.putNumber("RotValue", 0.01667 * angle);

        return 0.01667 * angle; 
    }


    public double getXValue() {
        double offSet = yPos - BLUE_TARGET_Y_POS;

        SmartDashboard.putNumber("x", offSet * 0.01667);
        return 0.01667*offSet;
    }

}
*/