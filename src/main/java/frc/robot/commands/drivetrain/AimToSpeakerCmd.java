package frc.robot.commands.drivetrain;

import java.util.Optional;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.SwerveSys;

public class AimToSpeakerCmd extends Command {

    private final SwerveSys swerveSys;

    private Translation2d targetTranslation;

    private final ProfiledPIDController aimController;

    public AimToSpeakerCmd(SwerveSys swerveSys) {
        this.swerveSys = swerveSys;

        aimController = new ProfiledPIDController(
            AutoConstants.autoAimkP, 0.0, AutoConstants.autoAimkD,
            new Constraints(
                AutoConstants.autoAimTurnSpeedRadPerSec,
                AutoConstants.autoAumTurnAccelRadPerSecSq));

        aimController.enableContinuousInput(-Math.PI, Math.PI);
    }
    
    @Override
    public void initialize() {
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            targetTranslation = FieldConstants.redAllianceSpeakerPose;
        }
        else {
            targetTranslation = FieldConstants.blueAllianceSpeakerPose;
        }
    }
    
    @Override
    public void execute() {
        double lateralDistanceToTargetMeters = swerveSys.getPose().getTranslation().getDistance(targetTranslation);

        double hypotDistanceToTargetMetes = 
		Math.hypot(lateralDistanceToTargetMeters, FieldConstants.speakerTargetHeightMeters - PivotConstants.pivotHeightMeters);

        double timeOfFlightSecs = hypotDistanceToTargetMetes / (RollerConstants.fireRPM * RollerConstants.metersPerSecondPerRPM);

		Translation2d extrapolation = new Translation2d(
            swerveSys.getFieldRelativeVelocity().getX() * timeOfFlightSecs,
            swerveSys.getFieldRelativeVelocity().getY() * timeOfFlightSecs);
    
        Translation2d extrapolatedTranslation = swerveSys.getPose().getTranslation().plus(extrapolation);
        Translation2d extrapolatedTargetOffset = targetTranslation.minus(extrapolatedTranslation);

        Rotation2d targetHeading = Rotation2d.fromRadians(MathUtil.angleModulus(extrapolatedTargetOffset.getAngle().getRadians()));
        
        SmartDashboard.putNumber("target heading deg", targetHeading.getDegrees());

        PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.of(targetHeading));

        if(Math.abs(swerveSys.getHeading().getDegrees() - targetHeading.getDegrees()) > AutoConstants.autoAimToleranceDeg) {
            double aimRadPerSec = aimController.calculate(swerveSys.getHeading().getRadians(), targetHeading.getRadians());
            swerveSys.setOmegaOverrideRadPerSec(Optional.of(aimRadPerSec));
        }
        else {
            swerveSys.setOmegaOverrideRadPerSec(Optional.of(0.0));
        }

        
	}

    @Override
    public void end(boolean isInterrupted) {
        swerveSys.setOmegaOverrideRadPerSec(Optional.empty());
        PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.empty());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
