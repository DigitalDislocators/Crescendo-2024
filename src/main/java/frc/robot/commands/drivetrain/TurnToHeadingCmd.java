package frc.robot.commands.drivetrain;

import java.util.Optional;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveSys;

public class TurnToHeadingCmd extends Command {

    private final SwerveSys swerveSys;

    private Rotation2d targetHeading;

    private final boolean shouldMirrorHeading;

    private final ProfiledPIDController aimController;

    /**
     * Creates a new TurnToHeadingCmd.
     * 
     * <p>TurnToHeadingCmd is used to aim the drivebase to a specified heading. It overrides the driver's
     * rotation control as well as PathPlanner's rotation target. While it is running.
     * 
     * <p>The command never finishes, so it should be bound to a button with whileTrue() and called in
     * autonomous programs with a raceWith().
     * 
     * @param targetHeading The desired heading to turn the drivebase to.
     * @param shouldMirrorHeading True if the heading should be mirrored if on the red alliance.
     * this should almost always be set to true.
     * @param swerveSys The SwerveSys to modify.
     */
    public TurnToHeadingCmd(Rotation2d targetHeading, boolean shouldMirrorHeading, SwerveSys swerveSys) {
        this.swerveSys = swerveSys;

        this.targetHeading = targetHeading;

        this.shouldMirrorHeading = shouldMirrorHeading;

        aimController = new ProfiledPIDController(
                AutoConstants.autoAimkP, 0.0, AutoConstants.autoAimkD,
                new Constraints(
                    AutoConstants.autoAimTurnSpeedRadPerSec,
                    AutoConstants.autoAumTurnAccelRadPerSecSq));

        aimController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Creates a new TurnToHeadingCmd.
     * 
     * <p>TurnToHeadingCmd is used to aim the drivebase to a specified heading. It overrides the driver's
     * rotation control as well as PathPlanner's rotation target. While it is running.
     * 
     * <p>The command never finishes, so it should be bound to a button with whileTrue() and called in
     * autonomous programs with a raceWith().
     * 
     * @param targetHeading The desired heading to turn the drivebase to. This heading is mirrored when
     * on the red alliance. This can be overridden by using the other constructor.
     * @param swerveSys The SwerveSys to modify.
     */
    public TurnToHeadingCmd(Rotation2d targetHeading, SwerveSys swerveSys) {
        this.swerveSys = swerveSys;

        this.targetHeading = targetHeading;

        this.shouldMirrorHeading = true;

        aimController = new ProfiledPIDController(
                AutoConstants.autoAimkP, 0.0, AutoConstants.autoAimkD,
                new Constraints(
                    AutoConstants.autoAimTurnSpeedRadPerSec,
                    AutoConstants.autoAumTurnAccelRadPerSecSq));

        aimController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
        PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.of(targetHeading));
        
        Rotation2d mirroredHeading;
        if(shouldMirrorHeading && DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            mirroredHeading = Rotation2d.fromDegrees(180).minus(targetHeading);
        }
        else {
            mirroredHeading = targetHeading;
        }

    
        if(Math.abs(swerveSys.getHeading().getDegrees() - mirroredHeading.getDegrees()) > AutoConstants.autoAimToleranceDeg) {
            double aimRadPerSec = aimController.calculate(swerveSys.getHeading().getRadians(), mirroredHeading.getRadians());
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
