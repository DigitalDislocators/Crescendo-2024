package frc.robot.commands.drivetrain;

import java.util.Optional;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveSys;

public class TurnToHeadingCmd extends Command {

    private final SwerveSys swerveSys;

    private final Rotation2d targetHeading;

    private final PIDController aimController;

    public TurnToHeadingCmd(Rotation2d targetHeading, SwerveSys swerveSys) {
        this.swerveSys = swerveSys;

        this.targetHeading = targetHeading;

        aimController = new PIDController(AutoConstants.rotkP, 0.0, AutoConstants.rotkD);

        aimController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {

    }
    
    @Override
    public void execute() {
        PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.of(targetHeading));
        double aimRadPerSec = aimController.calculate(swerveSys.getHeading().getRadians(), targetHeading.getRadians());
    
        swerveSys.setOmegaOverrideRadPerSec(Optional.of(aimRadPerSec));
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
