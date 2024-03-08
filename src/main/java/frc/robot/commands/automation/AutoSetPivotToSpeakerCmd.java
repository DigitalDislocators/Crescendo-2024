// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.PivotSys;
import frc.robot.subsystems.SwerveSys;

public class AutoSetPivotToSpeakerCmd extends Command {

	private final SwerveSys swerveSys;
	private final PivotSys pivotSys;

	private final Translation2d targetTranslation;

	/** Creates a new AutoSetPivotCmd. */
	public AutoSetPivotToSpeakerCmd(SwerveSys swerveSys, PivotSys pivotSys) {
		this.swerveSys = swerveSys;
		this.pivotSys = pivotSys;

		if(DriverStation.getAlliance().get() == Alliance.Red) {
			targetTranslation = FieldConstants.redAllianceSpeakerPose;
		}
		else {
			targetTranslation = FieldConstants.blueAllianceSpeakerPose;
		}

		addRequirements(pivotSys);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
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
        // Translation2d extrapolatedTargetOffset = targetTranslation.minus(extrapolatedTranslation);

        double extrapolatedDistanceToTargetMeters = extrapolatedTranslation.getDistance(targetTranslation);
        SmartDashboard.putNumber("distance from speaker meters", extrapolatedDistanceToTargetMeters);

		double targetAngleDeg = PivotConstants.pivotDegSpeakerShotInterpolator.get(extrapolatedDistanceToTargetMeters);

		pivotSys.setTargetDeg(targetAngleDeg);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
	}
}
