// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.lights;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LightsConstants;
import frc.robot.subsystems.LightsSys;

public class LightsDefaultCmd extends Command {

	private final LightsSys lightsSys;

	/** Creates a new LightsDefaultCmd. */
	public LightsDefaultCmd(LightsSys lightsSys) {
		this.lightsSys = lightsSys;

		addRequirements(lightsSys);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
			lightsSys.setColor(LightsConstants.redAllianceColor);
		}
		else {
			lightsSys.setColor(LightsConstants.blueAllianceColor);
		}
		lightsSys.setValue(1.0);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
  	}
}
