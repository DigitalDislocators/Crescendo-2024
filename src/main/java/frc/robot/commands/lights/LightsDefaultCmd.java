// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.lights;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LightsConstants;
import frc.robot.subsystems.LightsSys;

public class LightsDefaultCmd extends Command {

	private final LightsSys lightsSys;

	private final BooleanSupplier hasNoteSupplier;

	private boolean prevHasNote = true;

	/** Creates a new LightsDefaultCmd. */
	public LightsDefaultCmd(LightsSys lightsSys, BooleanSupplier hasNoteSupplier) {
		this.lightsSys = lightsSys;
		this.hasNoteSupplier = hasNoteSupplier;

		addRequirements(lightsSys);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		/*if(!prevHasNote && hasNoteSupplier.getAsBoolean()) {
			lightsSys.setColor(LightsConstants.hasNoteColor);
		}
		else if(DriverStation.getAlliance().isPresent() && prevHasNote && !hasNoteSupplier.getAsBoolean()) {*/
			if(DriverStation.getAlliance().get() == Alliance.Red) {
				lightsSys.setColor(LightsConstants.redAllianceColor);
			}
			else if(DriverStation.getAlliance().get() == Alliance.Blue) {
				lightsSys.setColor(LightsConstants.blueAllianceColor);
			}
			else {
				lightsSys.setColor(LightsConstants.noAllianceColor);
			}
		// }

		lightsSys.setValue(1.0);

		// prevHasNote = hasNoteSupplier.getAsBoolean();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
  	}

	@Override
	public boolean runsWhenDisabled() {
		return true;
	}
}
