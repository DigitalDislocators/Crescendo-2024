// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.lights;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LightsConstants;
import frc.robot.subsystems.LightsSys;
import frc.robot.util.led.LEDStrip;
import frc.robot.util.led.LEDStrip.TranslateDirection;

public class PartyModeCmd extends Command {

	private final LightsSys lightsSys;

	private int startingHue = 0;

	/** Creates a new LightsDefaultCmd. */
	public PartyModeCmd(LightsSys lightsSys) {
		this.lightsSys = lightsSys;

		addRequirements(lightsSys);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		for(LEDStrip strip : new LEDStrip[] {lightsSys.exampleStrip1, lightsSys.exampleStrip2, lightsSys.exampleStrip3}) {
			strip.setValue(1.0);
			int hue = startingHue;
			for(int i = 0; i < strip.getLength(); i++) {
				strip.setColor(Color.fromHSV(hue, 255, 255), i);
				if(hue >= 255) {
					hue = 0;
				}
				else {
					hue += LightsConstants.partyModeHueIncrement;
				}
			}
		}
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if(startingHue >= 180) {
			startingHue = 0;
		}
		else {
			startingHue += LightsConstants.partyModeHueIncrement;
		}
		
		for(LEDStrip strip : new LEDStrip[] {lightsSys.exampleStrip1, lightsSys.exampleStrip2, lightsSys.exampleStrip3}) {
			strip.translateColors(TranslateDirection.FORWARD, Color.fromHSV(startingHue, 255, 255));
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
  	}
}
