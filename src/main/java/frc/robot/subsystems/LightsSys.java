// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightsConstants;
import frc.robot.util.led.LEDStrip;
import frc.robot.util.led.LEDStripArray;

public class LightsSys extends SubsystemBase {

	public final LEDStrip exampleStrip1;
	public final LEDStrip exampleStrip2;
	public final LEDStrip exampleStrip3;

	private final LEDStripArray ledStripArray;

	/** Creates a new LightsSys. */
	public LightsSys() {
		exampleStrip1 = new LEDStrip(78);
		exampleStrip2 = new LEDStrip(128);
		exampleStrip3 = new LEDStrip(78);

		exampleStrip3.setReversed(true);

		ledStripArray = new LEDStripArray(9, exampleStrip1, exampleStrip2, exampleStrip3);

		ledStripArray.setBrightnesses(LightsConstants.brightnessPercentage);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void setColor(Color color) {
		ledStripArray.setColors(color);
	}

	public void setValue(double value) {
		ledStripArray.setValues(value);
	}
}
