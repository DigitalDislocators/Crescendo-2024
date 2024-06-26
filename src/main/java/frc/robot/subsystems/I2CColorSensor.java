// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;

public class I2CColorSensor extends SubsystemBase {

  private final ColorSensorV3 sensor;
  
  private final Timer timer;

  private boolean hasNote = false;

  /** Creates a new I2CColorSensor. */
  public I2CColorSensor() {
    sensor = new ColorSensorV3(Port.kOnboard);

    timer = new Timer();

    timer.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(timer.hasElapsed(RollerConstants.sensorTimeIntervalSecs)) {
      hasNote = sensor.getProximity() > RollerConstants.sensorHasNoteADCThreshold;
      timer.restart();
    }
  }

  public boolean hasNote() {
      return hasNote;
  }
}
