// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSys;

public class SetInitialPoseCmd extends Command {
  
  private final SwerveSys swerveSys;

  private final PathPlannerPath firstPath;

  /** Creates a new SetInitialPoseCmd. */
  public SetInitialPoseCmd(String firstPathName, SwerveSys swerveSys) {
    firstPath = PathPlannerPath.fromPathFile(firstPathName);

    this.swerveSys = swerveSys;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if(DriverStation.getAlliance().get() == Alliance.Red) {
      // firstPath.flipPath();
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      // swerveSys.setTranslation(firstPath.getPreviewStartingHolonomicPose().getTranslation());
      // swerveSys.setHeading(firstPath.getPreviewStartingHolonomicPose().getRotation());

      
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