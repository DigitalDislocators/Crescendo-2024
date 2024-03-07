// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.programs;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.auto.FollowPathCmd;
import frc.robot.subsystems.SwerveSys;

public class ExampleAuto extends SequentialCommandGroup {
  public ExampleAuto(SwerveSys swerveSys) {
    addCommands(
      // Again you can do it this way or keep the commands in their own files if you're more comfortable with that.
      Commands.runOnce(() -> swerveSys.setTranslation(new Translation2d(2.0, 2.0)), swerveSys),
      new FollowPathCmd("Example Path 1", swerveSys)
        .alongWith(new WaitUntilCommand(() -> swerveSys.getBlueSidePose().getX() > 2.0))
        .andThen(),
      new WaitCommand(2.0),
      new FollowPathCmd("Example Path 2", swerveSys)
    );
  }
}
