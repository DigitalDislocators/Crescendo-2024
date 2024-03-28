// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.programs;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.auto.FollowPathCmd;
import frc.robot.commands.auto.SetInitialPoseCmd;
import frc.robot.commands.automation.AutoAllHomeCmd;
import frc.robot.commands.automation.AutoGroundIntakeCmd;
import frc.robot.commands.automation.AutoSubwooferFireCmd;
import frc.robot.subsystems.FeederSys;
import frc.robot.subsystems.PivotSys;
import frc.robot.subsystems.RollersSys;
import frc.robot.subsystems.SpacebarSys;
import frc.robot.subsystems.SwerveSys;

public class SmashySmash extends SequentialCommandGroup {
  public SmashySmash(SwerveSys swerveSys, FeederSys FeederSys, RollersSys RollersSys, PivotSys PivotSys, SpacebarSys SpacebarSys) {
    addCommands(
      new SetInitialPoseCmd("SecondPickThreePathOne", swerveSys),
      new AutoSubwooferFireCmd(FeederSys, RollersSys, PivotSys),
      new WaitCommand(0.08),
      new FollowPathCmd("SecondPickThreePathOne", swerveSys)
        .alongWith(new WaitUntilCommand(() -> swerveSys.getBlueSidePose().getX() > 5.5)
          .andThen(new AutoGroundIntakeCmd(PivotSys, FeederSys, RollersSys, SpacebarSys))),
      new FollowPathCmd("SmashySmashPathTwo", swerveSys)
        .alongWith(new AutoAllHomeCmd(PivotSys, FeederSys, RollersSys)),
      new WaitCommand(0.5)
    );
  }
}
