// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.programs;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AutoConstants;
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

public class AllianceFour extends SequentialCommandGroup {
  public AllianceFour(SwerveSys swerveSys, FeederSys FeederSys, RollersSys RollersSys, PivotSys PivotSys, SpacebarSys SpacebarSys) {
    addCommands(
      // Again you can do it this way or keep the commands in their own files if you're more comfortable with that.
      new SetInitialPoseCmd("AllianceFivePathFive", swerveSys),
      // Commands.runOnce(() -> swerveSys.setTranslation(new Translation2d(1.3, 5.55)), swerveSys),
      // new AutoSubwooferFireCmd(FeederSys, RollersSys, PivotSys),
      // new WaitCommand(2.0),

      new AutoSubwooferFireCmd(FeederSys, RollersSys, PivotSys)
        .alongWith(new WaitCommand(0.01))
        .andThen(new FollowPathCmd("AllianceFivePathFive", swerveSys)
        .alongWith(new AutoGroundIntakeCmd(PivotSys, FeederSys, RollersSys, SpacebarSys))),
      new FollowPathCmd("AllianceFivePathSix", swerveSys)
        .alongWith(new AutoAllHomeCmd(PivotSys, FeederSys, RollersSys)
          .andThen(new WaitUntilCommand(() -> swerveSys.getBlueSidePose().getX() < AutoConstants.subwooferShotThreshold))
          .andThen(new AutoSubwooferFireCmd(FeederSys, RollersSys, PivotSys))),

      new FollowPathCmd("AllianceFivePathOne", swerveSys)
        .alongWith(new WaitUntilCommand(() -> swerveSys.getBlueSidePose().getX() > 4.5)
          .andThen(new AutoGroundIntakeCmd(PivotSys, FeederSys, RollersSys, SpacebarSys))),
      new FollowPathCmd("AllianceFivePathTwo", swerveSys)
        .alongWith(new AutoAllHomeCmd(PivotSys, FeederSys, RollersSys)
          .andThen(new WaitUntilCommand(() -> swerveSys.getBlueSidePose().getX() < AutoConstants.subwooferShotThreshold))
          .andThen(new AutoSubwooferFireCmd(FeederSys, RollersSys, PivotSys))),

      new FollowPathCmd("AllianceFivePathSeven", swerveSys)
        .alongWith(new AutoGroundIntakeCmd(PivotSys, FeederSys, RollersSys, SpacebarSys)),
      new FollowPathCmd("AllianceFivePathEight", swerveSys)
        .alongWith(new AutoAllHomeCmd(PivotSys, FeederSys, RollersSys)
          .andThen(new WaitUntilCommand(() -> swerveSys.getBlueSidePose().getX() < AutoConstants.subwooferShotThreshold))
          .andThen(new AutoSubwooferFireCmd(FeederSys, RollersSys, PivotSys))),

      // new FollowPathCmd("AllianceFivePathThree", swerveSys)
      //   .alongWith(new AutoGroundIntakeCmd(PivotSys, FeederSys, RollersSys, SpacebarSys)),
      // new FollowPathCmd("AllianceFivePathFour", swerveSys)
      //   .alongWith(new AutoAllHomeCmd(PivotSys, FeederSys, RollersSys)
      //     .andThen(new WaitUntilCommand(() -> swerveSys.getBlueSidePose().getX() < AutoConstants.subwooferShotThreshold))
      //     .andThen(new AutoSubwooferFireCmd(FeederSys, RollersSys, PivotSys))),

      new FollowPathCmd("AllianceFiveSPRINT", swerveSys)
        .alongWith(new WaitUntilCommand(() -> swerveSys.getBlueSidePose().getX() > 4.5)
          .andThen(new AutoGroundIntakeCmd(PivotSys, FeederSys, RollersSys, SpacebarSys))),
      new AutoAllHomeCmd(PivotSys, FeederSys, RollersSys),
      new WaitCommand(0.75)
    );
  }
}
