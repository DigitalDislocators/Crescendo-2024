// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.programs;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.auto.FollowTrajectoryCmd;
import frc.robot.commands.auto.SetInitialPoseCmd;
import frc.robot.commands.automation.AutoAllHomeCmd;
import frc.robot.commands.automation.AutoGroundIntakeCmd;
import frc.robot.commands.automation.AutoSubwooferFireCmd;
import frc.robot.subsystems.FeederSys;
import frc.robot.subsystems.PivotSys;
import frc.robot.subsystems.RollersSys;
import frc.robot.subsystems.SwerveSys;

public class PiHiThreePiece extends SequentialCommandGroup {
  public PiHiThreePiece(SwerveSys swerveSys, FeederSys FeederSys, RollersSys RollersSys, PivotSys PivotSys) {
    addCommands(
      // Again you can do it this way or keep the commands in their own files if you're more comfortable with that.
      new SetInitialPoseCmd("OffsetSubwooferPosToMidlineNoteFive", swerveSys),
      // Commands.runOnce(() -> swerveSys.setPoseFromPathStart("SubwooferPosToAllianceNoteOne"), swerveSys),
      new AutoSubwooferFireCmd(FeederSys, RollersSys, PivotSys),
      new WaitCommand(0.05),
      new FollowTrajectoryCmd("OffsetSubwooferPosToMidlineNoteFive", swerveSys)
        .alongWith(new WaitUntilCommand(() -> swerveSys.getBlueSidePose().getX() > 4.0)
        .andThen(new AutoGroundIntakeCmd(PivotSys, FeederSys, RollersSys))),
      new FollowTrajectoryCmd("MidlineNoteFiveToSubwooferPos", swerveSys)
        .alongWith(new AutoAllHomeCmd(PivotSys, FeederSys, RollersSys)
          .andThen(new WaitUntilCommand(() -> swerveSys.getBlueSidePose().getX() < 1.9))
          .andThen(new AutoSubwooferFireCmd(FeederSys, RollersSys, PivotSys))),
      new FollowTrajectoryCmd("SubwooferPosToAllianceNoteThree", swerveSys)
        .alongWith(new AutoGroundIntakeCmd(PivotSys, FeederSys, RollersSys)),
      new FollowTrajectoryCmd("AllianceNoteThreeToSubwooferPos", swerveSys)
        .alongWith(new AutoAllHomeCmd(PivotSys, FeederSys, RollersSys)
          .andThen(new WaitUntilCommand(() -> swerveSys.getBlueSidePose().getX() < 1.8))
          .andThen(new AutoSubwooferFireCmd(FeederSys, RollersSys, PivotSys)))
    );
  }
}
