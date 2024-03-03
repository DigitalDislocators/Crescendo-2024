// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.programs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
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

public class MidlineNoteThreePiece extends SequentialCommandGroup {
  public MidlineNoteThreePiece(SwerveSys swerveSys, FeederSys FeederSys, RollersSys RollersSys, PivotSys PivotSys) {
    addCommands(
      new SetInitialPoseCmd("OffsetSubwooferPosToMidlineNoteFive", swerveSys),
      // Commands.runOnce(() -> swerveSys.setHeading(new Rotation2d(60)), swerveSys),
      new AutoSubwooferFireCmd(FeederSys, RollersSys, PivotSys),
      new WaitCommand(0.1),
      new FollowTrajectoryCmd("OffsetSubwooferPosToMidlineNoteFive", swerveSys)
        .alongWith(new WaitUntilCommand(() -> swerveSys.getBlueSidePose().getX() > 4.0)
        .andThen(new AutoGroundIntakeCmd(PivotSys, FeederSys, RollersSys))),
      new FollowTrajectoryCmd("MidlineNoteFiveToOffsetSubwooferPos", swerveSys)
        .alongWith(new AutoAllHomeCmd(PivotSys, FeederSys, RollersSys)),
      new AutoSubwooferFireCmd(FeederSys, RollersSys, PivotSys),
      new WaitCommand(0.1),
      new FollowTrajectoryCmd("OffsetSubwooferPosToMidlineNoteFour", swerveSys)
      .alongWith(new WaitUntilCommand(() -> swerveSys.getBlueSidePose().getX() > 4.0)
      .andThen(new AutoGroundIntakeCmd(PivotSys, FeederSys, RollersSys))),
      new WaitCommand(0.2),
      new FollowTrajectoryCmd("MidlineNoteFourToOffsetSubwooferPos", swerveSys)
        .alongWith(new AutoAllHomeCmd(PivotSys, FeederSys, RollersSys)),
      // new AutoSubwooferFireCmd(FeederSys, RollersSys, PivotSys),
      new WaitCommand(0.5)
    );
  }
}
