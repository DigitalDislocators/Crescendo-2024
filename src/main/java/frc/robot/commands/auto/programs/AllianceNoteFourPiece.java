// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.programs;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.FollowTrajectoryCmd;
import frc.robot.commands.auto.SetInitialPoseCmd;
import frc.robot.commands.automation.AutoAllHomeCmd;
import frc.robot.commands.automation.AutoGroundIntakeCmd;
import frc.robot.commands.automation.AutoSubwooferFireCmd;
import frc.robot.subsystems.FeederSys;
import frc.robot.subsystems.PivotSys;
import frc.robot.subsystems.RollersSys;
import frc.robot.subsystems.SwerveSys;

public class AllianceNoteFourPiece extends SequentialCommandGroup {
  public AllianceNoteFourPiece(SwerveSys swerveSys, FeederSys FeederSys, RollersSys RollersSys, PivotSys PivotSys) {
    addCommands(
      new SetInitialPoseCmd("StartPosToAllianceNoteOne", swerveSys),
      new AutoSubwooferFireCmd(FeederSys, RollersSys, PivotSys),
      new WaitCommand(0.25),
      new FollowTrajectoryCmd("StartPosToAllianceNoteOne", swerveSys)
        .alongWith(new AutoGroundIntakeCmd(PivotSys, FeederSys, RollersSys)),
      new FollowTrajectoryCmd("AllianceNoteOneToStartPos", swerveSys),
        alongWith(new AutoAllHomeCmd(PivotSys, FeederSys, RollersSys)),
      new AutoSubwooferFireCmd(FeederSys, RollersSys, PivotSys),
      new WaitCommand(0.25),
      new FollowTrajectoryCmd("StartPosToAllianceNoteTwo", swerveSys),
        alongWith(new AutoGroundIntakeCmd(PivotSys, FeederSys, RollersSys)),
      new FollowTrajectoryCmd("AllianceNoteTwoToStartPos", swerveSys),
        alongWith(new AutoAllHomeCmd(PivotSys, FeederSys, RollersSys)),
      new AutoSubwooferFireCmd(FeederSys, RollersSys, PivotSys),
      new WaitCommand(0.25),
      new FollowTrajectoryCmd("StartPosToAllianceNoteThree", swerveSys),
        alongWith(new AutoGroundIntakeCmd(PivotSys, FeederSys, RollersSys)),
      new FollowTrajectoryCmd("AllianceNoteThreeToStartPos", swerveSys),
        alongWith(new AutoAllHomeCmd(PivotSys, FeederSys, RollersSys)),
      new AutoSubwooferFireCmd(FeederSys, RollersSys, PivotSys)
    );
  }
}
