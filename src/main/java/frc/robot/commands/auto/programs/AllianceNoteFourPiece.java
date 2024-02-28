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
      // Again you can do it this way or keep the commands in their own files if you're more comfortable with that.
      Commands.runOnce(() -> swerveSys.setTranslation(new Translation2d(1.25, 5.55)), swerveSys),
       new AutoSubwooferFireCmd(FeederSys, RollersSys, PivotSys),
      new WaitCommand(0.25),
      new FollowTrajectoryCmd("StartPosToAllianceNoteOne", swerveSys),
        alongWith(new AutoGroundIntakeCmd(PivotSys, FeederSys, RollersSys)),
      new FollowTrajectoryCmd("AllianceNoteOneToStartPos", swerveSys),
        alongWith(new AutoAllHomeCmd(PivotSys, FeederSys, RollersSys)),
      new AutoSubwooferFireCmd(FeederSys, RollersSys, PivotSys),
      new WaitCommand(0.1),
      new FollowTrajectoryCmd("SubwooferPosToAllianceNoteThree", swerveSys)
        .alongWith(new AutoGroundIntakeCmd(PivotSys, FeederSys, RollersSys)),
      new FollowTrajectoryCmd("AllianceNoteThreeToSubwooferPos", swerveSys)
        .alongWith(new AutoAllHomeCmd(PivotSys, FeederSys, RollersSys)),
      new AutoSubwooferFireCmd(FeederSys, RollersSys, PivotSys),
      new WaitCommand(0.75)
    );
  }
}
