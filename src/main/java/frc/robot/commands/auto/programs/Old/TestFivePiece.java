// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.programs.Old;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// import frc.robot.commands.auto.FollowPathCmd;
// import frc.robot.commands.auto.SetInitialPoseCmd;
// import frc.robot.commands.automation.AutoAllHomeCmd;
// import frc.robot.commands.automation.AutoGroundIntakeCmd;
// import frc.robot.commands.automation.AutoSetPivotToSpeakerCmd;
// import frc.robot.commands.automation.AutoSubwooferFireCmd;
// import frc.robot.commands.drivetrain.AimToSpeakerCmd;
// import frc.robot.commands.feeder.FeederFeedCmd;
// import frc.robot.commands.rollers.RollersFireCmd;
// import frc.robot.subsystems.FeederSys;
// import frc.robot.subsystems.PivotSys;
// import frc.robot.subsystems.RollersSys;
// import frc.robot.subsystems.SpacebarSys;
// import frc.robot.subsystems.SwerveSys;

// public class TestFivePiece extends SequentialCommandGroup {
//   public TestFivePiece(SwerveSys swerveSys, FeederSys feederSys, RollersSys rollersSys, PivotSys pivotSys, SpacebarSys SpacebarSys) {
//     addCommands(
//       new SetInitialPoseCmd("SubwooferPosToMidlineNoteThree", swerveSys),
//       new AutoSubwooferFireCmd(feederSys, rollersSys, pivotSys),
//       new FollowPathCmd("SubwooferPosToMidlineNoteThree", swerveSys)
//         .alongWith(new WaitUntilCommand(() -> swerveSys.getBlueSidePose().getX() > 4.0)
//         .andThen(new AutoGroundIntakeCmd(pivotSys, feederSys, rollersSys, SpacebarSys))),
//       new FollowPathCmd("MidlineNoteThreeToAllianceNoteTwo", swerveSys)
//         .alongWith(new AutoAllHomeCmd(pivotSys, feederSys, rollersSys)
//           .andThen(new RollersFireCmd(rollersSys))
//           .andThen(new AutoSetPivotToSpeakerCmd(swerveSys, pivotSys)
//             .raceWith(new WaitUntilCommand(() -> swerveSys.getBlueSidePose().getX() < 5.5)
//               .andThen(new AimToSpeakerCmd(swerveSys)
//                 .raceWith(new WaitUntilCommand(() -> swerveSys.getBlueSidePose().getX() < 4.5)
//                   .andThen(new FeederFeedCmd(feederSys))
//                   .andThen(new WaitCommand(0.5))))))),
//       new AutoGroundIntakeCmd(pivotSys, feederSys, rollersSys, SpacebarSys),
//       new FollowPathCmd("AllianceNoteTwoToAllianceNoteOne", swerveSys)
//         .alongWith(new WaitCommand(1.25)
//           .andThen(new AutoSetPivotToSpeakerCmd(swerveSys, pivotSys)
//             .raceWith(new AimToSpeakerCmd(swerveSys)
//               .raceWith(new RollersFireCmd(rollersSys)
//                 .andThen(new WaitCommand(0.5))
//                 .andThen(new FeederFeedCmd(feederSys))
//                 .andThen(new WaitCommand(1.0)))))
//         .andThen(new AutoGroundIntakeCmd(pivotSys, feederSys, rollersSys, SpacebarSys))),
//       new FollowPathCmd("AllianceNoteOneToAllianceNoteThree", swerveSys)
//         .alongWith(new WaitCommand(0.75)
//           .andThen(new RollersFireCmd(rollersSys))
//           .andThen(new AimToSpeakerCmd(swerveSys)
//             .raceWith(new AutoSetPivotToSpeakerCmd(swerveSys, pivotSys)
//               .raceWith(new WaitCommand(0.5)
//                 .andThen(new FeederFeedCmd(feederSys)
//                 .andThen(new WaitCommand(0.75))))))
//           .andThen(new AutoGroundIntakeCmd(pivotSys, feederSys, rollersSys, SpacebarSys))),
//       new FollowPathCmd("AllianceNoteThreeToSubwooferPos2", swerveSys)
//         .alongWith(new AutoAllHomeCmd(pivotSys, feederSys, rollersSys)
//           .andThen(new RollersFireCmd(rollersSys))
//           .andThen(new WaitUntilCommand(() -> swerveSys.getBlueSidePose().getX() < 2.0))
//           .andThen(new FeederFeedCmd(feederSys)))
//     );
//   }
// }
