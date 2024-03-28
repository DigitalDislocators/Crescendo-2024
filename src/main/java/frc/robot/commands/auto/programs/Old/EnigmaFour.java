// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.programs.Old;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.commands.auto.FollowPathCmd;
// import frc.robot.commands.auto.SetInitialPoseCmd;
// import frc.robot.commands.automation.AutoAllHomeCmd;
// import frc.robot.commands.automation.AutoGroundIntakeCmd;
// import frc.robot.commands.automation.AutoSubwooferFireCmd;
// import frc.robot.subsystems.FeederSys;
// import frc.robot.subsystems.PivotSys;
// import frc.robot.subsystems.RollersSys;
// import frc.robot.subsystems.SpacebarSys;
// import frc.robot.subsystems.SwerveSys;

// public class EnigmaFour extends SequentialCommandGroup {
//   public EnigmaFour(SwerveSys swerveSys, FeederSys FeederSys, RollersSys RollersSys, PivotSys PivotSys, SpacebarSys SpacebarSys) {
//     addCommands(
//       // Again you can do it this way or keep the commands in their own files if you're more comfortable with that.
//       new SetInitialPoseCmd("EnigmaFourPathOne", swerveSys),
//       // Commands.runOnce(() -> swerveSys.setTranslation(new Translation2d(1.3, 5.55)), swerveSys),
//       new AutoSubwooferFireCmd(FeederSys, RollersSys, PivotSys),
//       new WaitCommand(0.05),
//       new FollowPathCmd("EnigmaFourPathOne", swerveSys)
//         .alongWith(new WaitUntilCommand(() -> swerveSys.getBlueSidePose().getX() > 4.0)
//         .andThen(new AutoGroundIntakeCmd(PivotSys, FeederSys, RollersSys, SpacebarSys))),
//       new FollowPathCmd("EnigmaFourPathTwo", swerveSys)
//         .alongWith(new AutoAllHomeCmd(PivotSys, FeederSys, RollersSys)
//           .andThen(new WaitUntilCommand(() -> swerveSys.getBlueSidePose().getX() < AutoConstants.subwooferShotThreshold))
//           .andThen(new AutoSubwooferFireCmd(FeederSys, RollersSys, PivotSys))),
//       new FollowPathCmd("EnigmaFourPathThree", swerveSys)
//         .alongWith(new AutoGroundIntakeCmd(PivotSys, FeederSys, RollersSys, SpacebarSys)),
//       new FollowPathCmd("EnigmaFourPathFour", swerveSys)
//         .alongWith(new AutoAllHomeCmd(PivotSys, FeederSys, RollersSys)
//           .andThen(new WaitUntilCommand(() -> swerveSys.getBlueSidePose().getX() < AutoConstants.subwooferShotThreshold))
//           .andThen(new AutoSubwooferFireCmd(FeederSys, RollersSys, PivotSys))),
//       new FollowPathCmd("EnigmaFourPathFive", swerveSys)
//         .alongWith(new AutoGroundIntakeCmd(PivotSys, FeederSys, RollersSys, SpacebarSys)),
//       new FollowPathCmd("EnigmaFourPathSix", swerveSys)
//         .alongWith(new AutoAllHomeCmd(PivotSys, FeederSys, RollersSys)
//           .andThen(new WaitUntilCommand(() -> swerveSys.getBlueSidePose().getX() < AutoConstants.subwooferShotThreshold))
//           .andThen(new AutoSubwooferFireCmd(FeederSys, RollersSys, PivotSys))),
//       new WaitCommand(0.75)
//     );
//   }
// }
