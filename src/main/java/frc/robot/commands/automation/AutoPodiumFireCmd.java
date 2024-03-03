package frc.robot.commands.automation;

import frc.robot.commands.feeder.FeederFeedCmd;
import frc.robot.commands.feeder.FeederStopCmd;
import frc.robot.commands.pivot.PivotHomePresetCmd;
import frc.robot.commands.pivot.PivotPodiumPresetCmd;
import frc.robot.commands.rollers.RollersFireCmd;
import frc.robot.commands.rollers.RollersStopCmd;
import frc.robot.subsystems.RollersSys;
import frc.robot.subsystems.FeederSys;
import frc.robot.subsystems.PivotSys;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoPodiumFireCmd extends SequentialCommandGroup {

  public AutoPodiumFireCmd(FeederSys feeder, RollersSys rollers, PivotSys pivot) {
    super(
      new RollersFireCmd(rollers),
      new PivotPodiumPresetCmd(pivot),
      new WaitCommand(1.75),
      new FeederFeedCmd(feeder),
      new WaitCommand(0.75),
      new PivotHomePresetCmd(pivot),
      new RollersStopCmd(rollers),
      new FeederStopCmd(feeder)
    );
  }
}