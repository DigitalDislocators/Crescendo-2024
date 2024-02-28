package frc.robot.commands.automation;

import frc.robot.commands.feeder.FeederFeedCmd;
import frc.robot.commands.feeder.FeederStopCmd;
import frc.robot.commands.rollers.RollersFireCmd;
import frc.robot.commands.rollers.RollersStopCmd;
import frc.robot.subsystems.RollersSys;
import frc.robot.subsystems.FeederSys;
import frc.robot.subsystems.PivotSys;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoSubwooferFireCmd extends SequentialCommandGroup {

  public AutoSubwooferFireCmd(FeederSys feeder, RollersSys rollers, PivotSys pivot) {
    super(
      new RollersFireCmd(rollers),
      new WaitCommand(0.05),
      new FeederFeedCmd(feeder),
      new WaitCommand(0.6),
      new RollersStopCmd(rollers),
      new FeederStopCmd(feeder)
    );
  }
}