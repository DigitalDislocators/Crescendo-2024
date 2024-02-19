package frc.robot.commands.automation;

import frc.robot.commands.WaitCmd;
import frc.robot.commands.feeder.FeederFeedCmd;
import frc.robot.commands.feeder.FeederStopCmd;
import frc.robot.commands.rollers.RollersShootCmd;
import frc.robot.commands.rollers.RollersStopCmd;
import frc.robot.subsystems.RollersSys;
import frc.robot.subsystems.FeederSys;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoShootCmd extends SequentialCommandGroup {

  public AutoShootCmd(FeederSys feeder, RollersSys rollers) {
    super(
      new RollersShootCmd(rollers),
      new WaitCmd(0.1),
      new FeederFeedCmd(feeder),
      new WaitCmd(0.5),
      new RollersStopCmd(rollers),
      new FeederStopCmd(feeder)
    );
  }
}