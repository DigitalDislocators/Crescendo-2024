package frc.robot.commands.automation;

import frc.robot.commands.feeder.FeederFeedCmd;
import frc.robot.commands.feeder.FeederStopCmd;
import frc.robot.commands.pivot.PivotAmpPresetCmd;
import frc.robot.commands.pivot.PivotHomePresetCmd;
import frc.robot.commands.rollers.RollersAmpFireCmd;
import frc.robot.commands.rollers.RollersStopCmd;
import frc.robot.commands.spacebar.SpacebarHomeCmd;
import frc.robot.commands.spacebar.SpacebarOutCmd;
import frc.robot.subsystems.RollersSys;
import frc.robot.subsystems.SpacebarSys;
import frc.robot.subsystems.FeederSys;
import frc.robot.subsystems.PivotSys;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoAmpFireCmd extends SequentialCommandGroup {

  public AutoAmpFireCmd(FeederSys feeder, RollersSys rollers, PivotSys pivot, SpacebarSys spacebar) {
    super(
      new PivotAmpPresetCmd(pivot),
      new RollersAmpFireCmd(rollers),
      new WaitCommand(0.175),
      new SpacebarOutCmd(spacebar),
      new WaitCommand(0.35),
      new FeederFeedCmd(feeder),
      new WaitCommand(1.25),
      new SpacebarHomeCmd(spacebar),
      new WaitCommand(0.8),
      new PivotHomePresetCmd(pivot),
      new RollersStopCmd(rollers),
      new FeederStopCmd(feeder)
    );
  }
}