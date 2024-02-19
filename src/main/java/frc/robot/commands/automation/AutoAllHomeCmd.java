package frc.robot.commands.automation;

import frc.robot.commands.feeder.FeederStopCmd;
import frc.robot.commands.pivot.PivotHomePresetCmd;
import frc.robot.commands.rollers.RollersStopCmd;
import frc.robot.subsystems.PivotSys;
import frc.robot.subsystems.RollersSys;
import frc.robot.subsystems.FeederSys;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoAllHomeCmd extends SequentialCommandGroup {

  public AutoAllHomeCmd(PivotSys pivot, FeederSys feeder, RollersSys roller) {
    super(
      new PivotHomePresetCmd(pivot),
      new FeederStopCmd(feeder),
      new RollersStopCmd(roller)
    );
    
  }
}