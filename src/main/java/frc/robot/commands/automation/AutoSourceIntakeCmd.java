package frc.robot.commands.automation;

import frc.robot.commands.feeder.FeederInCmd;
import frc.robot.commands.pivot.PivotHomePresetCmd;
import frc.robot.commands.rollers.RollersIntakeCmd;
import frc.robot.subsystems.PivotSys;
import frc.robot.subsystems.RollersSys;
import frc.robot.subsystems.FeederSys;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoSourceIntakeCmd extends SequentialCommandGroup {

  public AutoSourceIntakeCmd(PivotSys pivot, FeederSys feeder, RollersSys roller) {
    super(
      new PivotHomePresetCmd(pivot),
      new RollersIntakeCmd(roller),
      new FeederInCmd(feeder)
    );
    
  }
}