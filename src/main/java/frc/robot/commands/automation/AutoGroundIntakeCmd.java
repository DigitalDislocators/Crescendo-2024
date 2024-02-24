package frc.robot.commands.automation;

import frc.robot.commands.feeder.FeederInCmd;
import frc.robot.commands.pivot.PivotGroundPresetCmd;
import frc.robot.commands.rollers.RollersIntakeCmd;
import frc.robot.subsystems.PivotSys;
import frc.robot.subsystems.RollersSys;
import frc.robot.subsystems.FeederSys;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoGroundIntakeCmd extends SequentialCommandGroup {

  public AutoGroundIntakeCmd(PivotSys pivot, FeederSys feeder, RollersSys roller) {
    super(
      new PivotGroundPresetCmd(pivot),
      new FeederInCmd(feeder),
      new RollersIntakeCmd(roller)
    );
    
  }
}