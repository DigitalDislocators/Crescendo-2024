package frc.robot.commands.automation;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.pivot.PivotAmpPresetCmd;
import frc.robot.commands.spacebar.SpacebarOutCmd;
import frc.robot.subsystems.PivotSys;
import frc.robot.subsystems.SpacebarSys;


public class AutoAmpHoldUpCmd extends SequentialCommandGroup {
  public AutoAmpHoldUpCmd(PivotSys pivot, SpacebarSys spacebar) {
    super(
      new PivotAmpPresetCmd(pivot),
      new WaitCommand(0.175),
      new SpacebarOutCmd(spacebar)
    );
  }

public AutoAmpHoldUpCmd(SpacebarSys spacebarSys) {
    //TODO Auto-generated constructor stub
}
}
