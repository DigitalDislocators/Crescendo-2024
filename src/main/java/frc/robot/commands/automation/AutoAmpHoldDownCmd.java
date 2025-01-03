package frc.robot.commands.automation;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.pivot.PivotHomePresetCmd;
import frc.robot.commands.spacebar.SpacebarHomeCmd;
import frc.robot.subsystems.PivotSys;
import frc.robot.subsystems.SpacebarSys;

public class AutoAmpHoldDownCmd extends SequentialCommandGroup {
  public AutoAmpHoldDownCmd(PivotSys pivot, SpacebarSys spacebar) {
    super(
      new SpacebarHomeCmd(spacebar),
      new WaitCommand(0.775),
      new PivotHomePresetCmd(pivot)
    );
  }

  public AutoAmpHoldDownCmd(SpacebarSys spacebarSys) {
    //TODO Auto-generated constructor stub
  }
} 
