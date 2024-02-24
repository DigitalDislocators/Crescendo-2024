package frc.robot.commands.pivot;

import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSys;

import edu.wpi.first.wpilibj2.command.Command;

public class PivotHomePresetCmd extends Command {

  private final PivotSys pivot;

  public PivotHomePresetCmd(PivotSys pivot) {
    this.pivot = pivot;

    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivot.setPivotDeg(PivotConstants.homePresetDeg);
  }
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}