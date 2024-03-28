package frc.robot.commands.spacebar;

import frc.robot.Constants.SpacebarConstants;
import frc.robot.subsystems.SpacebarSys;
import edu.wpi.first.wpilibj2.command.Command;

public class SpacebarHomeCmd extends Command {

  private final SpacebarSys spacebar;

  public SpacebarHomeCmd(SpacebarSys spacebar) {
    this.spacebar = spacebar;

    addRequirements(spacebar);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    spacebar.setTargetDeg(SpacebarConstants.spacebarHomeDeg);
    // spacebar.setPower(-1.0);
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