package frc.robot.commands.rollers;

import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.RollersSys;

import edu.wpi.first.wpilibj2.command.Command;

public class RollersAmpFireCmd extends Command {

  private final RollersSys rollers;

  public RollersAmpFireCmd(RollersSys rollers) {
    this.rollers = rollers;
    
    addRequirements(rollers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rollers.setRPM(RollerConstants.ampRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}