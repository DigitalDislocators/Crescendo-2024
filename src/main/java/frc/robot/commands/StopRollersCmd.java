package frc.robot.commands;

import frc.robot.subsystems.RollerSys;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class StopRollersCmd extends Command {

  private final RollerSys rollers;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public StopRollersCmd(RollerSys rollers) {
    this.rollers = rollers;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rollers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

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