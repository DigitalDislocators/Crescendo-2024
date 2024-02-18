package frc.robot.commands;

import frc.robot.subsystems.RollerSys;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ManualRollersCmd extends Command {

  private RollerSys rollers;

  private DoubleSupplier manualpower;

  /**
   * Creates a new ExampleCommand.
   *
   * @param rollers The subsystem used by this command.
   * 
   */
  public ManualRollersCmd(DoubleSupplier manualpower, RollerSys rollers) {
    this.rollers = rollers;
    this.manualpower = manualpower;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rollers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rollers.setManualPower(manualpower.getAsDouble());
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
