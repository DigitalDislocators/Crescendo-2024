package frc.robot.commands;

import frc.robot.subsystems.PivotSys;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class PivotManualCmd extends Command {

  private final PivotSys pivot;

  private final DoubleSupplier power;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PivotManualCmd(DoubleSupplier power, PivotSys pivot) {
    this.pivot = pivot;
    this.power = power;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //pivot.manualRollerControl(power.getAsDouble());

    pivot.setPivotDeg(power.getAsDouble());
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