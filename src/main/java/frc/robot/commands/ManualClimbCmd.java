package frc.robot.commands;

import frc.robot.subsystems.ClimberSys;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ManualClimbCmd extends Command {

  private ClimberSys climber;

  private DoubleSupplier climberPower;

  /**
   * Creates a new ExampleCommand.
   *
   * @param  The subsystem used by this command.
   * 
   */
  public ManualClimbCmd(DoubleSupplier climberPower, ClimberSys climber) {
    this.climber = climber;
    this.climberPower = climberPower;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setClimberSpeed(climberPower.getAsDouble());
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