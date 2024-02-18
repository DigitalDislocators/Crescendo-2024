package frc.robot.commands;

import frc.robot.subsystems.ServoSys;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShooterServoCmd extends Command {

  private ServoSys servo;

  private DoubleSupplier value;

  /**
   * Creates a new ExampleCommand.
   *
   * @param value The subsystem used by this command.
   * 
   */
  public ShooterServoCmd(DoubleSupplier value, ServoSys servo) {
    this.servo = servo;
    this.value = value;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(value);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //servo.set(value.getAsDouble());
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
