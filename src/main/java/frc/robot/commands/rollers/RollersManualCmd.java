package frc.robot.commands.rollers;

import frc.robot.subsystems.RollersSys;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class RollersManualCmd extends Command {

  private RollersSys rollers;

  private DoubleSupplier manualpower;

  public RollersManualCmd(DoubleSupplier manualPower, RollersSys rollers) {
    this.rollers = rollers;
    this.manualpower = manualPower;
    
    addRequirements(rollers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (manualpower.getAsDouble() == 0.0) {

    }
    else {
        rollers.setPower(manualpower.getAsDouble());
    }
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
