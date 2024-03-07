package frc.robot.commands.pivot;

import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSys;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;

public class PivotManualCmd extends Command {
  
  SlewRateLimiter filter = new SlewRateLimiter(PivotConstants.maxManualDegPerSecSq);

  private final PivotSys pivot;

  private final DoubleSupplier power; 

  public PivotManualCmd(DoubleSupplier power, PivotSys pivot) {
    this.pivot = pivot;
    this.power = power;

    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivot.setManualPower(filter.calculate(power.getAsDouble() * PivotConstants.maxManualPower));
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