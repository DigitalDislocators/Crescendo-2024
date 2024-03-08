package frc.robot.commands.pivot;

import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSys;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;

public class PivotManualCmd extends Command {
  
  private final SlewRateLimiter slewRateLimiter;

  private final PivotSys pivot;

  private final DoubleSupplier input; 

  public PivotManualCmd(DoubleSupplier input, PivotSys pivot) {
    this.pivot = pivot;
    this.input = input;

    slewRateLimiter = new SlewRateLimiter(PivotConstants.maxManualDegPerSecSq);

    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivot.setManualSpeedDegPerSec(slewRateLimiter.calculate(input.getAsDouble() * PivotConstants.maxManualDegPerSec));
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