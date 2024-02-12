package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControllerConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class ClimberSys extends SubsystemBase {
  private final CANSparkMax leftClimber = new CANSparkMax(CANDevices.leftClimbMtrId, MotorType.kBrushless);
  private final CANSparkMax rightClimber = new CANSparkMax(CANDevices.rightClimbMtrId, MotorType.kBrushless);
  
  private final SlewRateLimiter climberRateLimiter = new SlewRateLimiter(ClimberConstants.climberRateLimit);

  public ClimberSys() {
    //leftClimber.enableVoltageCompensation(DriveConstants.NOMINAL_VOLTAGE);

    leftClimber.setIdleMode(IdleMode.kBrake);
    rightClimber.setIdleMode(IdleMode.kBrake);

    leftClimber.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.climberForwardLimit);
    rightClimber.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.climberReverseLimit);

    leftClimber.enableSoftLimit(SoftLimitDirection.kForward, true);
    rightClimber.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  public void setClimberSpeed(double speed) {
    leftClimber.set(climberRateLimiter.calculate(speed) * ClimberConstants.climberSpeedFactor);
  }

  public double getClimberPosition() {
    return leftClimber.getEncoder().getPosition();
  }

  public boolean getUpperLimit() {
    return Math.abs(getClimberPosition() - leftClimber.getSoftLimit(SoftLimitDirection.kForward)) < ClimberConstants.limitVariability;
  }

  public boolean getLowerLimit() {
    return Math.abs(getClimberPosition() - rightClimber.getSoftLimit(SoftLimitDirection.kReverse)) < ClimberConstants.limitVariability;
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}