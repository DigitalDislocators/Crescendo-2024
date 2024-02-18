package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.ClimberConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

//import edu.wpi.first.math.filter.SlewRateLimiter;

public class ClimberSys extends SubsystemBase {
  private final CANSparkMax leaderClimber = new CANSparkMax(CANDevices.leaderClimbMtrId, MotorType.kBrushless);
  private final CANSparkMax followerClimber = new CANSparkMax(CANDevices.followerClimbMtrId, MotorType.kBrushless);
  
  // private final SlewRateLimiter climberRateLimiter = new SlewRateLimiter(ClimberConstants.climberRateLimit);

  public ClimberSys() {
    //leftClimber.enableVoltageCompensation(DriveConstants.NOMINAL_VOLTAGE);

    leaderClimber.setIdleMode(IdleMode.kBrake);
    followerClimber.setIdleMode(IdleMode.kBrake);

    leaderClimber.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.climberForwardLimit);
    leaderClimber.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.climberReverseLimit);

    leaderClimber.enableSoftLimit(SoftLimitDirection.kForward, true);
    leaderClimber.enableSoftLimit(SoftLimitDirection.kReverse, true);

    leaderClimber.setInverted(true);

    followerClimber.follow(leaderClimber, true);
  }

  public void setClimberSpeed(double d) {
    leaderClimber.set(/*climberRateLimiter.calculate(*/d * ClimberConstants.climberSpeedFactor);
  }

  public double getClimberPosition() {
    return leaderClimber.getEncoder().getPosition();
  }

  public boolean getUpperLimit() {
    return Math.abs(getClimberPosition() - leaderClimber.getSoftLimit(SoftLimitDirection.kForward)) < ClimberConstants.limitVariability;
  }

  public boolean getLowerLimit() {
    return Math.abs(getClimberPosition() - followerClimber.getSoftLimit(SoftLimitDirection.kReverse)) < ClimberConstants.limitVariability;
  }

  @Override
  public void periodic() {
    System.out.println(getClimberPosition());
  }

  @Override
  public void simulationPeriodic() {}
}