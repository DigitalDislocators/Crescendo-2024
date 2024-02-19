package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.ClimberConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimberSys extends SubsystemBase {
  
  private final CANSparkMax leaderClimber = new CANSparkMax(CANDevices.leaderClimbMtrId, MotorType.kBrushless);
  private final CANSparkMax followerClimber = new CANSparkMax(CANDevices.followerClimbMtrId, MotorType.kBrushless);
  
  public ClimberSys() {
    leaderClimber.setIdleMode(IdleMode.kBrake);
    followerClimber.setIdleMode(IdleMode.kBrake);

    leaderClimber.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.climberForwardLimit);
    leaderClimber.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.climberReverseLimit);

    leaderClimber.enableSoftLimit(SoftLimitDirection.kForward, true);
    leaderClimber.enableSoftLimit(SoftLimitDirection.kReverse, true);

    leaderClimber.setInverted(true);

    followerClimber.follow(leaderClimber, true);
  }

  @Override
  public void periodic() {
    // System.out.println(getClimberPosition());
  }

  public void setClimberPower(double power) {
    leaderClimber.set(power * ClimberConstants.climberSpeedFactor);
  }

  public double getClimberPosition() {
    return leaderClimber.getEncoder().getPosition();
  }

  public boolean isAtUpperLimit() {
    return Math.abs(getClimberPosition() - leaderClimber.getSoftLimit(SoftLimitDirection.kForward)) < ClimberConstants.limitThreshold;
  }

  public boolean isAtLowerLimit() {
    return Math.abs(getClimberPosition() - followerClimber.getSoftLimit(SoftLimitDirection.kReverse)) < ClimberConstants.limitThreshold;
  }
}