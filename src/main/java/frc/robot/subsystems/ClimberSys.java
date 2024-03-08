package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.ClimberConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimberSys extends SubsystemBase {
  
  private final CANSparkMax leftClimberMtr = new CANSparkMax(CANDevices.leftClimberMtrId, MotorType.kBrushless);
  private final CANSparkMax rightClimberMtr = new CANSparkMax(CANDevices.rightClimberMtrId, MotorType.kBrushless);
  
  public ClimberSys() {
    leftClimberMtr.setIdleMode(IdleMode.kBrake);
    rightClimberMtr.setIdleMode(IdleMode.kBrake);

    leftClimberMtr.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.climberForwardLimit);
    leftClimberMtr.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.climberReverseLimit);

    leftClimberMtr.enableSoftLimit(SoftLimitDirection.kForward, true);
    leftClimberMtr.enableSoftLimit(SoftLimitDirection.kReverse, true);

    rightClimberMtr.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.climberForwardLimit);
    rightClimberMtr.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.climberReverseLimit);

    rightClimberMtr.enableSoftLimit(SoftLimitDirection.kForward, true);
    rightClimberMtr.enableSoftLimit(SoftLimitDirection.kReverse, true);

    leftClimberMtr.setInverted(true);
  }

  @Override
  public void periodic() {}

  public void setClimberPower(double power) {
    leftClimberMtr.set(power * ClimberConstants.climberSpeedFactor);
    rightClimberMtr.set(power * ClimberConstants.climberSpeedFactor);
  }

  public double getClimberPosition() {
    return (leftClimberMtr.getEncoder().getPosition() + rightClimberMtr.getEncoder().getPosition()) / 2;
  }
}