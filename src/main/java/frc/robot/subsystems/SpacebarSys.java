package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.SpacebarConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SpacebarSys extends SubsystemBase {
  
  private final CANSparkMax spacebarMtr = new CANSparkMax(CANDevices.spacebarMtrId, MotorType.kBrushless);

  private final ProfiledPIDController spacebarController;

  private double targetDeg = 0.0;
  
  public SpacebarSys() {
    spacebarMtr.setIdleMode(IdleMode.kBrake);

    spacebarMtr.setSoftLimit(SoftLimitDirection.kForward, SpacebarConstants.spacebarForwardLimit);
    spacebarMtr.setSoftLimit(SoftLimitDirection.kReverse, SpacebarConstants.spacebarReverseLimit);

    spacebarMtr.enableSoftLimit(SoftLimitDirection.kForward, true);
    spacebarMtr.enableSoftLimit(SoftLimitDirection.kReverse, true);

    spacebarMtr.setInverted(false);

    spacebarController = new ProfiledPIDController(SpacebarConstants.kP, 0.0, SpacebarConstants.kD, 
    new Constraints(SpacebarConstants.maxVelDegPerSec, SpacebarConstants.maxAccelDegPerSecSq));
  }

  @Override
  public void periodic() {
    spacebarMtr.set(spacebarController.calculate(getCurrentPositionDeg(), targetDeg));
  }

  // public void setPower(double power) {
    // spacebarMtr.set(power);
  // }

  public void setTargetDeg(double degrees) {
    targetDeg = degrees;
  }

  public double getCurrentPositionDeg() {
    return ((spacebarMtr.getEncoder().getPosition() / 42.0) * 360 * SpacebarConstants.gearReduction);
  }

  public double getCurrentPositionEncCounts() {
    return (spacebarMtr.getEncoder().getPosition());
  }
}