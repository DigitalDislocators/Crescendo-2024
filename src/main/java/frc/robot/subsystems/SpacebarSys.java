package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.SpacebarConstants;

public class SpacebarSys extends SubsystemBase {
  
  private final CANSparkMax spacebarMtr = new CANSparkMax(CANDevices.spacebarMtrId, MotorType.kBrushless);

  private final ProfiledPIDController spacebarController;

  private double targetDeg = 0.0;
  
  public SpacebarSys() {
    spacebarMtr.setIdleMode(IdleMode.kBrake);

    spacebarMtr.enableVoltageCompensation(10);

    spacebarMtr.setSoftLimit(SoftLimitDirection.kForward, SpacebarConstants.spacebarForwardLimit);
    spacebarMtr.setSoftLimit(SoftLimitDirection.kReverse, SpacebarConstants.spacebarReverseLimit);

    spacebarMtr.enableSoftLimit(SoftLimitDirection.kForward, false);
    spacebarMtr.enableSoftLimit(SoftLimitDirection.kReverse, false);

    spacebarMtr.setInverted(false);

    spacebarController = new ProfiledPIDController(SpacebarConstants.kP, 0.0, SpacebarConstants.kD, 
    new Constraints(SpacebarConstants.maxVelDegPerSec, SpacebarConstants.maxAccelDegPerSecSq));

    setTargetDeg(-2);
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
    return (spacebarMtr.getEncoder().getPosition() * 360 / SpacebarConstants.gearReduction);
    // return (spacebarMtr.getEncoder().getCountsPerRevolution());
  }

  public double getCurrentPositionEncCounts() {
    return (spacebarMtr.getEncoder().getPosition());
  }

  public double getSpacebarTargetDeg() {
    return (targetDeg);
  }
}