package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.PivotConstants;

public class PivotSys extends SubsystemBase {

    // Declare actuators, sensors, versh, and other variables here
    private final CANSparkFlex leftPivotMtr;
    private final CANSparkFlex rightPivotMtr;

    private final RelativeEncoder leftPivotEnc;
    private final RelativeEncoder rightPivotEnc;

    private final ProfiledPIDController pivotController;

    private final DutyCycleEncoder absPivotEnc;

    private double targetDeg = 0.0;

    private double manualPower = 0.0;

    public PivotSys() {
        leftPivotMtr = new CANSparkFlex(CANDevices.leftPivotMtrId, MotorType.kBrushless);
        rightPivotMtr = new CANSparkFlex(CANDevices.rightPivotMtrId, MotorType.kBrushless);

        leftPivotMtr.setInverted(true);

        leftPivotMtr.enableVoltageCompensation(10);
        rightPivotMtr.enableVoltageCompensation(10);

        leftPivotMtr.setSmartCurrentLimit(PivotConstants.maxPivotCurrentAmps);
        rightPivotMtr.setSmartCurrentLimit(PivotConstants.maxPivotCurrentAmps);

        leftPivotMtr.setSoftLimit(SoftLimitDirection.kForward, PivotConstants.upperLimitDeg);
        leftPivotMtr.setSoftLimit(SoftLimitDirection.kReverse, PivotConstants.lowerLimitDeg);
        rightPivotMtr.setSoftLimit(SoftLimitDirection.kForward, PivotConstants.upperLimitDeg);
        rightPivotMtr.setSoftLimit(SoftLimitDirection.kReverse, PivotConstants.lowerLimitDeg);

        leftPivotMtr.enableSoftLimit(SoftLimitDirection.kForward, true);
        leftPivotMtr.enableSoftLimit(SoftLimitDirection.kReverse, true);
        rightPivotMtr.enableSoftLimit(SoftLimitDirection.kForward, true);
        rightPivotMtr.enableSoftLimit(SoftLimitDirection.kReverse, true);
        
        leftPivotMtr.setIdleMode(IdleMode.kBrake);
        rightPivotMtr.setIdleMode(IdleMode.kBrake);
        
        leftPivotEnc = leftPivotMtr.getEncoder();
        rightPivotEnc = rightPivotMtr.getEncoder();

        leftPivotEnc.setPositionConversionFactor(PivotConstants.degPerEncRev);
        leftPivotEnc.setVelocityConversionFactor(PivotConstants.degPerSecPerRPM);
        rightPivotEnc.setPositionConversionFactor(PivotConstants.degPerEncRev);
        rightPivotEnc.setVelocityConversionFactor(PivotConstants.degPerSecPerRPM);

        leftPivotEnc.setPosition(PivotConstants.homePresetDeg);
        rightPivotEnc.setPosition(PivotConstants.homePresetDeg);

        absPivotEnc = new DutyCycleEncoder(9);

        absPivotEnc.setDistancePerRotation(360);

        absPivotEnc.setPositionOffset(PivotConstants.absPivotEncOffset);

        pivotController = new ProfiledPIDController(
            PivotConstants.kP, 0.0, PivotConstants.kD,
            new Constraints(PivotConstants.maxVelDegPerSec, PivotConstants.maxAccelDegPerSecSq));
    }

    // This method will be called once per scheduler run

    @Override
    public void periodic() {
        if(manualPower == 0.0) {
            leftPivotMtr.set(pivotController.calculate(absPivotEnc.getDistance(), targetDeg));
            rightPivotMtr.set(pivotController.calculate(absPivotEnc.getDistance(), targetDeg));
        }
        else {
            leftPivotMtr.set(manualPower);
            rightPivotMtr.set(manualPower);
            targetDeg = getCurrentPositionDeg();
            pivotController.reset(targetDeg);
        }

        if(DriverStation.isDisabled()) {
            targetDeg = getCurrentPositionDeg();
            pivotController.reset(targetDeg);
        }

        SmartDashboard.putNumber("pivot error deg", getCurrentPositionDeg() - targetDeg);
    }

    // Put methods for controlling this subsystem here. Call these from Commands.
    public double getCurrentPositionDeg() {
        return absPivotEnc.getDistance();
    }

    public void setTargetDeg(double degrees) {
        targetDeg = degrees;
    }

    public void setManualSpeedDegPerSec(double degPerSec) {
        double manualPower = (degPerSec / 6.0) / PivotConstants.freeSpeedRPM;
        this.manualPower = manualPower;
    }

    public boolean isAtTarget() {
        return Math.abs(getCurrentPositionDeg() - targetDeg) < PivotConstants.toleranceDeg;
    }
}