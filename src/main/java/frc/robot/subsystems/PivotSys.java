package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.PivotConstants;

public class PivotSys extends SubsystemBase {

    // Declare actuators, sensors, and other variables here
    private final CANSparkFlex leaderPivotMtr;
    private final CANSparkFlex followerPivotMtr;

    private final RelativeEncoder pivotEnc;

    private final ProfiledPIDController pivotController;

    private double targetDegrees = 0.0;

    private double manualDegPerSec = 0.0;

    public PivotSys() {
        leaderPivotMtr = new CANSparkFlex(CANDevices.leaderPivotMtrId, MotorType.kBrushless);
        followerPivotMtr = new CANSparkFlex(CANDevices.followerPivotMtrId, MotorType.kBrushless);



        leaderPivotMtr.restoreFactoryDefaults();
        followerPivotMtr.restoreFactoryDefaults();

        leaderPivotMtr.enableVoltageCompensation(10);
        followerPivotMtr.enableVoltageCompensation(10);

        leaderPivotMtr.setSmartCurrentLimit(PivotConstants.maxPivotCurrentAmps);

        leaderPivotMtr.setSoftLimit(SoftLimitDirection.kForward, PivotConstants.upperLimitDegrees);
        leaderPivotMtr.setSoftLimit(SoftLimitDirection.kReverse, PivotConstants.lowerLimitDegrees);

        leaderPivotMtr.enableSoftLimit(SoftLimitDirection.kForward, true);
        leaderPivotMtr.enableSoftLimit(SoftLimitDirection.kReverse, true);
        
        leaderPivotMtr.setIdleMode(IdleMode.kBrake);
        followerPivotMtr.setIdleMode(IdleMode.kBrake);

        followerPivotMtr.follow(leaderPivotMtr, true);
        
        pivotEnc = leaderPivotMtr.getEncoder();

        pivotEnc.setPositionConversionFactor(PivotConstants.degreesPerEncRev);
        pivotEnc.setVelocityConversionFactor(PivotConstants.degPerSecPerRPM);

        pivotEnc.setPosition(PivotConstants.homePresetDeg);

        pivotController = new ProfiledPIDController(
            PivotConstants.Kp, PivotConstants.Ki, PivotConstants.Kd,
            new Constraints(PivotConstants.maxVelDegPerSec, PivotConstants.maxAccelDegPerSecSq));
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        if(manualDegPerSec == 0.0) {
            leaderPivotMtr.set(pivotController.calculate(getCurrentPositionDegrees(), targetDegrees));
        }
        else {
            leaderPivotMtr.set(manualDegPerSec);
            targetDegrees = getCurrentPositionDegrees();
            pivotController.reset(targetDegrees);
        }

        if(DriverStation.isDisabled()) {
            targetDegrees = getCurrentPositionDegrees();
            pivotController.reset(targetDegrees);
        }
        System.out.println(getCurrentPositionDegrees());
    }

    // Put methods for controlling this subsystem here. Call these from Commands.
    public double getCurrentPositionDegrees() {
        return pivotEnc.getPosition();
    }

    public void setPivotDeg(double degrees) {
        targetDegrees = degrees;
    }

    public void setManualDegPerSec(double degPerSec) {
        manualDegPerSec = degPerSec;
    }
}