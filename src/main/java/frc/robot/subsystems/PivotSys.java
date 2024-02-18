package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.PivotConstants;

public class PivotSys extends SubsystemBase {

    // Declare actuators, sensors, and other variables here

    private final CANSparkFlex leaderPivotMtr;
    private final CANSparkFlex followerPivotMtr;

    // private final RelativeEncoder rollerEnc;

    private final RelativeEncoder pivotEnc;

    // private final SparkPIDController pivotController;

    private final ProfiledPIDController pivotController;

    private double targetDegrees = 0.0;

    private double manualDegreesPerSec = 0.0;

    private boolean pivotIsManual = false;

    public PivotSys() {
        leaderPivotMtr = new CANSparkFlex(CANDevices.leaderPivotMtrId, MotorType.kBrushless);
        followerPivotMtr = new CANSparkFlex(CANDevices.followerPivotMtrId, MotorType.kBrushless);

        leaderPivotMtr.restoreFactoryDefaults();
        followerPivotMtr.restoreFactoryDefaults();

        leaderPivotMtr.enableVoltageCompensation(12);
        followerPivotMtr.enableVoltageCompensation(12);

        leaderPivotMtr.setSmartCurrentLimit(PivotConstants.maxPivotCurrentAmps);

        leaderPivotMtr.setSoftLimit(SoftLimitDirection.kForward, 20.0f);
        leaderPivotMtr.setSoftLimit(SoftLimitDirection.kReverse, 0.5f);

        leaderPivotMtr.enableSoftLimit(SoftLimitDirection.kForward, true);
        leaderPivotMtr.enableSoftLimit(SoftLimitDirection.kReverse, true);
        
        leaderPivotMtr.setIdleMode(IdleMode.kBrake);
        followerPivotMtr.setIdleMode(IdleMode.kBrake);

        followerPivotMtr.follow(leaderPivotMtr, true);
        

       // rollerEnc = leaderRollerMtr.getEncoder();

        pivotEnc = leaderPivotMtr.getEncoder();
        pivotEnc.setPosition(0.0);
        // pivotEnc.setPositionConversionFactor(IntakeConstants.pivotDegreesPerEncRev);
        // pivotEnc.setVelocityConversionFactor(IntakeConstants.pivotDegPerSecPerRPM);


        pivotController = new ProfiledPIDController(0.09, 0.0, 0, new Constraints(150, 150));
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        //System.out.println(getCurrentPositionDegrees());
    }

    // Put methods for controlling this subsystem here. Call these from Commands.

    public double getCurrentPositionDegrees() {
        return pivotEnc.getPosition();
    }

    public void setPivotDeg(double deg) {
        leaderPivotMtr.set(deg);
        //leaderPivotMtr.set(pivotController.calculate(getCurrentPositionDegrees(), 10));
    }
}
