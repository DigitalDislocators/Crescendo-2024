package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.RollerConstants;

public class RollersSys extends SubsystemBase {

    // Declare actuators, sensors, and other variables here
    private final CANSparkFlex topRollerMtr;
    private final CANSparkFlex bottomRollerMtr;

    private final SparkPIDController topRollerController;
    private final SparkPIDController bottomRollerController;

    public RollersSys() {
        topRollerMtr = new CANSparkFlex(CANDevices.leaderRollerMtrId, MotorType.kBrushless);
        bottomRollerMtr = new CANSparkFlex(CANDevices.followerRollerMtrId, MotorType.kBrushless);

        topRollerMtr.restoreFactoryDefaults();
        bottomRollerMtr.restoreFactoryDefaults();

        topRollerMtr.enableVoltageCompensation(10);
        bottomRollerMtr.enableVoltageCompensation(10);

        topRollerMtr.setSmartCurrentLimit(RollerConstants.maxRollerCurrentAmps);
        
        topRollerMtr.setIdleMode(IdleMode.kBrake);
        bottomRollerMtr.setIdleMode(IdleMode.kBrake);

        topRollerMtr.setInverted(true);
        bottomRollerMtr.setInverted(true);
        
        topRollerController = topRollerMtr.getPIDController();
        bottomRollerController = bottomRollerMtr.getPIDController();

        topRollerController.setFF(RollerConstants.feedForward);
        topRollerController.setP(RollerConstants.kP);
        topRollerController.setD(RollerConstants.kD);

        bottomRollerController.setFF(RollerConstants.feedForward);
        bottomRollerController.setP(RollerConstants.kP);
        bottomRollerController.setD(RollerConstants.kD);
    }

    @Override
    public void periodic() {

    }

    public void setRPM(double rpm) {
        // leaderRollerMtr.set(rpm / RollerConstants.maxRPM);
        topRollerController.setReference(rpm, ControlType.kVelocity);
        bottomRollerController.setReference(rpm, ControlType.kVelocity);
    }

    public void setPower(double power) {
        topRollerMtr.set(power);
        bottomRollerMtr.set(power);
    }

    public double getRPM() {
        return (topRollerMtr.getEncoder().getVelocity() + bottomRollerMtr.getEncoder().getVelocity()) / 2;
    }

}