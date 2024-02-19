package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.RollerConstants;

public class RollersSys extends SubsystemBase {

    // Declare actuators, sensors, and other variables here
    private final CANSparkFlex leaderRollerMtr;
    private final CANSparkFlex followerRollerMtr;

    public RollersSys() {
        leaderRollerMtr = new CANSparkFlex(CANDevices.leaderRollerMtrId, MotorType.kBrushless);
        followerRollerMtr = new CANSparkFlex(CANDevices.followerRollerMtrId, MotorType.kBrushless);

        leaderRollerMtr.restoreFactoryDefaults();
        followerRollerMtr.restoreFactoryDefaults();

        leaderRollerMtr.enableVoltageCompensation(10);
        followerRollerMtr.enableVoltageCompensation(10);

        leaderRollerMtr.setSmartCurrentLimit(RollerConstants.maxRollerCurrentAmps);
        
        leaderRollerMtr.setIdleMode(IdleMode.kBrake);

        leaderRollerMtr.setInverted(true);
        
        followerRollerMtr.follow(leaderRollerMtr, false);
    }

    @Override
    public void periodic() {

    }

    public void setRPM(double rpm) {
        leaderRollerMtr.set(rpm / RollerConstants.maxRPM);
    }

    public void setPower(double power) {
        leaderRollerMtr.set(power);
    }

}