package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.RollerConstants;

public class RollerSys extends SubsystemBase {

    // Declare actuators, sensors, and other variables here

     private final CANSparkFlex leaderRollerMtr;
     private final CANSparkFlex followerRollerMtr;

    // private final RelativeEncoder rollerEnc;

    // private final SparkPIDController pivotController;

    private boolean rollersAreManual = false;

    public RollerSys() {
         leaderRollerMtr = new CANSparkFlex(CANDevices.leaderRollerMtrId, MotorType.kBrushless);
         followerRollerMtr = new CANSparkFlex(CANDevices.followerRollerMtrId, MotorType.kBrushless);

        leaderRollerMtr.restoreFactoryDefaults();
        followerRollerMtr.restoreFactoryDefaults();

        leaderRollerMtr.setSmartCurrentLimit(RollerConstants.maxRollerCurrentAmps);
        
        leaderRollerMtr.setIdleMode(IdleMode.kBrake);

        leaderRollerMtr.setInverted(true);
        
        followerRollerMtr.follow(leaderRollerMtr, false);
        

       // rollerEnc = leaderRollerMtr.getEncoder();
        // pivotEnc.setPositionConversionFactor(IntakeConstants.pivotDegreesPerEncRev);
        // pivotEnc.setVelocityConversionFactor(IntakeConstants.pivotDegPerSecPerRPM);


    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {}

    public void manualRollerControl(double power) {
        leaderRollerMtr.set(power);
    }

    public void setManualPower(double power) {
        leaderRollerMtr.set(power);
    }

}
