package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.IntakeConstants;

public class IntakeSys extends SubsystemBase {

    // Declare actuators, sensors, and other variables here

        private final CANSparkFlex leaderRollerMtr;
        private final CANSparkFlex followerRollerMtr;

        private final RelativeEncoder intakeEnc;

        private final SparkPIDController controller;
    /**
     * Constructs a new IntakeSys.
     * 
     * <p>ExampleSys contains the basic framework of a robot subsystem for use in command-based programming.
     */
    public IntakeSys() {
        // Initialize and configure actuators and sensors here
        
        leaderRollerMtr = new CANSparkFlex(CANDevices.leaderRollerMtrId, MotorType.kBrushless);
        followerRollerMtr = new CANSparkFlex(CANDevices.followerRollerMtrId, MotorType.kBrushless);

        leaderRollerMtr.setSmartCurrentLimit(IntakeConstants.maxRollerCurrentAmps);
            // Carl was here
        leaderRollerMtr.setIdleMode(IdleMode.kBrake);

        followerRollerMtr.follow(leaderRollerMtr, true);

        intakeEnc = leaderRollerMtr.getEncoder();

        intakeEnc.setPosition(0);

        intakeEnc.setPositionConversionFactor(IntakeConstants.inchesPerEncRev);

        //Change the Mtr to the pivot Mtr
        controller = leaderRollerMtr.getPIDController();

        controller.setP(IntakeConstants.KP);
        controller.setD(IntakeConstants.KD);


    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        
    }

    // Put methods for controlling this subsystem here. Call these from Commands.

}