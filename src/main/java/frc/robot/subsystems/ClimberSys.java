package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.ClimberConstants;

public class ClimberSys extends SubsystemBase {

    // Declare actuators, sensors, and other variables here

        private final CANSparkMax rightClimbMtr;
        private final CANSparkMax leftClimbMtr;

        private RelativeEncoder rightClimbEnc;

        private RelativeEncoder leftClimbEnc;


    /**
     * Constructs a new ClimberSys.
     * 
     * <p>ClimberSys contains the basic framework of a robot subsystem for use in command-based programming.
     */
    public ClimberSys() {
        // Initialize and configure actuators and sensors here
    
        rightClimbMtr = new CANSparkMax(CANDevices.rightClimbMtrId, MotorType.kBrushless);
        rightClimbMtr.getEncoder().setVelocityConversionFactor(ClimberConstants.rightClimbGearReduction);
        rightClimbMtr.setSmartCurrentLimit(ClimberConstants.maxClimberCurrentAmps);
        rightClimbMtr.setIdleMode(IdleMode.kBrake);

        leftClimbMtr = new CANSparkMax(CANDevices.leftClimbMtrId, MotorType.kBrushless);
        leftClimbMtr.getEncoder().setVelocityConversionFactor(ClimberConstants.leftClimbGearReduction);
        leftClimbMtr.setSmartCurrentLimit(ClimberConstants.maxClimberCurrentAmps);
        leftClimbMtr.setIdleMode(IdleMode.kBrake);


        rightClimbEnc = rightClimbMtr.getEncoder();
        rightClimbEnc.setPosition(0);
        rightClimbEnc.setInverted(false);
        rightClimbEnc.setPositionConversionFactor(ClimberConstants.inchesPerEncRev);

        leftClimbEnc = leftClimbMtr.getEncoder();
        leftClimbEnc.setPosition(0);
        leftClimbEnc.setInverted(false);
        leftClimbEnc.setPositionConversionFactor(ClimberConstants.inchesPerEncRev);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        
            if(power <= 0) {
                rightClimbEnc.set(power);
            }
            else {
                rightClimbEnc.set(0.0);
            }
    
            if(power > 0) {
                ClimberSys.climb(true);
            }
            else if(power < 0) {
                ClimberSys.climb(false);
            }
    
        }
    
        public void set(double power) {
            rightClimbMtr.set(power);
        }
    
        public void stop() {
            rightClimbMtr.stopMotor();
        }
    
        public double getClimberCounts() {
            return -rightClimbMtr.getSelectedSensorPosition();
        }
    
        public void zero() {
            rightClimbMtr.setSelectedSensorPosition(0.0);
        }
    }

    // Put methods for controlling this subsystem here. Call these from Commands.

