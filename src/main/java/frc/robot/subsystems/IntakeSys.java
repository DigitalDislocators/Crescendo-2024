package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.IntakeConstants;

public class IntakeSys extends SubsystemBase {

    // Declare actuators, sensors, and other variables here

        private final CANSparkFlex leaderRollerMtr;
        private final CANSparkFlex followerRollerMtr;

        private final CANSparkFlex leaderPivotMtr;
        private final CANSparkFlex followerPivotMtr;

        private final RelativeEncoder rollerEnc;

        private final RelativeEncoder pivotEnc;

        private final SparkPIDController pivotController;

        private double targetInches = 0.0;

        private boolean pivotIsManual = false;
        private boolean rollersAreManual = false; 
        private boolean rollersAreRelative = false;

        private DoubleSupplier robotSpeedMetersPerSecond;
        private double relativeSpeed = 0.0;

        /**
     * Intake needs to be offset since the encoder can't output negative values.
     * This value is an approximate midpoint between zero and its max value.
     */
    private final double offsetInches = 435.0;

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
        
        leaderRollerMtr.setIdleMode(IdleMode.kBrake);
        
        followerRollerMtr.follow(leaderRollerMtr, true);

        
        leaderPivotMtr = new CANSparkFlex(CANDevices.leaderPivotMtrId, MotorType.kBrushless);
        followerPivotMtr = new CANSparkFlex(CANDevices.followerPivotMtrId, MotorType.kBrushless);

        leaderPivotMtr.getEncoder().setVelocityConversionFactor(IntakeConstants.pivotGearReduction);
        followerPivotMtr.getEncoder().setVelocityConversionFactor(IntakeConstants.pivotGearReduction);

        leaderPivotMtr.setSmartCurrentLimit(IntakeConstants.maxPivotCurrentAmps);
        
        leaderPivotMtr.setIdleMode(IdleMode.kBrake);

        followerPivotMtr.follow(leaderPivotMtr, true);
        

        rollerEnc = leaderRollerMtr.getEncoder();
        rollerEnc.setPosition(0);
        rollerEnc.setInverted(false);
        rollerEnc.setPositionConversionFactor(IntakeConstants.inchesPerEncRev);

        pivotEnc = leaderPivotMtr.getEncoder();
        pivotEnc.setPosition(0);
        pivotEnc.setInverted(false);
        pivotEnc.setPositionConversionFactor(IntakeConstants.inchesPerEncRev);

        pivotController = leaderPivotMtr.getPIDController();
        pivotController.setP(IntakeConstants.KP);
        pivotController.setD(IntakeConstants.KD);

        pivotController.setIZone(0);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        if(!rollersAreManual) {
            if(!pivotIsManual && getCurrentPosition() > IntakeConstants.rollerStartInches && targetInches == IntakeConstants.outInches) {
                if(rollersAreRelative)
                    setRPM((relativeSpeed - (robotSpeedMetersPerSecond.getAsDouble() * IntakeConstants.rollerRelativeSpeedFactor)) * IntakeConstants.driveMetersPerSecondToRollerRPM);
                else
                    setRPM(relativeSpeed * IntakeConstants.driveMetersPerSecondToRollerRPM);
            }
            else {
                setRPM(0.0);
            }
        }
        
    }

    // Put methods for controlling this subsystem here. Call these from Commands.

public double getCurrentPosition() {
        return pivotEnc.getPosition() - offsetInches;
    }

    public void manualRollerControl(double power) {
        if(power != 0.0) {
            rollersAreManual = true;
            leaderRollerMtr.set(power);
        }
        else {
            rollersAreManual = false;
        }
    }
}