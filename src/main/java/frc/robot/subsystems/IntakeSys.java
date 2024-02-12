package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DriverStation;
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

    // private final SparkPIDController pivotController;

    private final ProfiledPIDController pivotController;

    private double targetDegrees = 0.0;

    private double manualDegreesPerSec = 0.0;

    private boolean pivotIsManual = false;
    private boolean rollersAreManual = false;

    /**
     * Constructs a new IntakeSys.
     * 
     * <p>ExampleSys contains the basic framework of a robot subsystem for use in command-based programming.
     */
    public IntakeSys() {
        // Initialize and configure actuators and sensors here
        
        leaderRollerMtr = new CANSparkFlex(CANDevices.leaderRollerMtrId, MotorType.kBrushless);
        followerRollerMtr = new CANSparkFlex(CANDevices.followerRollerMtrId, MotorType.kBrushless);

        leaderRollerMtr.restoreFactoryDefaults();
        followerRollerMtr.restoreFactoryDefaults();

        leaderRollerMtr.setSmartCurrentLimit(IntakeConstants.maxRollerCurrentAmps);
        
        leaderRollerMtr.setIdleMode(IdleMode.kBrake);
        
        followerRollerMtr.follow(leaderRollerMtr, true);

        
        leaderPivotMtr = new CANSparkFlex(CANDevices.leaderPivotMtrId, MotorType.kBrushless);
        followerPivotMtr = new CANSparkFlex(CANDevices.followerPivotMtrId, MotorType.kBrushless);

        leaderPivotMtr.restoreFactoryDefaults();
        followerPivotMtr.restoreFactoryDefaults();

        leaderPivotMtr.setSmartCurrentLimit(IntakeConstants.maxPivotCurrentAmps);
        
        leaderPivotMtr.setIdleMode(IdleMode.kBrake);

        followerPivotMtr.follow(leaderPivotMtr, true);
        

        rollerEnc = leaderRollerMtr.getEncoder();
        rollerEnc.setInverted(false);

        pivotEnc = leaderPivotMtr.getEncoder();
        pivotEnc.setPosition(0.0);
        pivotEnc.setInverted(false);
        pivotEnc.setPositionConversionFactor(IntakeConstants.pivotDegreesPerEncRev);
        pivotEnc.setVelocityConversionFactor(IntakeConstants.pivotDegPerSecPerRPM);

        // pivotController = leaderPivotMtr.getPIDController();
        // pivotController.setP(IntakeConstants.KP);
        // pivotController.setD(IntakeConstants.KD);

        pivotController = new ProfiledPIDController(IntakeConstants.KP, 0.0, IntakeConstants.KD, new Constraints(0.0, 0.0));
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        if(manualDegreesPerSec != 0.0) {
            pivotIsManual = true;
        }
        else {
            pivotIsManual = false;
        }
        
        if(pivotIsManual) {
            // pivotController.setReference(targetDegrees, ControlType.kPosition);
            pivotController.setGoal(targetDegrees);
            leaderPivotMtr.set(pivotController.calculate(getCurrentPositionDegrees()));
        }
        else {
            setPivotDegPerSec(manualDegreesPerSec);
            targetDegrees = getCurrentPositionDegrees();
        }

        if(DriverStation.isDisabled()) {
            pivotController.reset(getCurrentPositionDegrees());
            pivotController.setGoal(getCurrentPositionDegrees());
        }
    }

    // Put methods for controlling this subsystem here. Call these from Commands.

    public double getCurrentPositionDegrees() {
        return pivotEnc.getPosition();
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

    public void setRollersRPM(double rpm) {
        leaderRollerMtr.set(rpm / IntakeConstants.rollerFreeSpeedRPM);
    }

    public void setPivotDegPerSec(double degPerSec) {
        leaderPivotMtr.set((degPerSec / IntakeConstants.pivotDegPerSecPerRPM) / IntakeConstants.pivotFreeSpeedRPM);
    }

    public Object getAsDouble() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getAsDouble'");
    }
}
