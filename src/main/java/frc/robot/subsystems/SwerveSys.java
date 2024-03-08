package frc.robot.subsystems;

import java.util.Optional;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.LimelightPoseEstimator;

public class SwerveSys extends SubsystemBase {

    // Initializes swerve module objects
    private final SwerveModule frontLeftMod = 
        new SwerveModule(
            CANDevices.frontLeftDriveMtrId,
            CANDevices.frontLeftSteerMtrId,
            CANDevices.frontLeftCanCoderId,
            DriveConstants.frontLeftModOffset
        );

    private final SwerveModule frontRightMod = 
        new SwerveModule(
            CANDevices.frontRightDriveMtrId,
            CANDevices.frontRightSteerMtrId,
            CANDevices.frontRightCanCoderId,
            DriveConstants.frontRightModOffset
        );

    private final SwerveModule backLeftMod = 
        new SwerveModule(
            CANDevices.backLeftDriveMtrId,
            CANDevices.backLeftSteerMtrId,
            CANDevices.backLeftCanCoderId,
            DriveConstants.backLeftModOffset
        );

    private final SwerveModule backRightMod = 
        new SwerveModule(
            CANDevices.backRightDriveMtrId,
            CANDevices.backRightSteerMtrId,
            CANDevices.backRightCanCoderId,
            DriveConstants.backRightModOffset
        );

    private boolean isLocked = false;
    public boolean isLocked() {
        return isLocked;
    }

    private boolean isFieldOriented = true;
    public boolean isFieldOriented() {
        return isFieldOriented;
    }

    private double speedFactor = 1.0;
    public double getSpeedFactor() {
        return speedFactor;
    }
    /**
     * Sets the speed factor of the robot. Inputs are multiplied by this factor to reduce drive speed.
     * Useful for "turtle" or "sprint" modes.
     * @param speedFactor The factor to scale inputs, as a percentage.
     */
    public void setSpeedFactor(double speedFactor) {
        this.speedFactor = speedFactor;
    }

    private Optional<Double> omegaOverrideRadPerSec = Optional.empty();
    public void setOmegaOverrideRadPerSec(Optional<Double> omegaOverrideRadPerSec) {
        this.omegaOverrideRadPerSec = omegaOverrideRadPerSec;
    }
    public boolean hasOmegaOverride() {
        return omegaOverrideRadPerSec.isPresent();
    }

    private final Pigeon2 imu = new Pigeon2(CANDevices.imuId);

    // Odometry for the robot, measured in meters for linear motion and radians for rotational motion
    // Takes in kinematics and robot angle for parameters

    private final SwerveDrivePoseEstimator poseEstimator = 
        // new SwerveDrivePoseEstimator(
        //     DriveConstants.kinematics,
        //     imu.getRotation2d(),
        //     getModulePositions(),
        //     new Pose2d()
        // );
        new SwerveDrivePoseEstimator(
            DriveConstants.kinematics,
            imu.getRotation2d(),
            getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(0.5)),
            VecBuilder.fill(0.25, 0.25, Units.degreesToRadians(30.0)));

    private final LimelightPoseEstimator[] limelightPoseEstimators = new LimelightPoseEstimator[] {
        new LimelightPoseEstimator(VisionConstants.frontLimelightName, this::getPose),
        new LimelightPoseEstimator(VisionConstants.backLimelightName, this::getPose)
    };

    /**
     * Constructs a new SwerveSys.
     * 
     * <p>SwerveCmd contains 4 {@link SwerveModule}, a gyro, and methods to control the drive base and odometry.
     */
    public SwerveSys() {
        // Resets the measured distance driven for each module
        frontLeftMod.resetDriveDistance();
        frontRightMod.resetDriveDistance();
        backLeftMod.resetDriveDistance();
        backRightMod.resetDriveDistance();
        
        resetPose();
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        // Updates the odometry every 20ms
        poseEstimator.update(imu.getRotation2d(), getModulePositions());

        for(LimelightPoseEstimator limelightPoseEstimator : limelightPoseEstimators) {
            Optional<Pose2d> limelightPose = limelightPoseEstimator.getLimelightPoseEstimate();
            if(limelightPose.isPresent()) {
                // TODO find out if rotation from limelight works
                // Ignores heading from Limelights since gyro is very percise
                // poseEstimator.addVisionMeasurement(
                //     new Pose2d(limelightPose.get().getTranslation(), getHeading()),
                //     limelightPoseEstimator.getCaptureTimestamp());

                poseEstimator.addVisionMeasurement(limelightPose.get(), limelightPoseEstimator.getCaptureTimestamp());
            }
        }

        SmartDashboard.putBoolean("omega override", hasOmegaOverride());
        if(hasOmegaOverride()) {
            SmartDashboard.putNumber("omega override value", omegaOverrideRadPerSec.get());
        }
    }
    
    /**
     * Inputs drive values into the swerve drive base.
     * 
     * @param driveXMetersPerSec The desired forward/backward lateral motion, in meters per second.
     * @param driveYMetersPerSec The desired left/right lateral motion, in meters per second.
     * @param rotationRadPerSec The desired rotational motion, in radians per second.
     * @param isFieldOriented whether driving is field- or robot-oriented.
     */
    public void drive(double driveXMetersPerSec, double driveYMetersPerSec, double rotationRadPerSec, boolean isFieldOriented) {
        if(omegaOverrideRadPerSec.isPresent()) {
            rotationRadPerSec = omegaOverrideRadPerSec.get();
        }

        if(DriverStation.getAlliance().get() == Alliance.Red) {
            driveXMetersPerSec *= -1;
            driveYMetersPerSec *= -1;
        }

        if(driveXMetersPerSec != 0.0 || driveYMetersPerSec != 0.0 || rotationRadPerSec != 0.0) isLocked = false;
        
        if(isLocked) {
            setModuleStates(new SwerveModuleState[] {
                new SwerveModuleState(0.0, new Rotation2d(0.25 * Math.PI)),
                new SwerveModuleState(0.0, new Rotation2d(-0.25 * Math.PI)),
                new SwerveModuleState(0.0, new Rotation2d(-0.25 * Math.PI)),
                new SwerveModuleState(0.0, new Rotation2d(0.25 * Math.PI))
            });
        }
        else {
            // Reduces the speed of the drive base for "turtle" or "sprint" modes.
            driveXMetersPerSec *= speedFactor;
            driveYMetersPerSec *= speedFactor;
            rotationRadPerSec *= speedFactor;

            // Represents the overall state of the drive base.
            ChassisSpeeds speeds =
                isFieldOriented
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        driveXMetersPerSec, driveYMetersPerSec, rotationRadPerSec, getHeading())
                    : new ChassisSpeeds(driveXMetersPerSec, driveYMetersPerSec, rotationRadPerSec);

            ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

            // Uses kinematics (wheel placements) to convert overall robot state to array of individual module states.
            SwerveModuleState[] states = DriveConstants.kinematics.toSwerveModuleStates(discreteSpeeds);
            
            // Makes sure the wheels don't try to spin faster than the maximum speed possible
            SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxModuleSpeedMetersPerSec);

            setModuleStates(states);
        }
    }

    public void runCharacterizationVolts(double volts) {
        frontLeftMod.runCharacterization(volts);
        frontRightMod.runCharacterization(volts);
        backLeftMod.runCharacterization(volts);
        backRightMod.runCharacterization(volts);
    }

    /**
     * Stops the driving of the drive base.
     * <p>Sets all drive inputs to zero. This will set the drive power of each module to zero while maintaining module headings.
     */
    public void stop() {
        drive(0.0, 0.0, 0.0, isFieldOriented);
    }

    /**
     * Turns the modules to the X-lock position as long as drive inputs are zero.
     */
    public void lock() {
        isLocked = true;
    }

    /**
     * Sets the desired state for each swerve module.
     * <p>Uses PID and feedforward control (closed-loop) to control the linear and rotational values for the modules.
     * 
     * @param moduleStates An array module states to set. The order is FL, FR, BL, BR.
     */
    public void setModuleStates(SwerveModuleState[] moduleStates) {
        frontLeftMod.setDesiredState(moduleStates[0], true);
        frontRightMod.setDesiredState(moduleStates[1], true);
        backLeftMod.setDesiredState(moduleStates[2], true);
        backRightMod.setDesiredState(moduleStates[3], true);
    }

    /**
     * Returns the current motion of the drive base as a ChassisSpeeds.
     * 
     * @return A ChassisSpeeds representing the current motion of the drive base.
     */
    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.kinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Sets the ChassisSpeeds of the drive base.
     * 
     * @param chassisSpeeds The desired ChassisSpeeds.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        setModuleStates(DriveConstants.kinematics.toSwerveModuleStates(chassisSpeeds));
    }

    public Translation2d getFieldRelativeVelocity() {
        return new Translation2d(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond).rotateBy(getHeading());
    }
    

    /**
     * Returns an array of module states of the drive base. The order is FL, FR, BL, BR.
     * 
     * @return An array of SwerveModuleState.
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            new SwerveModuleState(frontLeftMod.getVelocityMetersPerSec(), frontLeftMod.getSteerEncAngle()),
            new SwerveModuleState(frontRightMod.getVelocityMetersPerSec(), frontRightMod.getSteerEncAngle()),
            new SwerveModuleState(backLeftMod.getVelocityMetersPerSec(), backLeftMod.getSteerEncAngle()),
            new SwerveModuleState(backRightMod.getVelocityMetersPerSec(), backRightMod.getSteerEncAngle())
        };
    }

    /**
     * Returns an array of CANcoder angles of the modules. The order is FL, FR, BL, BR.
     * 
     * @return An array of Rotation2d.
     */
    public Rotation2d[] getCanCoderAngles() {
        return new Rotation2d[] {
            frontLeftMod.getCanCoderAngle(),
            frontRightMod.getCanCoderAngle(),
            backLeftMod.getCanCoderAngle(),
            backRightMod.getCanCoderAngle()
        };
    }

    /**
     * Returns an array of module positions.
     * 
     * @return An array of SwerveModulePosition.
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeftMod.getPosition(),
            frontRightMod.getPosition(),
            backLeftMod.getPosition(),
            backRightMod.getPosition()
        };
    }

    /**
     * @return The current estimated position of the robot.
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * @return The current estimated pose of the robot, which will be mirrored if on the red alliance.
     * This is useful for checking the pose of the robot in an autonomous program, as PathPlanner paths
     * can mirror blue side paths for use on the red side.
     */
    public Pose2d getBlueSidePose() {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            return new Pose2d(16.54 - getPose().getX(), getPose().getY(), new Rotation2d(MathUtil.angleModulus(getPose().getRotation().getRadians() - Math.PI)));
        }
        else {
            return getPose();
        }
    }

    public void setHeading(Rotation2d heading) {
        imu.setYaw(MathUtil.inputModulus(heading.getDegrees(), 0.0, 360.0));
    }
    
    /**
     * Resets the current pose to (0, 0) with a heading of zero.
     */
    public void resetPose() {
        setPose(new Pose2d());
    }
    
    /**
     * Sets the pose of the robot.
     * 
     * @param pose The pose to set the robot to.
     */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(imu.getRotation2d(), getModulePositions(), pose);
    }

    public void setTranslation(Translation2d translation) {
        poseEstimator.resetPosition(imu.getRotation2d(), getModulePositions(), new Pose2d(translation, imu.getRotation2d()));
    }

    /**
     * Resets the measured distance driven for each module to zero.
     * <p>Resets the drive encoders of each module to zero.
     */
    public void resetDriveDistances() {
        frontLeftMod.resetDriveDistance();
        frontRightMod.resetDriveDistance();
        backLeftMod.resetDriveDistance();
        backRightMod.resetDriveDistance();
    }

    /**
     * Returns the average distance driven of each module to get an overall distance driven by the robot.
     * 
     * @return The overall distance driven by the robot in meters.
     */
    public double getAverageDriveDistanceMeters() {
        return (
            (frontLeftMod.getDriveDistanceMeters()
            + frontRightMod.getDriveDistanceMeters()
            + backLeftMod.getDriveDistanceMeters()
            + backRightMod.getDriveDistanceMeters())
            / 4.0
        );
    }

    /**
     * Returns the average velocity of each module to get an overall velocity of the robot.
     * 
     * @return The overall velocity of the robot in meters per second.
     */
    public double getAverageDriveVelocityMetersPerSec() {
        return (
            (Math.abs(frontLeftMod.getVelocityMetersPerSec())
            + Math.abs(frontRightMod.getVelocityMetersPerSec())
            + Math.abs(backLeftMod.getVelocityMetersPerSec() )
            + Math.abs(backRightMod.getVelocityMetersPerSec()))
            / 4.0
        );
    }

    public double getAverageDriveVoltage() {
        return (
            (Math.abs(frontLeftMod.getDriveVoltage())
            + Math.abs(frontRightMod.getDriveVoltage())
            + Math.abs(backLeftMod.getDriveVoltage())
            + Math.abs(backRightMod.getDriveVoltage()))
            / 4.0
        );
    }

    /**
     * Returns the average direction of each module to get an overall direction of travel of the robot.
     * 
     * @return The overall direction of travel of the robot.
     */
    public Rotation2d getDirectionOfTravel() {
        return new Rotation2d(
            (frontLeftMod.getSteerEncAngle().plus(new Rotation2d(frontLeftMod.getVelocityMetersPerSec() < 0.0 ? Math.PI : 0.0)).getRadians()
            + frontRightMod.getSteerEncAngle().plus(new Rotation2d(frontRightMod.getVelocityMetersPerSec() < 0.0 ? Math.PI : 0.0)).getRadians()
            + backLeftMod.getSteerEncAngle().plus(new Rotation2d(backLeftMod.getVelocityMetersPerSec() < 0.0 ? Math.PI : 0.0)).getRadians()
            + backRightMod.getSteerEncAngle().plus(new Rotation2d(backRightMod.getVelocityMetersPerSec() < 0.0 ? Math.PI : 0.0)).getRadians()
            ) / 4.0
        );
    }

    /**
     * Returns the average velocity in the direction relative to the robot.
     * 
     * @param relativeHeading The relative heading of the robot, where zero is the front of the robot.
     * 
     * @return The velocity in the direction relative to the robot in meters per second.
     */
    public double getRelativeVelocityMetersPerSec(Rotation2d relativeHeading) {
        return getDirectionOfTravel().minus(relativeHeading).getCos() * getAverageDriveVelocityMetersPerSec();
    }

    /**
     * Returns the current heading of the robot from the gyro.
     * 
     * @return The current heading of the robot as a Rotation2d.
     */
    public Rotation2d getHeading() {
        // return Rotation2d.fromRadians(MathUtil.angleModulus(Units.degreesToRadians(imu.getYaw().getValueAsDouble())));
        // return imu.getRotation2d();
        return getPose().getRotation();
    }

    /**
     * Returns the current pitch of the robot from the gyro.
     * 
     * @return The current pitch of the robot as a Rotation2d.
     */
    public Rotation2d getPitch() {
        // IMU is turned 90 degrees, so pitch and roll are flipped.
        return Rotation2d.fromDegrees(imu.getRoll().getValueAsDouble());
    }

    /**
     * Returns the current roll of the robot from the gyro.
     * 
     * @return The current roll of the robot as a Rotation2d.
     */
    public Rotation2d getRollDegrees() {
        // IMU is turned 90 degrees, so pitch and roll are flipped.
        return Rotation2d.fromDegrees(imu.getPitch().getValueAsDouble());
    }

    /**
     * Sets the gyro heading to zero.
     */
    public void resetHeading() {
        poseEstimator.resetPosition(
            imu.getRotation2d(),
            getModulePositions(),
            new Pose2d(
                getPose().getTranslation(),
                DriverStation.getAlliance().get() == Alliance.Red ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0)));
    }

    /**
     * Sets the current limit of the drive motors of each module to the desired amperage.
     * 
     * @param amps The desired current limit of the drive motors in amps.
     */
    public void setDriveCurrentLimit(int amps) {
        frontLeftMod.setDriveCurrentLimit(amps);
        frontRightMod.setDriveCurrentLimit(amps);
        backLeftMod.setDriveCurrentLimit(amps);
        backRightMod.setDriveCurrentLimit(amps);
    }
}