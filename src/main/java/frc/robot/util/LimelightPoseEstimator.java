// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstants;

public class LimelightPoseEstimator {

  	private TimeInterpolatableBuffer<Pose2d> drivePoseBuffer;

  	private final Supplier<Pose2d> drivePoseSupplier;

	private final String limelightName;

  	/** Creates a new VisionSys. */
  	public LimelightPoseEstimator(
		String limelightName,
		Supplier<Pose2d> drivePoseSupplier) {

		this.limelightName = limelightName;
		this.drivePoseSupplier = drivePoseSupplier;

		drivePoseBuffer = TimeInterpolatableBuffer.createBuffer(VisionConstants.bufferSizeSeconds);
	}

  	public void update() {
    	// This method will be called once per scheduler run
    	drivePoseBuffer.addSample(Timer.getFPGATimestamp(), drivePoseSupplier.get());
  	}

	// TODO find out if latency compensation here is necessary or if the one in PoseEstimator works. If not, get rid of all unecessary methods.
	public Optional<Pose2d> getRobotPose() {
		Optional<Pose2d> captureTimeDrivePose = getCaptureTimeDrivePose();
		Optional<Pose2d> limelightPose = getLimelightPoseEstimate();
		if(captureTimeDrivePose.isPresent() && limelightPose.isEmpty()) {
			Pose2d currentPoseFromDrive = drivePoseSupplier.get();
			Pose2d capturePoseFromDrive = captureTimeDrivePose.get();
			Pose2d capturePoseFromLL = captureTimeDrivePose.get();

			// Calculates the current pose from the limelight using the transform of the difference between the
			// current pose of the robot and the pose of the robot at the time of capture from the limelight.
			Pose2d currentPoseFromLL = capturePoseFromLL.plus(currentPoseFromDrive.minus(capturePoseFromDrive));
			return Optional.of(currentPoseFromLL);
		}
		else {
			return Optional.empty();
		}
	}

	public Optional<Pose2d> getLimelightPoseEstimate() {
		Pose2d limelightPose = LimelightHelpers.getBotPose2d_wpiBlue(limelightName);
		// Limelight returns a pose of the origin when it has no targets
		if(limelightPose == null || limelightPose.equals(new Pose2d())) {
			return Optional.empty();
		}
		else {
			return Optional.of(limelightPose);
		}
	}

	private Optional<Pose2d> getCaptureTimeDrivePose() {
		return drivePoseBuffer.getSample(getLatencySec());
	}

	public double getLatencySec() {
		return (LimelightHelpers.getLatency_Capture(limelightName) + LimelightHelpers.getLatency_Capture(limelightName)) * 0.001;
	}

	public double getCaptureTimestamp() {
		return Timer.getFPGATimestamp() - getLatencySec();
	}

}
