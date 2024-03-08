// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

public class LimelightPoseEstimator {

	private final String limelightName;

  	/** Creates a new VisionSys. */
  	public LimelightPoseEstimator(String limelightName) {
		this.limelightName = limelightName;
	}

	public Optional<Pose2d> getRobotPose() {
		Pose2d limelightPose = LimelightHelpers.getBotPose2d_wpiBlue(limelightName);
		// Limelight returns a pose of the origin when it has no targets
		if(limelightPose == null || limelightPose.equals(new Pose2d())) {
			return Optional.empty();
		}
		else {
			return Optional.of(limelightPose);
		}
	}

	public double getLatencySec() {
		return (LimelightHelpers.getLatency_Capture(limelightName) + LimelightHelpers.getLatency_Capture(limelightName)) * 0.001;
	}

	public double getCaptureTimestamp() {
		return Timer.getFPGATimestamp() - getLatencySec();
	}

}
