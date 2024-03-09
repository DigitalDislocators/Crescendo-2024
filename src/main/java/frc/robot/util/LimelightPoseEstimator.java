// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstants;

public class LimelightPoseEstimator {

	private final String limelightName;

  	/** Creates a new VisionSys. */
  	public LimelightPoseEstimator(String limelightName) {
		this.limelightName = limelightName;
	}

	public Optional<Pose2d> getRobotPose() {
		Pose2d limelightPose = LimelightHelpers.getBotPose2d_wpiBlue(limelightName);
		// Returns empty if either no pose is given or if the area of the largest tag is less than the threshold.
		if(limelightPose.equals(new Pose2d()) || LimelightHelpers.getTA(limelightName) < VisionConstants.targetAreaPercentThreshold) {
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
