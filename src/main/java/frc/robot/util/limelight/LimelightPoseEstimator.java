// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.limelight;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstants;

public class LimelightPoseEstimator {

	private final String limelightName;

	private final Transform2d poseOffset;

	/**
	 * Creates a new LimelightPoseEstimator.
	 * 
	 * <p>LimelightPoseEstimator is a helper object for returning robot pose estimates from Limelights.
	 * 
	 * @param limelightName The name of the Limelight pipeline.
	 * @param poseOffset A Transform2d to shift the Limelight pose if it is consistently off by a certain amount.
	 */
  	public LimelightPoseEstimator(String limelightName, Transform2d poseOffset) {
		this.limelightName = limelightName;
		this.poseOffset = poseOffset;
	}

	/**
	 * Creates a new LimelightPoseEstimator.
	 * 
	 * <p>LimelightPoseEstimator is a helper object for returning robot pose estimates from Limelights.
	 * 
	 * @param limelightName The name of the Limelight pipeline.
	 */
	public LimelightPoseEstimator(String limelightName) {
		this(limelightName, new Transform2d());
	}

	public Optional<Pose2d> getRobotPose() {
		Pose2d limelightPose = LimelightHelpers.getBotPose2d_wpiBlue(limelightName);
		// Returns empty if either no pose is given or if the area of the largest tag is less than the threshold.
		if(limelightPose.equals(new Pose2d()) || LimelightHelpers.getTA(limelightName) < VisionConstants.targetAreaPercentThreshold) {
			return Optional.empty();
		}
		else {
			return Optional.of(limelightPose.transformBy(poseOffset));
		}
	}

	public double getLatencySec() {
		return (LimelightHelpers.getLatency_Capture(limelightName) + LimelightHelpers.getLatency_Pipeline(limelightName)) * 0.001;
	}

	public double getCaptureTimestamp() {
		return Timer.getFPGATimestamp() - getLatencySec();
	}

}
