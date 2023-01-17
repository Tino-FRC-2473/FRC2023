package frc.robot;
/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

/**
 * The PhotonCameraWrapper class contains methods for estimating position
 * of robot relative to AprilTags on the field and updates SmartDashboard
 * with its coordinates.
 */
public class PhotonCameraWrapper {
		/** PhotonCamera object representing a camera that is
		 * connected to PhotonVision.*/
	private PhotonCamera photonCamera;
		/** RobotPoseEstimator object to estimate position of robot.*/
	private RobotPoseEstimator robotPoseEstimator;
		/** conversion constant: 39.3701 inches in a meter. */
	public static final double METERS_TO_INCHES = 39.3701;

		/** Creates a new PhotonCameraWrapper. */
	public PhotonCameraWrapper() {
		ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
		atList.add(new AprilTag(1, new Pose3d(new Pose2d(
			Constants.AprilTagConstants.X1,
			Constants.AprilTagConstants.Y1,
			Rotation2d.fromDegrees(
			Constants.AprilTagConstants.ROT1)))));

		AprilTagFieldLayout atfl =
				new AprilTagFieldLayout(atList,
										FieldConstants.LENGTH,
										FieldConstants.WIDTH);

		// Forward Camera
		photonCamera =
				new PhotonCamera(
						VisionConstants
								.CAMERA_NAME);
		// PhotonVision UI.

		// ... Add other cameras here

		// Assemble the list of cameras & mount locations
		var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
		camList.add(new Pair<PhotonCamera, Transform3d>(photonCamera,
												VisionConstants.ROBOT_TO_CAM));

		robotPoseEstimator = new RobotPoseEstimator(atfl,
										PoseStrategy.LOWEST_AMBIGUITY, camList);
	}

	/**
	 * Updates values on SmartDashboard.
	 */
	public final void update() {
		SmartDashboard.putNumber("locationX",
								getEstimatedGlobalPose().getFirst().getX());
		SmartDashboard.putNumber("locationY",
								getEstimatedGlobalPose().getFirst().getY());
		SmartDashboard.updateValues();
		System.out.println(" x: " + getEstimatedGlobalPose().getFirst().getX());
		System.out.println(" y: " + getEstimatedGlobalPose().getFirst().getY());
	}

	/**
	 * Gets estimated global pose.
	 * @return A pair of the fused camera observations to a single Pose2d
	 *  on the field, and the time of the observation. Assumes a planar
	 *  field and the robot is always firmly on the ground.
	 */
	public Pair<Pose2d, Double> getEstimatedGlobalPose() {
		double currentTime = Timer.getFPGATimestamp();
		Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();
		if (result.isPresent()) {
			return new Pair<Pose2d, Double>(
					result.get().getFirst().toPose2d(),
					currentTime - result.get().getSecond());
		} else {
			return new Pair<Pose2d, Double>(null, 0.0);
		}
	}
}
