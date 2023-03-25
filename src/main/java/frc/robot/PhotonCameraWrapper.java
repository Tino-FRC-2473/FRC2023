package frc.robot;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.photonvision.PhotonUtils;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/**
 * The PhotonCameraWrapper class contains methods for estimating position
 * of robot relative to AprilTags on the field and updates SmartDashboard
 * with its coordinates.
 */
public class PhotonCameraWrapper {
	public static final double APRIL_TAG_ANGLE_DEGREES = 180;
	public static final double ANGULAR_P = 0.01;
	public static final double ANGULAR_D = 0;
		/** PhotonCamera object representing a camera that is
		 * connected to PhotonVision.*/
	private PhotonCamera photonCamera;
		/** RobotPoseEstimator object to estimate position of robot.*/
	private PhotonPoseEstimator robotPoseEstimator;
		/** PIDController object to implement PID for robot turning.*/
	private PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

		/** Last timestamp holder to check if code is running faster than limelight.*/
	private double lastTs = 0;

		/** Creates a new PhotonCameraWrapper. */
	public PhotonCameraWrapper() {
		AprilTagFieldLayout atfl = null;
		try {
			atfl = AprilTagFieldLayout.loadFromResource(
				AprilTagFields.k2023ChargedUp.m_resourceFile);
			System.out.println(atfl);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		photonCamera =
				new PhotonCamera("OV5647");
		photonCamera.setDriverMode(false);
		robotPoseEstimator = new PhotonPoseEstimator(atfl, PoseStrategy.LOWEST_AMBIGUITY,
		photonCamera, new Transform3d(
			new Translation3d(Units.inchesToMeters(VisionConstants.CAM_OFFSET_INCHES),
			0,
			VisionConstants.CAM_HEIGHT_METERS),
			new Rotation3d(
					0, VisionConstants.CAM_PITCH_RADIANS,
					0)));
	}
	/**
	 * Gets estimated global pose.
	 * @return A pair of the fused camera observations to a single Pose2d
	 *  on the field, and the time of the observation. Assumes a planar
	 *  field and the robot is always firmly on the ground.
	 */
	public Pose3d getEstimatedGlobalPose() {
		photonCamera.setPipelineIndex(VisionConstants.THREEDTAG_PIPELINE_INDEX);
		Optional<EstimatedRobotPose> optPose = robotPoseEstimator.update();
		if (optPose.isEmpty()) {
			return null;
		}
		return optPose.get().estimatedPose;
	}
	/**
	 * Gets estimated global pose.
	 * @return A pair of the fused camera observations to a single Pose2d
	 *  on the field, and the time of the observation. Assumes a planar
	 *  field and the robot is always firmly on the ground.
	 */
	public double getTagRotation() {
		photonCamera.setPipelineIndex(VisionConstants.THREEDTAG_PIPELINE_INDEX);
		var result = photonCamera.getLatestResult();
		if (!result.hasTargets()) {
			return -Constants.INVALID_TURN_RETURN_DEGREES;
		}
		return Units.radiansToDegrees(
			result.getBestTarget().getBestCameraToTarget().getRotation().getAngle());
	}

	/**
	 * Sets PhotonCamera object pipeline index to the index inputted into the method.
	 * @param index
	 */
	public void setPipelineIndex(int index) {
		photonCamera.setPipelineIndex(index);
	}

	/**
	 * Returns the angle for the robot to turn to align with the lower reflective tape.
	 * @return an angle that tells the robot how much to turn to align in degrees
	 */
	public double getLowerTapeTurnAngle() {
		photonCamera.setPipelineIndex(VisionConstants.LOWERTAPE_PIPELINE_INDEX);
		var result = photonCamera.getLatestResult();
		if (result.hasTargets()) {
			return result.getBestTarget().getYaw() + Math.toDegrees(Math.atan(
				VisionConstants.CAM_OFFSET_INCHES / getLowerTapeDistance()));
		}
		return Constants.INVALID_TURN_RETURN_DEGREES;
	}

	/**
	 * Returns the rotation power for the robot to align with the cube object.
	 * @param cntID the specific contour selected
	 * @return a power to apply to all the motors
	 */
	public double getCubeTurnRotation(int cntID) {
		photonCamera.setPipelineIndex(VisionConstants.CUBE_PIPELINE_INDEX);
		var result = photonCamera.getLatestResult();
		double rotationSpeed;
		double curTs = photonCamera.getLatestResult().getTimestampSeconds();
		//compare curTs to lastTs: check if code is running faster than limelight (causes overshoot)
		if (result.hasTargets() && (lastTs == 0 || curTs != lastTs)) {
			rotationSpeed = -turnController.calculate(getCubeTurnAngle(cntID), 0);
		} else {
			// If we have no targets, stay still.
			rotationSpeed = 0;
		}
		lastTs = photonCamera.getLatestResult().getTimestampSeconds();
		return rotationSpeed;
	}

	/**
	 * Returns the rotation power for the robot to align with the cone object.
	 * @param cntID the specific contour selected
	 * @return a power to apply to all the motors
	 */
	public double getConeTurnRotation(int cntID) {
		photonCamera.setPipelineIndex(VisionConstants.CONE_PIPELINE_INDEX);
		var result = photonCamera.getLatestResult();
		double rotationSpeed;
		double curTs = photonCamera.getLatestResult().getTimestampSeconds();
		//compare curTs to lastTs: check if code is running faster than limelight (causes overshoot)
		if (result.hasTargets() && (lastTs == 0 || curTs != lastTs)) {
			rotationSpeed = -turnController.calculate(getConeTurnAngle(cntID), 0);
		} else {
			// If we have no targets, stay still.
			rotationSpeed = 0;
		}
		lastTs = photonCamera.getLatestResult().getTimestampSeconds();
		return rotationSpeed;
	}
	/**
	 * Returns the angle for the robot to turn to align with the higher reflective tape.
	 * @return an angle that tells the robot how much to turn to align in degrees
	 */
	public double getHigherTapeTurnAngle() {
		photonCamera.setPipelineIndex(VisionConstants.HIGHERTAPE_PIPELINE_INDEX);
		var result = photonCamera.getLatestResult();
		if (result.hasTargets()) {
			return result.getBestTarget().getYaw() + Math.toDegrees(Math.atan(
				VisionConstants.CAM_OFFSET_INCHES / getHigherTapeDistance()));
		}
		return Constants.INVALID_TURN_RETURN_DEGREES;
	}

	/**
	 * Returns the angle for the robot to turn to align with the grid april tag.
	 * @return an angle that tells the robot how much to turn to align in degrees
	 */
	public double getTagTurnAngle() {
		photonCamera.setPipelineIndex(VisionConstants.TWODTAG_PIPELINE_INDEX);
		var result = photonCamera.getLatestResult();
		if (result.hasTargets()) {
			return result.getBestTarget().getYaw() + Math.toDegrees(Math.atan(
				VisionConstants.CAM_OFFSET_INCHES / getTagDistance()));
		}
		return Constants.INVALID_TURN_RETURN_DEGREES;
	}

	/**
	 * Returns the rotation power for the robot to turn to align with the grid april tag (uses pid).
	 * @return an angle that tells the robot how much to turn to align in degrees
	 */
	public double getTagTurnRotation() {
		photonCamera.setPipelineIndex(VisionConstants.TWODTAG_PIPELINE_INDEX);
		var result = photonCamera.getLatestResult();
		double rotationSpeed;
		double curTs = photonCamera.getLatestResult().getTimestampSeconds();
		//compare curTs to lastTs: check if code is running faster than limelight (causes overshoot)
		if (result.hasTargets() && (lastTs == 0 || curTs != lastTs)) {
			// Calculate angular turn power
			// -1.0 required to ensure positive PID controller effort increases yaw
			rotationSpeed = -turnController.calculate(getTagTurnAngle(), 0);
		} else {
			// If we have no targets, stay still.
			rotationSpeed = 0;
		}
		lastTs = photonCamera.getLatestResult().getTimestampSeconds();
		return rotationSpeed;
	}
	/**
	 * Returns the distance to the grid april tag.
	 * @return a distance in inches
	 */
	public double getTagDistance() {
		photonCamera.setPipelineIndex(VisionConstants.TWODTAG_PIPELINE_INDEX);
		var result = photonCamera.getLatestResult();
		if (result.hasTargets()) {
			return Units.metersToInches(PhotonUtils.calculateDistanceToTargetMeters(
			VisionConstants.CAM_HEIGHT_METERS,
			AprilTagConstants.APRILTAG_1_HEIGHT_METERS, VisionConstants.CAM_PITCH_RADIANS,
			Units.degreesToRadians(result.getBestTarget().getPitch())));
		} else {
			return -1;
		}
	}

	/**
	 * Returns the distance to the lower reflective tape.
	 * @return a distance in inches
	 */
	public double getLowerTapeDistance() {
		photonCamera.setPipelineIndex(VisionConstants.LOWERTAPE_PIPELINE_INDEX);
		var result = photonCamera.getLatestResult();
		if (result.hasTargets()) {
			return Units.metersToInches(PhotonUtils.calculateDistanceToTargetMeters(
			VisionConstants.CAM_HEIGHT_METERS,
			VisionConstants.LOW_TAPE_HEIGHT_METERS, VisionConstants.CAM_PITCH_RADIANS,
			Units.degreesToRadians(result.getBestTarget().getPitch())));
		} else {
			return -1;
		}
	}
	/**
	 * Returns the distance to the higher reflective tape.
	 * @return a distance in inches
	 */
	public double getHigherTapeDistance() {
		photonCamera.setPipelineIndex(VisionConstants.HIGHERTAPE_PIPELINE_INDEX);
		var result = photonCamera.getLatestResult();
		if (result.hasTargets()) {
			return Units.metersToInches(PhotonUtils.calculateDistanceToTargetMeters(
			VisionConstants.CAM_HEIGHT_METERS,
			VisionConstants.HIGH_TAPE_HEIGHT_METERS, VisionConstants.CAM_PITCH_RADIANS,
			Units.degreesToRadians(result.getBestTarget().getPitch())));
		} else {
			return -1;
		}
	}
	/** @return Returns a distance in meters from the closest cone and -1 if there are no cones.*/
	public double getDistanceToCone() {
		photonCamera.setPipelineIndex(VisionConstants.CONE_PIPELINE_INDEX);
		var result = photonCamera.getLatestResult();
		if (result.hasTargets()) {
			return Units.metersToInches(PhotonUtils.calculateDistanceToTargetMeters(
				VisionConstants.CAM_HEIGHT_METERS, VisionConstants.CONE_HEIGHT_METERS,
				VisionConstants.CAM_PITCH_RADIANS, Units.degreesToRadians(
					result.getBestTarget().getPitch()))) + Constants.CONE_DISTANCE_ADD;
		}
		return -1;
	}
/** @return Returns a distance in meters from the closest cube and -1 if there are no cubes.*/
	public double getDistanceToCube() {
		photonCamera.setPipelineIndex(VisionConstants.CUBE_PIPELINE_INDEX);
		var result = photonCamera.getLatestResult();
		if (result.hasTargets()) {
			return Units.metersToInches(PhotonUtils.calculateDistanceToTargetMeters(
				VisionConstants.CAM_HEIGHT_METERS, VisionConstants.CUBE_HEIGHT_METERS,
				VisionConstants.CAM_PITCH_RADIANS, Units.degreesToRadians(
					result.getBestTarget().getPitch()))) + Constants.CUBE_DISTANCE_ADD;
		}
		return -1;
	}
/** @return Returns the angle needed to turn for aligning the robot to the cone
 * @param cnt the specific contour to find the angle towards
 * and 360 if there are no cones.*/
	public double getConeTurnAngle(int cnt) {
		photonCamera.setPipelineIndex(VisionConstants.CONE_PIPELINE_INDEX);
		var result = photonCamera.getLatestResult();
		if (result.hasTargets()) {
			return result.getTargets().get(cnt).getYaw() + Math.toDegrees(Math.atan(
				VisionConstants.CAM_OFFSET_INCHES / getDistanceToCone()));
		}
		return Constants.INVALID_TURN_RETURN_DEGREES;
	}
/** @return Returns the angle needed to turn for aligning the robot to the cube
 * @param cnt the specific contour to find the angle towards
 * and 360 if there are no cubes.*/
	public double getCubeTurnAngle(int cnt) {
		photonCamera.setPipelineIndex(VisionConstants.CUBE_PIPELINE_INDEX);
		var result = photonCamera.getLatestResult();
		if (result.hasTargets()) {
			return result.getTargets().get(cnt).getYaw() + Math.toDegrees(Math.atan(
				VisionConstants.CAM_OFFSET_INCHES / getDistanceToCube()));
		}
		return Constants.INVALID_TURN_RETURN_DEGREES;
	}
/**
 * @return Returns the ID of the closest April Tag seen.
 */
	public int getAprilTagID() {
		photonCamera.setPipelineIndex(VisionConstants.TWODTAG_PIPELINE_INDEX);
		return photonCamera.getLatestResult().getBestTarget().getFiducialId();
	}

/**
 * @return Returns if the robot is parallel to the april tag of the double substation.
 */
	public boolean isParallelToSubstation() {
		double rotation = getTagRotation();
		if (rotation == -Constants.INVALID_TURN_RETURN_DEGREES) {
			return false;
		}
		if (rotation < APRIL_TAG_ANGLE_DEGREES + Constants.SUBSTATION_ANGLE_THRESHOLD_DEGREES
			&& rotation > APRIL_TAG_ANGLE_DEGREES - Constants.SUBSTATION_ANGLE_THRESHOLD_DEGREES) {
			return true;
		}
		return false;
	}
/**
 * @return Returns the number of targets.
 */
	public int getNumberofTargets() {
		return photonCamera.getLatestResult().getTargets().size();
	}
}
