package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

public class LimeLight {
    private PhotonCamera camera;
	private static final double HUB_CAMERA_ANGLE = Math.toRadians(0); //RADIANS //30.25
	//private static final double BALL_CAMERA_ANGLE = Math.toRadians(); //RADIANS
	private static final double HUB_HEIGHT = 2.6; //METERS
	private static final double APRIL_TAG_HEIGHT = 0.4699; //METERS
	//private static final double BALL_HEIGHT = ; //METERS
	private static final double CAMERA_HEIGHT = 0.584; //METERS
	public static final double INVALID_RETURN = -2;
	private double last_seen_location = -1;

    public LimeLight() {
		camera = new PhotonCamera("gloworm");
	}

	public void update() {
		//SmartDashboard.putNumber("Distance", getAprilTagDistance());
		SmartDashboard.updateValues();

	}

    public double getAprilTagDistance() {
		camera.setPipelineIndex(1);
		var result = camera.getLatestResult();
		if (result.hasTargets()) {
			//SmartDashboard.putNumber("angle", result.getBestTarget().getPitch());
			//SmartDashboard.updateValues();
			/*
			return PhotonUtils.calculateDistanceToTargetMeters(
								CAMERA_HEIGHT,
								APRIL_TAG_HEIGHT,
								HUB_CAMERA_ANGLE,
								Math.toRadians(result.getBestTarget().getPitch()));
			*/
			//39.3701 inches = 1 meter
			double visionDist = 39.3701 * (APRIL_TAG_HEIGHT - CAMERA_HEIGHT)/Math.tan(HUB_CAMERA_ANGLE+Math.toRadians(result.getBestTarget().getPitch()));
			return 13.2*Math.exp(0.0168 * visionDist); //within 5 inches except when distance is 4.5-7.3 ft
		}
		return -1;
	}
}
