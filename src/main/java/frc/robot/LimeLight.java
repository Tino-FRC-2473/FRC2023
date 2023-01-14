package frc.robot;

import org.photonvision.PhotonCamera;
//import org.photonvision.PhotonUtils;
import org.photonvision.PhotonUtils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/** Handles all actions for the Limelight camera. */
public class LimeLight {
  /** Photonvision camera object. */
  private PhotonCamera camera;

  /** angle of camera (radians). */
  private static final double CAMERA_ANGLE = Math.toRadians(0);

  /** height of AprilTag (meters). */
  private static final double APRIL_TAG_HEIGHT = 0.4444;

  /** height of camera (meters). On robot is .4953, on ground is .3810. */
  private static final double CAMERA_HEIGHT = 0.3810;

  /** invalid return constant. */
  public static final double INVALID_RETURN = -2;

  /** converion constant: 39.3701 inches in a meter. */
  public static final double METERS_TO_INCHES = 39.3701;

  /** converion constant: degrees to radians. */
  private static final double DEG_TO_RAD = Math.PI / 180;

  /** Creates a new LimeLight and initiates a camera. */
  public LimeLight() {
    camera = new PhotonCamera("gloworm");
  }

  /** Updates values on SmartDashboard. */
  public final void update() {
    SmartDashboard.putNumber("Distance", getAprilTagDistance());
    SmartDashboard.updateValues();
    System.out.println("Distance " + getAprilTagDistance());
  }

   /**
   * Returns distance to AprilTags.
   * @return distance to AprilTag
   */
  public final double getAprilTagDistance() {
    camera.setPipelineIndex(1);
    var result = camera.getLatestResult();
    if (result.hasTargets()) {
      /*
      double heightDiff = APRIL_TAG_HEIGHT - CAMERA_HEIGHT;
      double pitch = result.getBestTarget().getPitch();
      double angle = CAMERA_ANGLE + Math.toRadians(pitch);
      double visionDist = heightDiff / Math.tan(angle);
      double visionDistInches = METERS_TO_INCHES * visionDist;
      return visionDistInches;
      */
      return METERS_TO_INCHES
        * PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT,
                APRIL_TAG_HEIGHT,
                CAMERA_ANGLE,
                (result.getBestTarget().getPitch() * DEG_TO_RAD));
    }
    return -1;
  }

}
