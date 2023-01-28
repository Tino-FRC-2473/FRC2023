package frc.robot;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.Pair;

public class DrivePoseEstimator {
	private PhotonCameraWrapper pcw = new PhotonCameraWrapper();
	private final DifferentialDriveKinematics kinematics =
			new DifferentialDriveKinematics(Constants.TRACK_WIDTH_METERS);

	private final DifferentialDrivePoseEstimator poseEstimator =
			new DifferentialDrivePoseEstimator(
				kinematics, new Rotation2d(), 0.0, 0.0, new Pose2d());

	/**Updates the estimated pose of the robot based on vision targets and encoder and
	 *  gyro values.
	 * @param gyroAngle current gyro reading
	 * @param leftEncoderPos current encoder reading of the left motor
	 * @param rightEncoderPos current encoder reading of the right motor
	 * */
	public void updatePose(double gyroAngle, double leftEncoderPos, double rightEncoderPos) {
		poseEstimator.update(new Rotation2d(Units.degreesToRadians(gyroAngle)),
			leftEncoderPos, rightEncoderPos);
		Optional<EstimatedRobotPose> result = pcw.getEstimatedGlobalPose();
		if (!result.isEmpty()) {
			poseEstimator.addVisionMeasurement(
					result.get().estimatedPose.toPose2d(), result.get().timestampSeconds);
		}

	}
	/**@return returns the estimated pose of the robot */
	public Pose2d getCurPose() {
		return poseEstimator.getEstimatedPosition();
	}
}
