package frc.robot.systems;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
import frc.robot.PhotonCameraWrapper;
import frc.robot.drive.DriveModes;
import frc.robot.drive.DrivePower;
import frc.robot.drive.DriveFunctions;
import frc.robot.Constants;
import frc.robot.DrivePoseEstimator;

// Java Imports

public class DriveFSMSystem {


	// FSM state definitions
	public enum FSMState {
		TELE_STATE_2_MOTOR_DRIVE,
		TELE_STATE_BALANCE,
		TELE_STATE_CV_ALLIGN,
		TELE_STATE_MECANUM,
		PURE_PURSUIT,
		TURNING_STATE,
		IDLE
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax leftMotor;
	private CANSparkMax rightMotor;

	private double leftPower;
	private double rightPower;

	private boolean finishedTurning;

	private boolean isInArcadeDrive = true;

	private double roboXPos = 0;
	private double roboYPos = 0;
	private double currentEncoderPos;
	private double prevEncoderPos = 0;
	private double gyroAngleForOdo = 0;
	private AHRS gyro;
	private double startAngle;

	private double angleToTurnToFaceTag = 0;
	private double initialAngle = 0;

	private DrivePoseEstimator dpe = new DrivePoseEstimator();
	private PhotonCameraWrapper pcw = new PhotonCameraWrapper();
	private double xToATag = 0;
	private double yToATag = 0;
	private boolean isAllignedToATag = false;
	private boolean CVButtonPressed = false;

	private Optional<EstimatedRobotPose> result;


	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public DriveFSMSystem() {
		// Perform hardware init
		leftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_LEFT,
										CANSparkMax.MotorType.kBrushless);
		rightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_RIGHT,
										CANSparkMax.MotorType.kBrushless);

		leftPower = 0;
		rightPower = 0;


		finishedTurning = false;

		gyro = new AHRS(SPI.Port.kMXP);
		startAngle = 0;

		// Reset state machine
		resetAutonomous();

	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public FSMState getCurrentState() {
		return currentState;
	}

	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void resetAutonomous() {

		rightMotor.getEncoder().setPosition(0);
		leftMotor.getEncoder().setPosition(0);

		gyro.reset();
		gyro.zeroYaw();
		gyroAngleForOdo = 0;

		currentState = FSMState.TELE_STATE_2_MOTOR_DRIVE;

		roboXPos = 0;
		roboYPos = 0;

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}
	/**
	 * A.
	 */
	public void resetTeleop() {

		rightMotor.getEncoder().setPosition(0);
		leftMotor.getEncoder().setPosition(0);

		gyro.reset();
		gyro.zeroYaw();
		gyroAngleForOdo = 0;

		currentState = FSMState.TELE_STATE_2_MOTOR_DRIVE;

		roboXPos = 0;
		roboYPos = 0;

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		result = pcw.getEstimatedGlobalPose();
		//dpe.updatePose(gyro.getAngle(), leftMotor.getEncoder().getPosition(),
		//	rightMotor.getEncoder().getPosition());
		// SmartDashboard.putNumber("X", dpe.getCurPose().getX());
		// SmartDashboard.putNumber("Y", dpe.getCurPose().getY());
		// SmartDashboard.putNumber("Rotation", dpe.getCurPose().getRotation().getDegrees());

		if(!result.isEmpty()) {
			SmartDashboard.putNumber("X", result.get().estimatedPose.getX() * 39.3701);
			SmartDashboard.putNumber("Y", result.get().estimatedPose.getY() * 39.3701);
			SmartDashboard.putNumber("Rotation", 360-Units.radiansToDegrees(result.get().estimatedPose.getRotation().getAngle()));
			//SmartDashboard.putNumber("Rotation2", result.get().estimatedPose.getRotation().getAngle());
		}
		gyroAngleForOdo = gyro.getAngle();

		currentEncoderPos = ((leftMotor.getEncoder().getPosition()
			- rightMotor.getEncoder().getPosition()) / 2.0);

		updateLineOdometryTele(gyro.getAngle());

		switch (currentState) {
			case TELE_STATE_2_MOTOR_DRIVE:
				handleTeleOp2MotorState(input);
				break;

			case TELE_STATE_BALANCE:
				handleTeleOpBalanceState(input);
				break;

			case TELE_STATE_CV_ALLIGN:
			if (!result.isEmpty()) {
				xToATag = result.get().estimatedPose.getX() * 39.3701;
				yToATag = result.get().estimatedPose.getY() * 39.3701;
				System.out.println("x to tag " + xToATag);
				System.out.println("y to tag " + yToATag);

				handleCVAllignState(input);
			}
				break;

			case IDLE:
				handleIdleState(input);
				break;

			case TURNING_STATE:
				handleTurnState(input, angleToTurnToFaceTag);
				break;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		currentState = nextState(input);
	}

	/* ======================== Private methods ======================== */
	/**
	 * Decide the next state to transition to. This is a function of the inputs
	 * and the current state of this FSM. This method should not have any side
	 * effects on outputs. In other words, this method should only read or get
	 * values to decide what state to go to.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	private FSMState nextState(TeleopInput input) {
		switch (currentState) {

			case TELE_STATE_2_MOTOR_DRIVE:
				if (input != null && input.isDriveJoystickEngageButtonPressedRaw()) {
					return FSMState.TELE_STATE_BALANCE;
				} else if (input != null && input.isDriveJoystickCVAllignLeftButtonPressedRaw()) { 
					// Allign to node left of april tag
					CVButtonPressed = true;
					initialAngle = gyro.getAngle();
					isAllignedToATag = false;
					return FSMState.TELE_STATE_CV_ALLIGN;
				}
				// } else if (input != null && input.isDriveJoystickCVAllignMiddleButtonPressedRaw()) {
				// 	// Allign to node middle of april tag
				// 	return FSMState.TELE_STATE_CV_ALLIGN;
				// } else if (input != null && input.isDriveJoystickCVAllignRightButtonPressedRaw()) {
				// 	// Allign to node right of april tag
				// 	return FSMState.TELE_STATE_CV_ALLIGN;
				// }
				isAllignedToATag = false;
				CVButtonPressed = false;
				return FSMState.TELE_STATE_2_MOTOR_DRIVE;

			case TELE_STATE_MECANUM:
				return FSMState.TELE_STATE_MECANUM;

			case TURNING_STATE:
				System.out.println(finishedTurning);
				if (finishedTurning) {
					return FSMState.TELE_STATE_2_MOTOR_DRIVE;
				} else {
					return FSMState.TURNING_STATE;
				}

			case TELE_STATE_CV_ALLIGN:
				if(isAllignedToATag) {
					System.out.println("has alligned");
					return FSMState.TELE_STATE_2_MOTOR_DRIVE;
				}
				return FSMState.TELE_STATE_CV_ALLIGN;

			case IDLE:
				return FSMState.IDLE;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in TELE_STATE_2_MOTOR_DRIVE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleTeleOp2MotorState(TeleopInput input) {
		if (input == null) {
			return;
		}

		if (isInArcadeDrive) {

			currentEncoderPos = ((leftMotor.getEncoder().getPosition()
				- rightMotor.getEncoder().getPosition()) / 2.0);

			updateLineOdometryTele(gyroAngleForOdo);

			double steerAngle = input.getSteerAngle();
			double currentLeftPower = leftMotor.get();
			double currentRightPower = rightMotor.get();


			DrivePower targetPower = DriveModes.arcadeDrive(input.getdriveJoystickY(),
				steerAngle, currentLeftPower,
				currentRightPower, true);

			// multiple speed modes
			if (input.isDriveJoystickTriggerPressedRaw()) {
				targetPower.scale(Constants.MAX_POWER);
			} else {
				targetPower.scale(Constants.REDUCED_MAX_POWER);
			}

			DrivePower power;

			// acceleration
			power = DriveFunctions.accelerate(targetPower, new DrivePower(currentLeftPower,
				currentRightPower));

			// turning in place
			if (Math.abs(input.getdriveJoystickY()) < Constants.TURNING_IN_PLACE_THRESHOLD) {
				power = DriveFunctions.turnInPlace(0, steerAngle);
			}

			leftPower = power.getLeftPower();
			rightPower = power.getRightPower();
			Optional<EstimatedRobotPose> result = pcw.getEstimatedGlobalPose();
			if (!result.isEmpty() && !CVButtonPressed) {
				//if(result.get().estimatedPose.getY() < 0) {
					//angleToTurnToFaceTag = Math.toDegrees(result.get().estimatedPose.getRotation().getAngle());
				//} else {
					//angleToTurnToFaceTag = -1 * (360 - Math.toDegrees(result.get().estimatedPose.getRotation().getAngle()));
				//}
				//angleToTurnToFaceTag = 180 + angleToTurnToFaceTag - Math.toDegrees(Math.atan2(result.get().estimatedPose.getY(), result.get().estimatedPose.getX()));
				
				// left is negative right is positive
				System.out.println(!result.isEmpty());
				angleToTurnToFaceTag = -1 * (360 - Math.toDegrees(result.get().estimatedPose.getRotation().getAngle()));
				
				if(result.get().estimatedPose.getY() < 0) {
					// angleToTurnToFaceTag = -1 * (180 + angleToTurnToFaceTag + Math.toDegrees(Math.atan2(result.get().estimatedPose.getY(), result.get().estimatedPose.getX())));
					angleToTurnToFaceTag = -1 * (180 + angleToTurnToFaceTag + Math.toDegrees(Math.atan2(result.get().estimatedPose.getY(), result.get().estimatedPose.getX())));

					//System.out.println("angle to face: " + (-1 * (180 + angleToTurnToFaceTag + Math.toDegrees(Math.atan2(result.get().estimatedPose.getY(), result.get().estimatedPose.getX())))));
				} else {
					angleToTurnToFaceTag = 180 + angleToTurnToFaceTag - Math.toDegrees(Math.atan2(result.get().estimatedPose.getY(), result.get().estimatedPose.getX()));
					//System.out.println("angle to face: " + (180 + angleToTurnToFaceTag - Math.toDegrees(Math.atan2(result.get().estimatedPose.getY(), result.get().estimatedPose.getX()))));
				}

				//angleToTurnToFaceTag = angleToTurnToFaceTag * -1;
				// angleToTurnToFaceTag = Math.toDegrees(Math.atan2(result.get().estimatedPose.getY(), result.get().estimatedPose.getX()));
				SmartDashboard.putNumber("angle to face: ", angleToTurnToFaceTag);
				SmartDashboard.putNumber("gyro: ", gyroAngleForOdo);


			}
			// System.out.println("gyro " + gyro.getAngle());
			// System.out.println("angle " + angleToTurnToFaceTag);
			// System.out.println("X: " + roboXPos);
			// System.out.println("Y: " + roboYPos);

			rightMotor.set(rightPower);
			leftMotor.set(leftPower);

		} else {
			leftMotor.set((input.getdriveJoystickY()));
			rightMotor.set(-(input.getmechJoystickY()));
		}

	}

	/**
	 * Handle behavior in TELE_STATE_2_MOTOR_DRIVE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleTeleOpBalanceState(TeleopInput input) {
		if (gyro.getPitch() >= -Constants.CHARGING_STATION_LEVELED_ERROR_DEGREES
			&& gyro.getPitch() <= Constants.CHARGING_STATION_LEVELED_ERROR_DEGREES) {
			leftPower = 0;
			rightPower = 0;
		} else {
			leftPower = gyro.getPitch() / Constants.CHARGING_STATION_BALANCE_CONSTANT_PID_P;
			rightPower = -gyro.getPitch() / Constants.CHARGING_STATION_BALANCE_CONSTANT_PID_P;
		}
	}

	/**
	 * Handle behavior in TELE_STATE_CV_ALLIGN.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleCVAllignState(TeleopInput input) {

		System.out.println("angleToTurnToFaceTag: " + angleToTurnToFaceTag);
		handleTurnState(input, (angleToTurnToFaceTag + initialAngle)*.9);
		if (finishedTurning) {
			isAllignedToATag = true;
		}
		// double distToTravelToATag = Math.sqrt(Math.pow(xToATag, 2) + Math.pow(yToATag, 2)) - 15;
		// System.out.println("distToTravelToATag: " + distToTravelToATag);
		// if (Math.sqrt(Math.pow(xToATag, 2) + Math.pow(yToATag, 2)) < distToTravelToATag) {
		// 	leftMotor.set(0.1);
		// 	rightMotor.set(0.1);
		// } else {
		// 	handleTurnState(input, -angleToTurnToFaceTag);
		// }
	}

	/**
	 * Turns the robot to a fixed angle.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 * @param degrees amount of degrees to turn
	 */
	public void handleTurnState(TeleopInput input, double degrees) {
		System.out.println("in method");
		// if (input != null) {
		// 	return;
		// }
		finishedTurning = false;
		System.out.println(getHeading());
		double error = degrees - getHeading();
		if (error > Constants.HALF_REVOLUTION_DEGREES) {
			error -= Constants.ONE_REVOLUTION_DEGREES;
		}
		if (error < -Constants.HALF_REVOLUTION_DEGREES) {
			error += Constants.ONE_REVOLUTION_DEGREES;
		}
		System.out.println("ERROR: " + error);
		if (Math.abs(error) <= Constants.TURN_ERROR_THRESHOLD_DEGREE) {
			System.out.println("DONE");
			leftMotor.set(0);
			rightMotor.set(0);
			finishedTurning = true;
			return;
		}
		double power = Math.abs(error) / Constants.TURN_ERROR_POWER_RATIO;
		if (power < Constants.MIN_TURN_POWER) {
			power = Constants.MIN_TURN_POWER;
		}
		power *= (error < 0 && error > -Constants.HALF_REVOLUTION_DEGREES) ? -0.5 : 0.5;
		// power *= power * 1.3;

		leftMotor.set(-power);
		rightMotor.set(-power);
		// turning right is positive and left is negative
	}

	/**
	* Gets the heading from the gyro.
	* @return the gyro heading
	*/
	public double getHeading() {
		double angle = gyro.getAngle() % Constants.ONE_REVOLUTION_DEGREES;
		if (angle < 0) {
			angle += Constants.ONE_REVOLUTION_DEGREES;
		}
		return angle;
		// angle will be between 0 - 360
	}

	/**
	 * Handle behavior in IDlE State.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void handleIdleState(TeleopInput input) {
		leftMotor.set(0);
		rightMotor.set(0);
	}

	/**
	 * Tracks the robo's position on the field.
	 * @param gyroAngle robot's angle
	 */
	public void updateLineOdometryTele(double gyroAngle) {

		double dEncoder = (currentEncoderPos - prevEncoderPos)
			/ Constants.REVOLUTIONS_PER_INCH;
		double dX = dEncoder * Math.cos(Math.toRadians(gyroAngleForOdo))
			* Constants.DX_INCHES_CONST;
		double dY = dEncoder * Math.sin(Math.toRadians(gyroAngleForOdo))
			* Constants.DY_INCHES_CONST;

		roboXPos += dX;
		roboYPos += dY;

		prevEncoderPos = this.currentEncoderPos;

		// System.out.println("X Pos: " + roboXPos);
		// System.out.println("Y Pos: " + roboYPos);
		// System.out.println("Gyro: " + gyroAngleForOdo);

	}


}
