package frc.robot.systems;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.SPI;

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
		TELE_STATE_CV_ALIGN,
		TELE_STATE_MECANUM,
		PURE_PURSUIT,
		TURNING_STATE,
		IDLE,
		P1N1,
		P1N2,
		P1N3,

		P2N1,
		P2N2,

		P3N1,
		P3N2,
		P3N3,
		P3N4,
		P3N5,
		P3N6
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax leftMotor1;
	private CANSparkMax rightMotor1;
	private CANSparkMax leftMotor2;
	private CANSparkMax rightMotor2;

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

	private DrivePoseEstimator dpe = new DrivePoseEstimator();
	private PhotonCameraWrapper pcw = new PhotonCameraWrapper();
	private double xToATag = 0;
	private double yToATag = 0;
	private boolean isAlignedToATag = false;


	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public DriveFSMSystem() {
		// Perform hardware init
		leftMotor1 = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_LEFT1,
										CANSparkMax.MotorType.kBrushless);
		rightMotor1 = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_RIGHT1,
										CANSparkMax.MotorType.kBrushless);
		leftMotor2 = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_LEFT2,
										CANSparkMax.MotorType.kBrushless);
		rightMotor2 = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_RIGHT2,
										CANSparkMax.MotorType.kBrushless);

		rightMotor1.getEncoder().setPosition(0);
		leftMotor1.getEncoder().setPosition(0);
		rightMotor2.getEncoder().setPosition(0);
		leftMotor2.getEncoder().setPosition(0);

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

		rightMotor1.getEncoder().setPosition(0);
		leftMotor1.getEncoder().setPosition(0);
		rightMotor2.getEncoder().setPosition(0);
		leftMotor2.getEncoder().setPosition(0);

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

		rightMotor1.getEncoder().setPosition(0);
		leftMotor1.getEncoder().setPosition(0);
		rightMotor2.getEncoder().setPosition(0);
		leftMotor2.getEncoder().setPosition(0);

		gyro.reset();
		gyro.zeroYaw();
		gyroAngleForOdo = 0;

		currentState = FSMState.TELE_STATE_2_MOTOR_DRIVE;

		roboXPos = 0;
		roboYPos = 0;

		// System.out.println("X: " + roboXPos);
		// System.out.println("Y: " + roboYPos);

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
		// dpe.updatePose(gyro.getAngle(), leftMotor1.getEncoder().getPosition(),
		// 	rightMotor1.getEncoder().getPosition());

		// if (!pcw.getEstimatedGlobalPose().isEmpty()) {
		// 	SmartDashboard.putNumber("X",
		// 		Units.metersToInches(pcw.getEstimatedGlobalPose().get().estimatedPose.getX()));
		// 	SmartDashboard.putNumber("Y",
		// 		Units.metersToInches(pcw.getEstimatedGlobalPose().get().estimatedPose.getY()));
		// 	SmartDashboard.putNumber("Rotation", Constants.ONE_REVOLUTION_DEGREES
		// 		- Units.radiansToDegrees(
		// 		pcw.getEstimatedGlobalPose().get().estimatedPose.getRotation().getAngle()));
		// 	SmartDashboard.putNumber("Rotation2",
		// 		pcw.getEstimatedGlobalPose().get().estimatedPose.getRotation().getAngle());
		// }
		gyroAngleForOdo = gyro.getAngle() * Constants.GYRO_MULTIPLER_TELOP;

		currentEncoderPos = ((leftMotor1.getEncoder().getPosition()
			- rightMotor1.getEncoder().getPosition()) / 2.0);

		updateLineOdometryTele(gyroAngleForOdo);

		// System.out.println(gyroAngleForOdo);
		// System.out.println("Velocity: " + Math.sqrt(Math.abs(Math.pow(gyro.getVelocityX(), 2))
		// 	+ Math.abs(Math.pow(gyro.getVelocityY(), 2))));

		switch (currentState) {
			case TELE_STATE_2_MOTOR_DRIVE:
				handleTeleOp2MotorState(input);
				break;

			case TELE_STATE_BALANCE:
				handleTeleOpBalanceState(input);
				break;

			case TELE_STATE_CV_ALIGN:
				handleCVAlignState(input);
				break;

			case IDLE:
				handleIdleState(input);
				break;

			case TURNING_STATE:
				handleTurnState(input, Constants.HALF_REVOLUTION_DEGREES);
				break;

			// path 1

			case P1N1:
				moveState(input, true, Constants.P1X1, 0);
				break;

			case P1N2:
				moveState(input, false, Constants.P1X2, 0);
				break;

			case P1N3:
				moveState(input, true, Constants.P1X3, 0);
				break;

			// path 2

			case P2N1:
				moveState(input, true, Constants.P2X1, 0);
				break;

			case P2N2:
				moveState(input, false, Constants.P2X2, 0);
				break;

			// path 3

			case P3N1:
				moveState(input, true, Constants.P3X1, 0);
				break;

			case P3N2:
				moveState(input, false, Constants.P3X2, 0);
				break;

			case P3N3:
				handleTurnState(input, Constants.P3A3);
				break;

			case P3N4:
				moveState(input, true, Constants.P3X4, Constants.P3Y4);
				break;

			case P3N5:
				handleTurnState(input, Constants.P3A5);
				break;

			case P3N6:
				moveState(input, true, Constants.P3X6, Constants.P3Y6);
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
		double roboX = -roboXPos;
		double roboY = roboYPos;

		switch (currentState) {

			case TELE_STATE_2_MOTOR_DRIVE:
				if (input != null && input.isDriveJoystickEngageButtonPressedRaw()) {
					return FSMState.TELE_STATE_BALANCE;
				} else if (input != null && input.isDriveJoystickCVAlignLeftButtonPressedRaw()) {
					// Align to node left of april tag
					return FSMState.TELE_STATE_CV_ALIGN;
				}
				isAlignedToATag = false;
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

			case TELE_STATE_CV_ALIGN:
				if (isAlignedToATag) {
					return FSMState.TELE_STATE_2_MOTOR_DRIVE;
				}
				return FSMState.TELE_STATE_CV_ALIGN;

			case TELE_STATE_BALANCE:
				if (input != null && input.isDriveJoystickEngageButtonPressedRaw()) {
					return FSMState.TELE_STATE_BALANCE;
				}
				return FSMState.TELE_STATE_2_MOTOR_DRIVE;

			case IDLE:
				return FSMState.IDLE;

			// path 1
			case P1N1:
				if (Math.abs(roboX - Constants.P1X1) <= Constants.AUTONOMUS_MOVE_THRESHOLD
					&& Math.abs(roboY) <= Constants.AUTONOMUS_MOVE_THRESHOLD) {
					return FSMState.P1N2;
				} else {
					return FSMState.P1N1;
				}

			case P1N2:
				if (Math.abs(roboX - Constants.P1X2) <= Constants.AUTONOMUS_MOVE_THRESHOLD
					&& Math.abs(roboY) <= Constants.AUTONOMUS_MOVE_THRESHOLD) {
					return FSMState.P1N3;
				} else {
					return FSMState.P1N2;
				}

			case P1N3:
				if (Math.abs(roboX - Constants.P1X3) <= Constants.AUTONOMUS_MOVE_THRESHOLD
					&& Math.abs(roboY) <= Constants.AUTONOMUS_MOVE_THRESHOLD) {
					return null;
				} else {
					return FSMState.P1N3;
				}

			// path 2

			case P2N1:
				if (Math.abs(roboX - Constants.P2X1) <= Constants.AUTONOMUS_MOVE_THRESHOLD
					&& Math.abs(roboY) <= Constants.AUTONOMUS_MOVE_THRESHOLD) {
					return FSMState.P2N2;
				} else {
					return FSMState.P2N1;
				}

			case P2N2:
				if (Math.abs(roboX - Constants.P2X2) <= Constants.AUTONOMUS_MOVE_THRESHOLD
					&& Math.abs(roboY) <= Constants.AUTONOMUS_MOVE_THRESHOLD) {
					return null;
				} else {
					return FSMState.P2N2;
				}

			// path 3

			case P3N1:
				if (Math.abs(roboX - Constants.P3X1) <= Constants.AUTONOMUS_MOVE_THRESHOLD
					&& Math.abs(roboY) <= Constants.AUTONOMUS_MOVE_THRESHOLD) {
					return FSMState.P3N2;
				} else {
					return FSMState.P3N1;
				}

			case P3N2:
				if (Math.abs(roboX - Constants.P3X2) <= Constants.AUTONOMUS_MOVE_THRESHOLD
					&& Math.abs(roboY) <= Constants.AUTONOMUS_MOVE_THRESHOLD) {
					return FSMState.P3N3;
				} else {
					return FSMState.P3N2;
				}

			case P3N3:
				if (finishedTurning) {
					return FSMState.P3N4;
				} else {
					return FSMState.P3N3;
				}

			case P3N4:
				if (Math.abs(roboX - Constants.P3X4) <= Constants.AUTONOMUS_MOVE_THRESHOLD
					&& Math.abs(roboY - Constants.P3Y4) <= Constants.AUTONOMUS_MOVE_THRESHOLD) {
					return FSMState.P3N5;
				} else {
					return FSMState.P3N4;
				}

			case P3N5:
				if (finishedTurning) {
					return FSMState.P3N6;
				} else {
					return FSMState.P3N5;
				}

			case P3N6:
				if (Math.abs(roboX - Constants.P3X6) <= Constants.AUTONOMUS_MOVE_THRESHOLD
					&& Math.abs(roboY - Constants.P3Y6) <= Constants.AUTONOMUS_MOVE_THRESHOLD) {
					return null;
				} else {
					return FSMState.P3N6;
				}

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

		// Pose3d cvEstimatedPos = pcw.getEstimatedGlobalPose().get().estimatedPose;

		if (isInArcadeDrive) {

			currentEncoderPos = ((leftMotor1.getEncoder().getPosition()
				- rightMotor1.getEncoder().getPosition()) / 2.0);

			// updateLineOdometryTele(gyroAngleForOdo);

			double steerAngle = input.getSteerAngle();
			double currentLeftPower = leftMotor1.get();
			double currentRightPower = rightMotor1.get();


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

			// if (!pcw.getEstimatedGlobalPose().isEmpty()) {
			// 	// left is negative right is positive
			// 	angleToTurnToFaceTag = Math.abs((Constants.HALF_REVOLUTION_DEGREES
			// 		+ Constants.ONE_REVOLUTION_DEGREES - Math.toDegrees(
			// 		cvEstimatedPos.getRotation().getAngle())) + Math.toDegrees(
			// 			Math.atan2(cvEstimatedPos.getY(),
			// 			cvEstimatedPos.getX())));

			// 	if (cvEstimatedPos.getY() >= 0) {
			// 		angleToTurnToFaceTag = -angleToTurnToFaceTag;
			// 	}

			// 	SmartDashboard.putNumber("angle to face: ", angleToTurnToFaceTag);
			// 	SmartDashboard.putNumber("gyro: ", gyroAngleForOdo);

			// }

			System.out.println("X: " + roboXPos);
			System.out.println("Y: " + roboYPos);

			leftMotor1.set(leftPower);
			rightMotor1.set(rightPower);
			leftMotor2.set(leftPower);
			rightMotor2.set(rightPower);

		} else {
			leftMotor1.set((input.getdriveJoystickY()));
			rightMotor1.set(-(input.getmechJoystickY()));
			leftMotor2.set((input.getdriveJoystickY()));
			rightMotor2.set(-(input.getmechJoystickY()));
		}

	}

	/**
	 * Handle behavior in TELE_STATE_2_MOTOR_DRIVE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleTeleOpBalanceState(TeleopInput input) {

		if (Constants.HALF_REVOLUTION_DEGREES - Math.abs(gyro.getRoll())
			< Constants.CHARGING_STATION_LEVELED_ERROR_DEGREES && Constants.HALF_REVOLUTION_DEGREES
			- Math.abs(gyro.getRoll()) > -Constants.CHARGING_STATION_LEVELED_ERROR_DEGREES) {
			leftPower = 0;
			rightPower = 0;
		} else if (gyro.getRoll() > 0) {
			leftPower = (Constants.HALF_REVOLUTION_DEGREES - Math.abs(gyro.getRoll()))
				/ Constants.CHARGING_STATION_BALANCE_CONSTANT_PID_P;
			rightPower = (Constants.HALF_REVOLUTION_DEGREES - Math.abs(gyro.getRoll()))
				/ Constants.CHARGING_STATION_BALANCE_CONSTANT_PID_P;
		} else if (gyro.getRoll() < 0) {
			leftPower = -(Constants.HALF_REVOLUTION_DEGREES - Math.abs(gyro.getRoll()))
				/ Constants.CHARGING_STATION_BALANCE_CONSTANT_PID_P;
			rightPower = -(Constants.HALF_REVOLUTION_DEGREES - Math.abs(gyro.getRoll()))
				/ Constants.CHARGING_STATION_BALANCE_CONSTANT_PID_P;
		}


		leftMotor1.set(-leftPower);
		rightMotor1.set(rightPower);
		leftMotor2.set(-leftPower);
		rightMotor2.set(rightPower);
	}

	/**
	 * Handle behavior in TELE_STATE_CV_ALIGN.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleCVAlignState(TeleopInput input) {

		// xToATag = Units.metersToInches(pcw.getEstimatedGlobalPose().get().estimatedPose.getX());
		// yToATag = Units.metersToInches(pcw.getEstimatedGlobalPose().get().estimatedPose.getX());

		// System.out.println("angleToTurnToFaceTag: " + angleToTurnToFaceTag);
		// System.out.println("in method");
		// isAlignedToATag = true;
		// handleTurnState(input, 90);
		// double distToTravelToATag = Math.sqrt(Math.pow(xToATag, 2) + Math.pow(yToATag, 2)) - 10;
		// System.out.println("distToTravelToATag: " + distToTravelToATag);
		// if (Math.sqrt(Math.pow(xToATag, 2) + Math.pow(yToATag, 2)) < distToTravelToATag) {
		// 	leftMotor.set(0.1);
		// 	rightMotor.set(0.1);
		// } else {
		// 	handleTurnState(input, -angleToTurnToFaceTag);
		// }
	}

	/**
	 * Handle behavior in TURNING_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 * @param degrees How many degrees the robot is to turn
	 */
	public void handleTurnState(TeleopInput input, double degrees) {
		if (input != null) {
			return;
		}
		finishedTurning = false;
		// System.out.println(getHeading());
		double error = degrees - getHeading();
		// if (error > Constants.HALF_REVOLUTION_DEGREES) {
		// 	error -= Constants.ONE_REVOLUTION_DEGREES;
		// }
		// if (error < -Constants.HALF_REVOLUTION_DEGREES) {
		// 	error += Constants.ONE_REVOLUTION_DEGREES;
		// }

		if (error > Constants.HALF_REVOLUTION_DEGREES) {
			error -= Constants.ONE_REVOLUTION_DEGREES;
		}
		if (error < -Constants.HALF_REVOLUTION_DEGREES) {
			error += Constants.ONE_REVOLUTION_DEGREES;
		}
		System.out.println("ERROR: " + error);
		if (Math.abs(error) <= Constants.TURN_ERROR_THRESHOLD_DEGREE) {
			System.out.println("DONE");
			finishedTurning = true;
			leftMotor1.set(0);
			rightMotor2.set(0);
			leftMotor2.set(0);
			rightMotor2.set(0);
			return;
		}
		double power = Math.abs(error) / Constants.TURN_ERROR_POWER_RATIO;
		if (power < Constants.MIN_TURN_POWER) {
			power = Constants.MIN_TURN_POWER;
		}
		power *= ((error < 0 && error > -Constants.HALF_REVOLUTION_DEGREES) ? -1 : 1);

		leftMotor1.set(-power);
		rightMotor2.set(-power);
		leftMotor2.set(-power);
		rightMotor2.set(-power);
		// turning right is positive and left is negative
	}

	// /**
	//  * Handle behavior in TURNING_STATE.
	//  * @param input Global TeleopInput if robot in teleop mode or null if
	//  *        the robot is in autonomous mode.
	//  * @param degrees How many degrees the robot is to turn
	//  */
	// public void handleTurnState(TeleopInput input, double degrees) {
	// 	// if (input != null) {
	// 	// 	return;
	// 	// }

	// 	degrees *= Constants.GYRO_TURN_MULTIPLER_BELOW_90;

	// 	System.out.println("get heading: " + getHeading());
	// 	double error = degrees - getHeading();
	// 	if (error > Constants.HALF_REVOLUTION_DEGREES) {
	// 		error -= Constants.ONE_REVOLUTION_DEGREES;
	// 	}
	// 	if (Math.abs(error) <= Constants.TURN_ERROR_THRESHOLD_DEGREE) {
	// 		finishedTurning = true;
	// 		leftMotor1.set(0);
	// 		rightMotor2.set(0);
	// 		leftMotor2.set(0);
	// 		rightMotor2.set(0);
	// 		return;
	// 	}
	// 	double power = Math.abs(error) / Constants.TURN_ERROR_POWER_RATIO;
	// 	if (power < Constants.MIN_TURN_POWER) {
	// 		power = Constants.MIN_TURN_POWER;
	// 	}

	// 	power *= ((error < 0 && error > -Constants.HALF_REVOLUTION_DEGREES) ? -1 : 1) / 2;

	// 	leftMotor1.set(-power);
	// 	rightMotor2.set(-power);
	// 	leftMotor2.set(-power);
	// 	rightMotor2.set(-power);
	// }

	/**
	 * Handle behavior in IDlE State.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void handleIdleState(TeleopInput input) {
		leftMotor1.set(0);
		rightMotor2.set(0);
		leftMotor2.set(0);
		rightMotor2.set(0);
	}

	// /**
	// * Gets the heading from the gyro.
	// * @return the gyro heading
	// */
	// public double getHeading() {
	// 	// double angle = startAngle - gyro.getYaw();
	// 	double angle = startAngle - gyro.getAngle();
	// 	if (angle < 0) {
	// 		angle += Constants.ONE_REVOLUTION_DEGREES;
	// 	}
	// 	if (angle > Constants.ONE_REVOLUTION_DEGREES) {
	// 		angle -= Constants.ONE_REVOLUTION_DEGREES;
	// 	}
	// 	return angle;
	// }

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
	 * .
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 * @param forwards whether the robot is moving forwards or backwards
	 * @param x x position of goal point
	 * @param y y position of goal point
	 */
	public void moveState(TeleopInput input, boolean forwards, double x, double y) {
		if (input != null) {
			return;
		}
		double roboX = -roboXPos;
		double roboY = roboYPos;
		System.out.println("x: " + roboX);
		System.out.println("y: " + roboY);

		if (forwards) {
			leftMotor1.set(-Constants.AUTONOMUS_MOVE_POWER);
			rightMotor2.set(Constants.AUTONOMUS_MOVE_POWER);
			leftMotor2.set(-Constants.AUTONOMUS_MOVE_POWER);
			rightMotor1.set(Constants.AUTONOMUS_MOVE_POWER);
		} else {
			leftMotor1.set(Constants.AUTONOMUS_MOVE_POWER);
			rightMotor2.set(-Constants.AUTONOMUS_MOVE_POWER);
			leftMotor2.set(Constants.AUTONOMUS_MOVE_POWER);
			rightMotor1.set(-Constants.AUTONOMUS_MOVE_POWER);
		}
		if (Math.abs(roboX - x) <= Constants.AUTONOMUS_MOVE_THRESHOLD
			&& Math.abs(roboY - y) <= Constants.AUTONOMUS_MOVE_THRESHOLD) {
			leftMotor1.set(0);
			rightMotor2.set(0);
			leftMotor2.set(0);
			rightMotor1.set(0);
		}
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
	}


}
