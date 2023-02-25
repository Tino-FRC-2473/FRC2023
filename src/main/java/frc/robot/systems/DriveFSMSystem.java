package frc.robot.systems;

import com.kauailabs.navx.frc.AHRS;
// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.robot.HardwareMap;
import frc.robot.PhotonCameraWrapper;
// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.drive.DriveFunctions;
import frc.robot.drive.DriveModes;
import frc.robot.drive.DrivePower;
import frc.robot.Constants.VisionConstants;


public class DriveFSMSystem {


	// FSM state definitions
	public enum FSMState {
		TELE_STATE_2_MOTOR_DRIVE,
		TELE_STATE_BALANCE,
		TELE_STATE_HOLD_WHILE_TILTED,
		TELE_STATE_MECANUM,
		PURE_PURSUIT,
		TURNING_STATE,
		CV_LOW_TAPE_ALIGN,
		CV_HIGH_TAPE_ALIGN,
		CV_TAG_ALIGN,
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
		P3N6,

		P4N1,
		P4N2,
		P4N3,
		P4N4,
		P4N5,
		P4N6,
		P4N7,
		P4N8,
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax leftMotorBack;
	private CANSparkMax rightMotorFront;
	private CANSparkMax leftMotorFront;
	private CANSparkMax rightMotorBack;

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

	private boolean isNotForwardEnough = false;
	private boolean isAligned = false;
	private PhotonCameraWrapper pcw = new PhotonCameraWrapper();
	private CvSink cvSink;
	private CvSource outputStrem;
	static final int TURN_RIGHT_OPT = 4;
	static final int TURN_LEFT_OPT = 3;
	static final int MOVE_FORWARD_OPT = 1;
	static final int MOVE_BACKWARD_OPT = 2;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public DriveFSMSystem() {
		// Perform hardware init
		leftMotorBack = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_LEFT_BACK,
										CANSparkMax.MotorType.kBrushless);
		rightMotorFront = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_RIGHT_FRONT,
										CANSparkMax.MotorType.kBrushless);
		leftMotorFront = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_LEFT_FRONT,
										CANSparkMax.MotorType.kBrushless);
		rightMotorBack = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_RIGHT_BACK,
										CANSparkMax.MotorType.kBrushless);

		leftMotorBack.follow(leftMotorFront);
		rightMotorBack.follow(rightMotorFront);

		rightMotorFront.getEncoder().setPosition(0);
		leftMotorBack.getEncoder().setPosition(0);
		rightMotorBack.getEncoder().setPosition(0);
		leftMotorFront.getEncoder().setPosition(0);

		leftPower = 0;
		rightPower = 0;

		finishedTurning = false;

		gyro = new AHRS(SPI.Port.kMXP);

		CameraServer.startAutomaticCapture();
		cvSink = CameraServer.getVideo();
		outputStrem = CameraServer.putVideo("ROBOCAM",
			Constants.WEBCAM_PIXELS_WIDTH, Constants.WEBCAM_PIXELS_WIDTH);
		// Reset state machine
		resetTeleop();

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

		rightMotorFront.getEncoder().setPosition(0);
		leftMotorBack.getEncoder().setPosition(0);
		rightMotorBack.getEncoder().setPosition(0);
		leftMotorFront.getEncoder().setPosition(0);

		gyro.reset();
		gyro.zeroYaw();
		gyroAngleForOdo = 0;

		currentState = FSMState.P1N1;

		roboXPos = 0;
		roboYPos = 0;

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}
	/**
	 * A.
	 */
	public void resetTeleop() {

		rightMotorFront.getEncoder().setPosition(0);
		leftMotorBack.getEncoder().setPosition(0);
		rightMotorBack.getEncoder().setPosition(0);
		leftMotorFront.getEncoder().setPosition(0);

		gyro.reset();
		gyro.zeroYaw();
		gyroAngleForOdo = 0;

		currentState = FSMState.TELE_STATE_2_MOTOR_DRIVE;

		roboXPos = 0;
		roboYPos = 0;

		System.out.println("X: " + roboXPos);
		System.out.println("Y: " + roboYPos);

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
		gyroAngleForOdo = gyro.getAngle() * Constants.GYRO_MULTIPLER_TELOP;

		currentEncoderPos = ((leftMotorBack.getEncoder().getPosition()
			- rightMotorFront.getEncoder().getPosition()) / 2.0);

		updateLineOdometryTele(gyroAngleForOdo);

		switch (currentState) {
			case TELE_STATE_2_MOTOR_DRIVE:
				handleTeleOp2MotorState(input);
				break;

			case CV_LOW_TAPE_ALIGN:
				System.out.println("low");
				handleCVTapeAlignState(true);
				break;

			case CV_HIGH_TAPE_ALIGN:
				System.out.println("high");
				handleCVTapeAlignState(false);
				break;

			case CV_TAG_ALIGN:
				System.out.println("tag");
				handleCVTagAlignState();
				break;

			case TELE_STATE_BALANCE:
				handleTeleOpBalanceState(input);
			case TELE_STATE_HOLD_WHILE_TILTED:
				handleTeleOpHoldWhileTiltedState(input);
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
				moveState(input, false, Constants.P3X4, Constants.P3Y4);
				break;

			case P3N5:
				handleTurnState(input, Constants.P3A5);
				break;

			case P3N6:
				moveState(input, false, Constants.P3X6, Constants.P3Y6);
				break;

			// path 4
			case P4N1:
				handleTurnState(input, Constants.P4A1);
				break;

			case P4N2:
				moveState(input, true, Constants.P4X2,  Constants.P4Y2);
				break;

			case P4N3:
				handleTurnState(input, Constants.P4A3);
				break;

			case P4N4:
				moveState(input, true, Constants.P4X4, Constants.P4Y4);
				break;

			case P4N5:
				moveState(input, false, Constants.P4X5, Constants.P4Y5);
				break;

			case P4N6:
				handleTurnState(input, Constants.P4A6);
				break;

			case P4N7:
				moveState(input, true, 0, Constants.P4Y7);
				break;

			case P4N8:
				handleTurnState(input, Constants.P4A6);
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
				} else if (input != null && input.isSteeringWheelHoldPressedRaw()) {
					return FSMState.TELE_STATE_HOLD_WHILE_TILTED;
				}
				return getCVState(input);
			case TELE_STATE_MECANUM: return FSMState.TELE_STATE_MECANUM;
			case TURNING_STATE:
				if (finishedTurning) {
					return FSMState.TELE_STATE_2_MOTOR_DRIVE;
				}
				return FSMState.TURNING_STATE;
			case CV_LOW_TAPE_ALIGN:
				if (!input.isDriveJoystickCVLowTapeButtonPressedRaw()) {
					return FSMState.TELE_STATE_2_MOTOR_DRIVE;
				}
				return FSMState.CV_LOW_TAPE_ALIGN;
			case CV_HIGH_TAPE_ALIGN:
				if (!input.isDriveJoystickCVHighTapeButtonPressedRaw()) {
					return FSMState.TELE_STATE_2_MOTOR_DRIVE;
				}
				return FSMState.CV_HIGH_TAPE_ALIGN;
			case CV_TAG_ALIGN:
				if (!input.isDriveJoystickCVTagButtonPressedRaw()) {
					return FSMState.TELE_STATE_2_MOTOR_DRIVE;
				}
				return FSMState.CV_TAG_ALIGN;
			case IDLE: return FSMState.IDLE;
			case TELE_STATE_BALANCE:
				if (input != null && input.isDriveJoystickEngageButtonPressedRaw()) {
					return FSMState.TELE_STATE_BALANCE;
				}
				return FSMState.TELE_STATE_2_MOTOR_DRIVE;
			case TELE_STATE_HOLD_WHILE_TILTED:
				if ((input != null && input.isSteeringWheelHoldPressedRaw())) {
					return FSMState.TELE_STATE_HOLD_WHILE_TILTED;
				}
				return FSMState.TELE_STATE_2_MOTOR_DRIVE;
			case P1N1:
				if (Math.abs(roboX - Constants.P1X1) <= Constants.AUTONOMUS_MOVE_THRESHOLD
					&& Math.abs(roboY) <= Constants.AUTONOMUS_MOVE_THRESHOLD) {
					return FSMState.P1N2;
				}
				return FSMState.P1N1;
			case P1N2:
				if (Math.abs(roboX - Constants.P1X2) <= Constants.AUTONOMUS_MOVE_THRESHOLD
					&& Math.abs(roboY) <= Constants.AUTONOMUS_MOVE_THRESHOLD) {
					return FSMState.P1N3;
				}
				return FSMState.P1N2;
			case P1N3:
				if (Math.abs(roboX - Constants.P1X3) <= Constants.AUTONOMUS_MOVE_THRESHOLD
					&& Math.abs(roboY) <= Constants.AUTONOMUS_MOVE_THRESHOLD) {
					return null;
				}
				return FSMState.P1N3;
			case P2N1:
				if (Math.abs(roboX - Constants.P2X1) <= Constants.AUTONOMUS_MOVE_THRESHOLD
					&& Math.abs(roboY) <= Constants.AUTONOMUS_MOVE_THRESHOLD) {
					return FSMState.P2N2;
				}
				return FSMState.P2N1;
			case P2N2:
				if (Math.abs(roboX - Constants.P2X2) <= Constants.AUTONOMUS_MOVE_THRESHOLD
					&& Math.abs(roboY) <= Constants.AUTONOMUS_MOVE_THRESHOLD) {
					return null;
				}
				return FSMState.P2N2;
			case P3N1:
				if (Math.abs(roboX - Constants.P3X1) <= Constants.AUTONOMUS_MOVE_THRESHOLD
					&& Math.abs(roboY) <= Constants.AUTONOMUS_MOVE_THRESHOLD) {
					return FSMState.P3N2;
				}
				return FSMState.P3N1;
			case P3N2:
				if (Math.abs(roboX - Constants.P3X2) <= Constants.AUTONOMUS_MOVE_THRESHOLD
					&& Math.abs(roboY) <= Constants.AUTONOMUS_MOVE_THRESHOLD) {
					return FSMState.P3N3;
				}
				return FSMState.P3N2;
			case P3N3:
				if (finishedTurning) {
					return FSMState.P3N4;
				}
				return FSMState.P3N3;
			case P3N4:
				if (Math.abs(roboX - Constants.P3X4) <= Constants.AUTONOMUS_MOVE_THRESHOLD
					&& Math.abs(roboY - Constants.P3Y4) <= Constants.AUTONOMUS_MOVE_THRESHOLD) {
					return FSMState.P3N5;
				}
				return FSMState.P3N4;
			case P3N5:
				if (finishedTurning) {
					return FSMState.P3N6;
				}
				return FSMState.P3N5;
			case P3N6:
				if (Math.abs(roboX - Constants.P3X6) <= Constants.AUTONOMUS_MOVE_THRESHOLD
					&& Math.abs(roboY - Constants.P3Y6) <= Constants.AUTONOMUS_MOVE_THRESHOLD) {
					return null;
				}
				return FSMState.P3N6;
			case P4N1:
				if (finishedTurning) {
					return FSMState.P4N2;
				}
				return FSMState.P4N1;
			case P4N2:
				if (Math.abs(roboX - Constants.P4X2) <= Constants.AUTONOMUS_MOVE_THRESHOLD
					&& Math.abs(roboY - Constants.P4Y2) <= Constants.AUTONOMUS_MOVE_THRESHOLD) {
					return FSMState.P4N3;
				}
				return FSMState.P4N2;
			case P4N3:
				if (finishedTurning) {
					return FSMState.P4N4;
				}
				return FSMState.P4N3;
			case P4N4:
				if (Math.abs(roboX - Constants.P4X4) <= Constants.AUTONOMUS_MOVE_THRESHOLD
					&& Math.abs(roboY - Constants.P4Y4) <= Constants.AUTONOMUS_MOVE_THRESHOLD) {
					return FSMState.P4N5;
				}
				return FSMState.P4N4;
			case P4N5:
				if (Math.abs(roboX - Constants.P4X5) <= Constants.AUTONOMUS_MOVE_THRESHOLD
					&& Math.abs(roboY - Constants.P4Y5) <= Constants.AUTONOMUS_MOVE_THRESHOLD) {
					return FSMState.P4N6;
				}
				return FSMState.P4N5;
			case P4N6:
				if (finishedTurning) {
					return FSMState.P4N7;
				}
				return FSMState.P3N6;
			case P4N7:
				if (Math.abs(roboX) <= Constants.AUTONOMUS_MOVE_THRESHOLD
					&& Math.abs(roboY - Constants.P4Y7) <= Constants.AUTONOMUS_MOVE_THRESHOLD) {
					return FSMState.P4N8;
				}
				return FSMState.P4N7;
			case P4N8:
				return finishedTurning ? null : FSMState.P4N8;
			default: throw new IllegalStateException("Invalid state: " + currentState.toString()); }
	}

	private FSMState getCVState(TeleopInput input) {
		if (input != null && input.isDriveJoystickCVLowTapeButtonPressedRaw()) {
			isAligned = false; isNotForwardEnough = true; return FSMState.CV_LOW_TAPE_ALIGN;
		} else if (input != null && input.isDriveJoystickCVHighTapeButtonPressedRaw()) {
			isAligned = false; isNotForwardEnough = true;
			return FSMState.CV_HIGH_TAPE_ALIGN;
		} else if (input != null && input.isDriveJoystickCVTagButtonPressedRaw()) {
			isAligned = false; isNotForwardEnough = true; return FSMState.CV_TAG_ALIGN;
		}
		return FSMState.TELE_STATE_2_MOTOR_DRIVE;
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

			currentEncoderPos = ((leftMotorFront.getEncoder().getPosition()
				- rightMotorFront.getEncoder().getPosition()) / 2.0);

			double steerAngle = input.getSteerAngle();
			double currentLeftPower = leftMotorFront.get();
			double currentRightPower = rightMotorFront.get();


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

			System.out.println("X: " + roboXPos);
			System.out.println("Y: " + roboYPos);

			rightMotorFront.set(rightPower);
			leftMotorFront.set(leftPower);

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


		rightMotorFront.set(rightPower);
		leftMotorFront.set(-leftPower);
	}

	/**
	 * Handle behavior in TELE_STATE_2_MOTOR_DRIVE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleTeleOpHoldWhileTiltedState(TeleopInput input) {

		if (Constants.HALF_REVOLUTION_DEGREES - Math.abs(gyro.getRoll())
			< Constants.CHARGING_STATION_LEVELED_ERROR_DEGREES && Constants.HALF_REVOLUTION_DEGREES
			- Math.abs(gyro.getRoll()) > -Constants.CHARGING_STATION_LEVELED_ERROR_DEGREES) {
			leftPower = 0;
			rightPower = 0;
		} else if (gyro.getRoll() > 0) {
			leftPower = Constants.POWER_TO_HOLD_ROBO_ON_TILTED_CS;
			rightPower = Constants.POWER_TO_HOLD_ROBO_ON_TILTED_CS;
		} else if (gyro.getRoll() < 0) {
			leftPower = -Constants.POWER_TO_HOLD_ROBO_ON_TILTED_CS;
			rightPower = -Constants.POWER_TO_HOLD_ROBO_ON_TILTED_CS;
		}


		rightMotorFront.set(rightPower);
		leftMotorFront.set(-leftPower);
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
			finishedTurning = true;
			leftMotorBack.set(0);
			rightMotorBack.set(0);
			leftMotorFront.set(0);
			rightMotorBack.set(0);
			return;
		}
		double power = Math.abs(error) / Constants.TURN_ERROR_POWER_RATIO;
		if (power < Constants.MIN_TURN_POWER) {
			power = Constants.MIN_TURN_POWER;
		}
		power *= ((error < 0 && error > -Constants.HALF_REVOLUTION_DEGREES) ? -1 : 1);

		leftMotorFront.set(-power);
		rightMotorFront.set(-power);
		// turning right is positive and left is negative
	}

	/**
	 * Handle behavior in IDlE State.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void handleIdleState(TeleopInput input) {
		leftMotorBack.set(0);
		rightMotorBack.set(0);
		leftMotorFront.set(0);
		rightMotorBack.set(0);
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
			leftMotorFront.set(-Constants.AUTONOMUS_MOVE_POWER);
			rightMotorFront.set(Constants.AUTONOMUS_MOVE_POWER);
		} else {
			leftMotorFront.set(Constants.AUTONOMUS_MOVE_POWER);
			rightMotorFront.set(-Constants.AUTONOMUS_MOVE_POWER);
		}
		if (Math.abs(roboX - x) <= Constants.AUTONOMUS_MOVE_THRESHOLD
			&& Math.abs(roboY - y) <= Constants.AUTONOMUS_MOVE_THRESHOLD) {
			leftMotorFront.set(0);
			rightMotorFront.set(0);
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

		//System.out.println("X Pos: " + roboXPos);
		//System.out.println("Y Pos: " + roboYPos);
		//System.out.println("Gyro: " + gyroAngleForOdo);

	}

	/**.
 	* Aligns with reflective tape (higher or lower tape is dependent on boolean passed in)
		and drives within 42 inches (lower) or 65 inches (higher)
		@param lower lower of higher tape
	*/
	public void handleCVTapeAlignState(boolean lower) {
		double angle;
		if (lower) {
			angle = pcw.getLowerTapeTurnAngle();
			isNotForwardEnough =  pcw.getLowerTapeDistance()
				> Constants.LOWER_TAPE_DRIVEUP_DISTANCE_INCHES;
			System.out.println("distance: " + pcw.getLowerTapeDistance());
			//drives forward until within 42 inches of lower tape
		} else {
			angle = pcw.getHigherTapeTurnAngle();
			isNotForwardEnough = pcw.getHigherTapeDistance()
				> Constants.HIGHER_TAPE_DRIVEUP_DISTANCE_INCHES;
			System.out.println("distance: " + pcw.getHigherTapeDistance());
			//drives forward until within 65 inches of higher tape
		}
		System.out.println("angle: " + angle);
		if (angle == Constants.INVALID_TURN_RETURN_DEGREES) {
			return;
		}
		if (angle > Constants.ANGLE_TO_TARGET_THRESHOLD_DEGREES) {
			cvmove(TURN_RIGHT_OPT);
		} else if (angle  < -Constants.ANGLE_TO_TARGET_THRESHOLD_DEGREES) {
			cvmove(TURN_LEFT_OPT);
		} else {
			isAligned = true;
			if (isNotForwardEnough) {
				cvmove(MOVE_FORWARD_OPT);
			} else {
				cvmove(0);
			}
		}

	}

	/**.
 	* Aligns to april tag and drives up to within 35 inches of it
	*/
	public void handleCVTagAlignState() {
		System.out.println("TAG");
		double angle = pcw.getTagTurnAngle();
		if (angle == Constants.INVALID_TURN_RETURN_DEGREES) {
			return;
		}
		isNotForwardEnough =  pcw.getTagDistance() > Constants.TAG_DRIVEUP_DISTANCE_INCHES;
		System.out.println(pcw.getTagDistance());
		System.out.println(pcw.getTagTurnAngle());
		if (angle > Constants.ANGLE_TO_TARGET_THRESHOLD_DEGREES) {
			cvmove(TURN_RIGHT_OPT);
		} else if (angle  < -Constants.ANGLE_TO_TARGET_THRESHOLD_DEGREES) {
			cvmove(TURN_LEFT_OPT);
		} else {
			isAligned = true;
			if (isNotForwardEnough) {
				cvmove(MOVE_FORWARD_OPT);
			} else {
				cvmove(0);
			}
		}
	}
	/**
	 * Aligns to the high cube node (the one without an april tag).
	 */
	public void handleMidCubeNodeAlignState() {
		pcw.setPipelineIndex(VisionConstants.THREEDTAG_PIPELINE_INDEX); //3d pipeline
		System.out.println("CUBE NODE");
		double x = pcw.getEstimatedGlobalPose().getX();
		double y = pcw.getEstimatedGlobalPose().getY();
		double angle = pcw.getEstimatedGlobalPose().getRotation().getAngle();
		isNotForwardEnough = x > Units.inchesToMeters(Constants.TAG_DRIVEUP_DISTANCE_INCHES);
		if (Math.abs(angle) < Constants.ANGLE_TO_TARGET_THRESHOLD_DEGREES) {
			isAligned = true;
			if (isNotForwardEnough) {
				cvmove(1);
				x = pcw.getEstimatedGlobalPose().getX();
				System.out.println("pose x: " + pcw.getEstimatedGlobalPose().getX());
			} else {
				cvmove(0);
				pcw.setPipelineIndex(VisionConstants.TWODTAG_PIPELINE_INDEX); //2d pipeline
				double midAngle = Math.atan(y / (x + Constants.APRILTAG_TO_HIGH_CUBENODE_METERS));
				if (midAngle > Constants.ANGLE_TO_TARGET_THRESHOLD_DEGREES) {
					cvmove(TURN_RIGHT_OPT);
				} else if (midAngle < -Constants.ANGLE_TO_TARGET_THRESHOLD_DEGREES) {
					cvmove(TURN_LEFT_OPT);
				}
			}
		} else {
			if (angle > Constants.ANGLE_TO_TARGET_THRESHOLD_DEGREES) {
				cvmove(TURN_RIGHT_OPT);
			} else if (angle < -Constants.ANGLE_TO_TARGET_THRESHOLD_DEGREES) {
				cvmove(TURN_LEFT_OPT);
			}
		}
	}
	/**
	 * Basic moving commands for CV.
	 * @param opt how you want to move: forward, backwards, turn left, turn right, or idle.
	 */
	public void cvmove(int opt) {
		switch (opt) {
			//stop
			case 0:
				leftMotorFront.set(0);
				rightMotorFront.set(0);
				leftMotorBack.set(0);
				rightMotorBack.set(0);
				break;
			//forward
			case MOVE_FORWARD_OPT:
				leftMotorFront.set(-Constants.CV_FORWARD_POWER);
				rightMotorFront.set(Constants.CV_FORWARD_POWER);
				leftMotorBack.set(-Constants.CV_FORWARD_POWER);
				rightMotorBack.set(Constants.CV_FORWARD_POWER);
				break;
			//backward
			case MOVE_BACKWARD_OPT:
				leftMotorFront.set(Constants.CV_FORWARD_POWER);
				rightMotorFront.set(-Constants.CV_FORWARD_POWER);
				leftMotorBack.set(Constants.CV_FORWARD_POWER);
				rightMotorBack.set(-Constants.CV_FORWARD_POWER);
				break;
			//turn left
			case TURN_LEFT_OPT:
				leftMotorFront.set(Constants.CV_TURN_POWER);
				rightMotorFront.set(Constants.CV_TURN_POWER);
				leftMotorBack.set(Constants.CV_TURN_POWER);
				rightMotorBack.set(Constants.CV_TURN_POWER);
				break;
			//turn right
			case TURN_RIGHT_OPT:
				leftMotorFront.set(-Constants.CV_TURN_POWER);
				rightMotorFront.set(-Constants.CV_TURN_POWER);
				leftMotorBack.set(-Constants.CV_TURN_POWER);
				rightMotorBack.set(-Constants.CV_TURN_POWER);
				break;
			default:
				System.out.println("ERROR");
				break;
		}
	}
}
