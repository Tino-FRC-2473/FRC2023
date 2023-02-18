package frc.robot.systems;

import com.kauailabs.navx.frc.AHRS;
// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.HardwareMap;
import frc.robot.PhotonCameraWrapper;
// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.drive.DriveFunctions;
import frc.robot.drive.DriveModes;
import frc.robot.drive.DrivePower;


public class DriveFSMSystem {


	// FSM state definitions
	public enum FSMState {
		TELE_STATE_2_MOTOR_DRIVE,
		TELE_STATE_BALANCE,
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
	//be private to their owner system and may not be used elsewhere.
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

	private boolean isForwardEnough = false;
	private boolean isAligned = false;
	private PhotonCameraWrapper pcw = new PhotonCameraWrapper();
	private CvSink cvSink;
	private CvSource outputStrem;

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

		leftMotor1.set(0);
		rightMotor1.set(0);
		leftMotor2.set(0);
		rightMotor2.set(0);

		finishedTurning = false;

		gyro = new AHRS(SPI.Port.kMXP);

		CameraServer.startAutomaticCapture();
		cvSink = CameraServer.getVideo();
		outputStrem = CameraServer.putVideo("ROBOCAM",
			VisionConstants.WEBCAM_PIXELS_WIDTH, VisionConstants.WEBCAM_PIXELS_HEIGHT);
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

		// rightMotor1.getEncoder().setPosition(0);
		// leftMotor1.getEncoder().setPosition(0);
		// rightMotor2.getEncoder().setPosition(0);
		// leftMotor2.getEncoder().setPosition(0);

		gyro.reset();
		gyro.zeroYaw();
		gyroAngleForOdo = 0;

		currentState = FSMState.P2N1;

		roboXPos = 0;
		roboYPos = 0;

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}
	/**
	 * A.
	 */
	public void resetTeleop() {

		// rightMotor1.getEncoder().setPosition(0);
		// leftMotor1.getEncoder().setPosition(0);
		// rightMotor2.getEncoder().setPosition(0);
		// leftMotor2.getEncoder().setPosition(0);

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
		gyroAngleForOdo = gyro.getAngle();

		currentEncoderPos = ((leftMotor1.getEncoder().getPosition()
			- rightMotor1.getEncoder().getPosition()) / 2.0);

		updateLineOdometryTele(gyro.getAngle());

		switch (currentState) {
			case TELE_STATE_2_MOTOR_DRIVE:
				handleTeleOp2MotorState(input);
				break;

			case CV_LOW_TAPE_ALIGN:
				System.out.println("low state");
				handleCVTapeAlignState(true);
				break;

			case CV_HIGH_TAPE_ALIGN:
				System.out.println("high state");
				handleCVTapeAlignState(false);
				break;

			case CV_TAG_ALIGN:
				System.out.println("tag state");
				handleCVTagAlignState();
				break;

			case TELE_STATE_BALANCE:
				handleTeleOpBalanceState(input);
				break;

			case IDLE:
				handleIdleState(input);
				break;

			case TURNING_STATE:
				handleTurnState(input, 0);
				break;

			// path 1

			case P1N1:
				moveState(input, true, Constants.P1X1, 0);
				// set the grabber to be at the low state to drop off block
				// reset the arm to idle state
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
				// set the grabber to be at the low state to drop off block
				// reset the arm to idle state
				break;

			case P2N2:
				moveState(input, false, Constants.P2X2, 0);
				break;

			// path 3

			case P3N1:
				moveState(input, true, Constants.P3X1, 0);
				// set grabber at high height to drop off cube
				break;

			case P3N2:
				moveState(input, false, Constants.P3X2, 0);
				// set grabber at low height to pick up another cube
				break;

			case P3N3:
				handleTurnState(input, Constants.P3A3);
				// set grabnber at mid height to drop off cube
				// reset the arm to idle state
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
				if (input != null && input.isDriveJoystickCVLowTapeButtonPressedRaw()) {
					isAligned = false; isForwardEnough = true; return FSMState.CV_LOW_TAPE_ALIGN;
				} else if (input != null && input.isDriveJoystickCVHighTapeButtonPressedRaw()) {
					isAligned = false; isForwardEnough = true; return FSMState.CV_HIGH_TAPE_ALIGN;
				} else if (input != null && input.isDriveJoystickCVTagButtonPressedRaw()) {
					isAligned = false; isForwardEnough = true; return FSMState.CV_TAG_ALIGN;
				} else {
					return FSMState.TELE_STATE_2_MOTOR_DRIVE;
				}
			case TELE_STATE_MECANUM: return FSMState.TELE_STATE_MECANUM;
			case TURNING_STATE:
				if (finishedTurning) {
					return FSMState.IDLE;
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
		if (Math.abs(error) <= Constants.TURN_ERROR_THRESHOLD_DEGREE) {
			finishedTurning = true;
			leftMotor1.set(0);
			rightMotor1.set(0);
			leftMotor2.set(0);
			rightMotor2.set(0);
			return;
		}
		double power = Math.abs(error) / Constants.TURN_ERROR_POWER_RATIO;
		if (power < Constants.MIN_TURN_POWER) {
			power = Constants.MIN_TURN_POWER;
		}
		power *= ((error < 0 && error > -Constants.HALF_REVOLUTION_DEGREES) ? -1 : 1);

		leftMotor1.set(power);
		rightMotor1.set(power);
		leftMotor2.set(power);
		rightMotor2.set(power);
		// turning right is positive and left is negative
	}

	/**
	 * Handle behavior in IDlE State.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void handleIdleState(TeleopInput input) {
		// leftMotor1.set(0);
		// rightMotor1.set(0);
		// leftMotor2.set(0);
		// rightMotor2.set(0);
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

	/**
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

	/**.
 	* Aligns with reflective tape (higher or lower tape is dependent on boolean passed in)
		and drives within 42 inches (lower) or 65 inches (higher)
		@param lower lower of higher tape
	*/
	public void handleCVTapeAlignState(boolean lower) {
		double angle;
		if (lower) {
			angle = pcw.getLowerTapeTurnAngle();
			isForwardEnough =  pcw.getLowerTapeDistance()
				> VisionConstants.LOWER_TAPE_DRIVEUP_DISTANCE_INCHES;
			//drives forward until within 42 inches of lower tape
		} else {
			angle = pcw.getHigherTapeTurnAngle();
			isForwardEnough = pcw.getHigherTapeDistance()
				> VisionConstants.HIGHER_TAPE_DRIVEUP_DISTANCE_INCHES;
			//drives forward until within 65 inches of higher tape
		}
		if (angle > VisionConstants.ANGLE_TO_TARGET_THRESHOLD_DEGREES) {
			leftMotor1.set(-VisionConstants.CV_TURN_POWER);
			leftMotor2.set(-VisionConstants.CV_TURN_POWER);
			rightMotor1.set(-VisionConstants.CV_TURN_POWER);
			rightMotor2.set(-VisionConstants.CV_TURN_POWER);
		} else if (angle  < -VisionConstants.ANGLE_TO_TARGET_THRESHOLD_DEGREES) {
			leftMotor1.set(VisionConstants.CV_TURN_POWER);
			rightMotor1.set(VisionConstants.CV_TURN_POWER);
			leftMotor2.set(VisionConstants.CV_TURN_POWER);
			rightMotor2.set(VisionConstants.CV_TURN_POWER);
		} else {
			isAligned = true;
			if (isForwardEnough) {
				leftMotor1.set(-VisionConstants.CV_FORWARD_POWER);
				rightMotor1.set(VisionConstants.CV_FORWARD_POWER);
				leftMotor2.set(-VisionConstants.CV_FORWARD_POWER);
				rightMotor2.set(VisionConstants.CV_FORWARD_POWER);
			} else {
				leftMotor1.set(0);
				rightMotor1.set(0);
				leftMotor2.set(0);
				rightMotor2.set(0);
			}
		}

	}

	/**.
 	* Aligns to april tag and drives up to within 35 inches of it
	*/
	public void handleCVTagAlignState() {
		double angle = pcw.getTagTurnAngle();
		isForwardEnough =  pcw.getTagDistance() > VisionConstants.TAG_DRIVEUP_DISTANCE_INCHES;
		if (angle > VisionConstants.ANGLE_TO_TARGET_THRESHOLD_DEGREES) {
			leftMotor1.set(-VisionConstants.CV_TURN_POWER);
			rightMotor1.set(-VisionConstants.CV_TURN_POWER);
			leftMotor2.set(-VisionConstants.CV_TURN_POWER);
			rightMotor2.set(-VisionConstants.CV_TURN_POWER);
		} else if (angle  < -VisionConstants.ANGLE_TO_TARGET_THRESHOLD_DEGREES) {
			leftMotor1.set(VisionConstants.CV_TURN_POWER);
			rightMotor1.set(VisionConstants.CV_TURN_POWER);
			leftMotor2.set(VisionConstants.CV_TURN_POWER);
			rightMotor2.set(VisionConstants.CV_TURN_POWER);
		} else {
			isAligned = true;
			if (isForwardEnough) {
				leftMotor1.set(-VisionConstants.CV_FORWARD_POWER);
				rightMotor1.set(VisionConstants.CV_FORWARD_POWER);
				leftMotor2.set(-VisionConstants.CV_FORWARD_POWER);
				rightMotor2.set(VisionConstants.CV_FORWARD_POWER);
			} else {
				leftMotor1.set(0);
				rightMotor1.set(0);
				leftMotor2.set(0);
				rightMotor2.set(0);
			}
		}
	}
}
