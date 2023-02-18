package frc.robot.systems;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
import frc.robot.drive.DriveModes;
import frc.robot.drive.DrivePower;
import frc.robot.drive.DriveFunctions;
import frc.robot.Constants;
import frc.robot.DrivePoseEstimator;
import frc.robot.PhotonCameraWrapper;

public class DriveFSMSystem {


	// FSM state definitions
	public enum FSMState {
		TELE_STATE_2_MOTOR_DRIVE,
		TELE_STATE_BALANCE,
		TELE_STATE_MECANUM,
		PURE_PURSUIT,
		TURNING_STATE,
		CVLowTapeAlign,
		CVHighTapeAlign,
		CVTagAlign,
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
	// private CANSparkMax leftMotor1;
	// private CANSparkMax rightMotor1;
	// private CANSparkMax leftMotor2;
	// private CANSparkMax rightMotor2;

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

	private boolean forward = false;
	private boolean aligned = false;
	private PhotonCameraWrapper pcw = new PhotonCameraWrapper();

	//private DrivePoseEstimator dpe = new DrivePoseEstimator();


	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public DriveFSMSystem() {
		// Perform hardware init
		// leftMotor1 = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_LEFT1,
		// 								CANSparkMax.MotorType.kBrushless);
		// rightMotor1 = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_RIGHT1,
		// 								CANSparkMax.MotorType.kBrushless);
		// leftMotor2 = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_LEFT2,
		// 								CANSparkMax.MotorType.kBrushless);
		// rightMotor2 = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_RIGHT2,
		// 								CANSparkMax.MotorType.kBrushless);

		// rightMotor1.getEncoder().setPosition(0);
		// leftMotor1.getEncoder().setPosition(0);
		// rightMotor2.getEncoder().setPosition(0);
		// leftMotor2.getEncoder().setPosition(0);

		// leftMotor1.set(0);
		// rightMotor1.set(0);
		// leftMotor2.set(0);
		// rightMotor2.set(0);

		leftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_LEFT,
										CANSparkMax.MotorType.kBrushless);
		rightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_RIGHT,
										CANSparkMax.MotorType.kBrushless);

		rightMotor.getEncoder().setPosition(0);
		leftMotor.getEncoder().setPosition(0);

		leftMotor.set(0);
		rightMotor.set(0);

		finishedTurning = false;

		gyro = new AHRS(SPI.Port.kMXP);

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
		// dpe.updatePose(gyro.getAngle(), leftMotor1.getEncoder().getPosition(),
		// 	rightMotor1.getEncoder().getPosition());
		// SmartDashboard.putNumber("X", dpe.getCurPose().getX());
		// SmartDashboard.putNumber("Y", dpe.getCurPose().getY());
		// SmartDashboard.putNumber("Rotation", dpe.getCurPose().getRotation().getDegrees());
		
		// System.out.println("X: " + roboXPos);
		// System.out.println("Y: " + roboYPos);

		gyroAngleForOdo = gyro.getAngle();

		currentEncoderPos = ((leftMotor.getEncoder().getPosition()
			- rightMotor.getEncoder().getPosition()) / 2.0);

		updateLineOdometryTele(gyro.getAngle());

		switch (currentState) {
			case TELE_STATE_2_MOTOR_DRIVE:
				handleTeleOp2MotorState(input);
				break;

			case CVLowTapeAlign:
				System.out.println("low");
				handleCVTapeAlignState(true);
				break;

			case CVHighTapeAlign:
				System.out.println("high");
				handleCVTapeAlignState(false);
				break;

			case CVTagAlign:
				System.out.println("tag");
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
				moveState(input, false, Constants.P3X1, 0);
				break;

			case P3N2:
				moveState(input, true, Constants.P3X2, 0);
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
				if (input != null && input.isDriveJoystickCVLowTapeButtonPressedRaw()) {
					System.out.println("low");
					aligned = false;
					forward = true;
					return FSMState.CVLowTapeAlign;
				}else if (input != null && input.isDriveJoystickCVHighTapeButtonPressedRaw()) {
					System.out.println("high");
					aligned = false;
					forward = true;
					return FSMState.CVHighTapeAlign;
				}else if (input != null && input.isDriveJoystickCVTagButtonPressedRaw()) {
					System.out.println("tag");
					aligned = false;
					forward = true;
					return FSMState.CVTagAlign;
				}
				return FSMState.TELE_STATE_2_MOTOR_DRIVE;

			case TELE_STATE_MECANUM:
				return FSMState.TELE_STATE_MECANUM;

			case TURNING_STATE:
				System.out.println(finishedTurning);
				if (finishedTurning) {
					return FSMState.IDLE;
				} else {
					return FSMState.TURNING_STATE;
				}

			case CVLowTapeAlign:
				if (!forward && aligned) {
					return FSMState.TELE_STATE_2_MOTOR_DRIVE;
				}
				return FSMState.CVLowTapeAlign;

			case CVHighTapeAlign:
				if (!forward && aligned) {
					return FSMState.TELE_STATE_2_MOTOR_DRIVE;
				}
				return FSMState.CVHighTapeAlign;

			case CVTagAlign:
				if (!forward && aligned) {
					return FSMState.TELE_STATE_2_MOTOR_DRIVE;
				}
				return FSMState.CVTagAlign;

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

			// path 4

			case P4N1:
				if (finishedTurning) {
					return FSMState.P4N2;
				} else {
					return FSMState.P4N1;
				}

			case P4N2:
				if (Math.abs(roboX - Constants.P4X2) <= Constants.AUTONOMUS_MOVE_THRESHOLD
					&& Math.abs(roboY - Constants.P4Y2) <= Constants.AUTONOMUS_MOVE_THRESHOLD) {
					return FSMState.P4N3;
				} else {
					return FSMState.P4N2;
				}

			case P4N3:
				if (finishedTurning) {
					return FSMState.P4N4;
				} else {
					return FSMState.P4N3;
				}

			case P4N4:
				if (Math.abs(roboX - Constants.P4X4) <= Constants.AUTONOMUS_MOVE_THRESHOLD
					&& Math.abs(roboY - Constants.P4Y4) <= Constants.AUTONOMUS_MOVE_THRESHOLD) {
					return FSMState.P4N5;
				} else {
					return FSMState.P4N4;
				}

			case P4N5:
				if (Math.abs(roboX - Constants.P4X5) <= Constants.AUTONOMUS_MOVE_THRESHOLD
					&& Math.abs(roboY - Constants.P4Y5) <= Constants.AUTONOMUS_MOVE_THRESHOLD) {
					return FSMState.P4N6;
				} else {
					return FSMState.P4N5;
				}

			case P4N6:
				if (finishedTurning) {
					return FSMState.P4N7;
				} else {
					return FSMState.P3N6;
				}

			case P4N7:
				if (Math.abs(roboX) <= Constants.AUTONOMUS_MOVE_THRESHOLD
					&& Math.abs(roboY - Constants.P4Y7) <= Constants.AUTONOMUS_MOVE_THRESHOLD) {
					return FSMState.P4N8;
				} else {
					return FSMState.P4N7;
				}

			case P4N8:
				if (finishedTurning) {
					return null;
				} else {
					return FSMState.P4N8;
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

			currentEncoderPos = ((leftMotor.getEncoder().getPosition()
				- rightMotor.getEncoder().getPosition()) / 2.0);

			// updateLineOdometryTele(gyroAngleForOdo);

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
			// System.out.println("X: " + roboXPos);
			// System.out.println("Y: " + roboYPos);

			leftMotor.set(leftPower);
			rightMotor.set(rightPower);

			// leftMotor1.set(leftPower);
			// rightMotor1.set(rightPower);
			// leftMotor2.set(leftPower);
			// rightMotor2.set(rightPower);

		} else {
			leftMotor.set((input.getdriveJoystickY()));
			rightMotor.set(-(input.getmechJoystickY()));

			// leftMotor1.set((input.getdriveJoystickY()));
			// rightMotor1.set(-(input.getmechJoystickY()));
			// leftMotor2.set((input.getdriveJoystickY()));
			// rightMotor2.set(-(input.getmechJoystickY()));
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
		System.out.println(getHeading());
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
			// leftMotor1.set(0);
			// rightMotor1.set(0);
			// leftMotor2.set(0);
			// rightMotor2.set(0);
			return;
		}
		double power = Math.abs(error) / Constants.TURN_ERROR_POWER_RATIO;
		if (power < Constants.MIN_TURN_POWER) {
			power = Constants.MIN_TURN_POWER;
		}
		power *= ((error < 0 && error > -Constants.HALF_REVOLUTION_DEGREES) ? -1 : 1);
		power = -power / 4;

		// leftMotor1.set(-power);
		// rightMotor1.set(-power);
		// leftMotor2.set(-power);
		// rightMotor2.set(-power);
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

		//System.out.println("X Pos: " + roboXPos);
		//System.out.println("Y Pos: " + roboYPos);
		//System.out.println("Gyro: " + gyroAngleForOdo);

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
		System.out.println("x: " + roboX);
		System.out.println("y: " + roboY);

		// if (forwards) {
		// 	leftMotor1.set(-Constants.AUTONOMUS_MOVE_POWER);
		// 	rightMotor2.set(Constants.AUTONOMUS_MOVE_POWER);
		// 	leftMotor2.set(-Constants.AUTONOMUS_MOVE_POWER);
		// 	rightMotor1.set(Constants.AUTONOMUS_MOVE_POWER);
		// } else {
		// 	leftMotor1.set(Constants.AUTONOMUS_MOVE_POWER);
		// 	rightMotor2.set(-Constants.AUTONOMUS_MOVE_POWER);
		// 	leftMotor2.set(Constants.AUTONOMUS_MOVE_POWER);
		// 	rightMotor1.set(-Constants.AUTONOMUS_MOVE_POWER);
		// }
		// if (Math.abs(roboX - x) <= Constants.AUTONOMUS_MOVE_THRESHOLD
		// 	&& Math.abs(roboY - y) <= Constants.AUTONOMUS_MOVE_THRESHOLD) {
		// 	leftMotor1.set(0);
		// 	rightMotor2.set(0);
		// 	leftMotor2.set(0);
		// 	rightMotor1.set(0);
		// }
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
			forward =  pcw.getLowerTapeDistance() > 42;
			System.out.println("distance: " + pcw.getLowerTapeDistance());
			//drives forward until within 42 inches of lower tape
		} else {
			angle = pcw.getHigherTapeTurnAngle();
			forward = pcw.getHigherTapeDistance() > 65;
			System.out.println("distance: " + pcw.getHigherTapeDistance());
			//drives forward until within 65 inches of higher tape
		}
		System.out.println("angle: " + angle);
		
		if (angle > 4) {
			leftMotor.set(-0.05);
			rightMotor.set(-0.05);
		} else if (angle  < -4) {
			leftMotor.set(0.05);
			rightMotor.set(0.05);
		} else {
			aligned = true;
			if (forward) {
				leftMotor.set(-0.1);
				rightMotor.set(0.1);
			} else {
				leftMotor.set(0);
				rightMotor.set(0);
			}
		}

	}

	/**.
 	* Aligns to april tag and drives up to within 35 inches of it
	*/
	public void handleCVTagAlignState() {
		System.out.println("TAG");
		double angle = pcw.getTagTurnAngle();
		forward =  pcw.getTagDistance() > 35;
		if (angle > 4) {
			leftMotor.set(-0.05);
			rightMotor.set(-0.05);
		} else if (angle  < -4) {
			leftMotor.set(0.05);
			rightMotor.set(0.05);
		} else {
			aligned = true;
			if (forward) {
				leftMotor.set(-0.1);
				rightMotor.set(0.1);
			}else {
				leftMotor.set(0);
				rightMotor.set(0);
			}
		}
	}
}
