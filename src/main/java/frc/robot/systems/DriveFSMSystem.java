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

public class DriveFSMSystem {


	// FSM state definitions
	public enum FSMState {
		TELE_STATE_2_MOTOR_DRIVE,
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

		currentState = FSMState.TURNING_STATE;

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

		// gyroAngleForOdo = gyro.getAngle();

		currentEncoderPos = ((leftMotor.getEncoder().getPosition()
			- rightMotor.getEncoder().getPosition()) / 2.0);

		// updateLineOdometryTele(gyro.getAngle());

		switch (currentState) {
			case TELE_STATE_2_MOTOR_DRIVE:
				handleTeleOp2MotorState(input);
				break;

			case IDLE:
				handleIdleState(input);
				break;

			case TURNING_STATE:
				handleTurnState(input, 180);
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

			// System.out.println("VELOC: " + gyro.getVelocityX());
			// System.out.print("POPO: " + leftMotor.get());

			currentEncoderPos = ((leftMotor.getEncoder().getPosition()
				- rightMotor.getEncoder().getPosition()) / 2.0);

			// updateLineOdometryTele(gyroAngleForOdo);

			double steerAngle = input.getSteerAngle();
			double currentLeftPower = leftMotor.get();
			double currentRightPower = rightMotor.get();


			DrivePower targetPower = DriveModes.arcadeDrive(input.getLeftJoystickY(),
				steerAngle, currentLeftPower,
				currentRightPower, true);

			// multiple speed modes
			if (input.isLeftJoystickTriggerPressedRaw()) {
				targetPower.scale(Constants.MAX_POWER);
			} else {
				targetPower.scale(Constants.REDUCED_MAX_POWER);
			}

			DrivePower power;

			// acceleration
			power = DriveFunctions.accelerate(targetPower, new DrivePower(currentLeftPower,
				currentRightPower));

			// turning in place
			if (Math.abs(input.getLeftJoystickY()) < Constants.TURNING_IN_PLACE_THRESHOLD) {
				power = DriveFunctions.turnInPlace(input.getRightJoystickY(), steerAngle);
			}

			// System.out.println("ANGLE: " + getAngleToHub());

			leftPower = power.getLeftPower();
			rightPower = power.getRightPower();

			rightMotor.set(rightPower);
			leftMotor.set(leftPower);

		} else {
			leftMotor.set((input.getLeftJoystickY()));
			rightMotor.set(-(input.getRightJoystickY()));
		}

	}

	// ——————————————————————————————————————————————————————————————— //
	// ———————Below commented out due to not having gyro library—————— //
	// ——————————————————————————————————————————————————————————————— //
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

		degrees *= 0.987;

		System.out.println(getHeading());
		double error = degrees - getHeading();
		if (error > 180) {
			error -= 360;
		}
		if (Math.abs(error) <= Constants.TURN_ERROR_THRESHOLD_DEGREE) {
			finishedTurning = true;
			leftMotor.set(0);
			rightMotor.set(0);
			return;
		}
		double power = Math.abs(error) / Constants.TURN_ERROR_POWER_RATIO;
		if (power < Constants.MIN_TURN_POWER) {
			power = Constants.MIN_TURN_POWER;
		}

		power *= (error < 0 && error > -180) ? -1 : 1;

		leftMotor.set(power);
		rightMotor.set(power);
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

	// ——————————————————————————————————————————————————————————————— //
	// ———————Below commented out due to not having gyro library—————— //
	// ——————————————————————————————————————————————————————————————— //
	/**
	* Gets the heading from the gyro.
	* @return the gyro heading
	*/
	public double getHeading() {
		// double angle = startAngle - gyro.getYaw();
		double angle = startAngle - gyro.getAngle();
		if (angle < 0) {
			angle += 360;
		}
		if (angle > 360) {
			angle -= 360;
		}
		return angle;
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

		System.out.println("X Pos: " + roboXPos);
		System.out.println("Y Pos: " + roboYPos);
		// System.out.println("Gyro: " + gyroAngleForOdo);

	}


}
