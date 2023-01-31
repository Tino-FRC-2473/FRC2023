package frc.robot.systems;

// Third party Hardware Imports
import edu.wpi.first.wpilibj.SPI;
import com.revrobotics.CANSparkMax;

import com.kauailabs.navx.frc.AHRS;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
import frc.robot.Constants;

public class FSMSystem {

	// FSM state definitions
	public enum FSMState {
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
	private CANSparkMax leftMotor;
	private CANSparkMax rightMotor;

	private double roboXPos = 0;
	private double roboYPos = 0;
	private double currentEncoderPos;
	private double prevEncoderPos = 0;
	private double gyroAngleForOdo = 0;
	private AHRS gyro;
	private boolean finishedTurning;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public FSMSystem() {
		// Perform hardware init
		rightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_RIGHT1,
										CANSparkMax.MotorType.kBrushless);
		leftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_LEFT1,
										CANSparkMax.MotorType.kBrushless);
		gyro = new AHRS(SPI.Port.kMXP);

		// Reset state machine
		reset();

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
	public void reset() {

		rightMotor.getEncoder().setPosition(0);
		leftMotor.getEncoder().setPosition(0);

		gyro.reset();
		gyro.zeroYaw();
		gyroAngleForOdo = 0;

		currentState = FSMState.P1N1;

		roboXPos = 0;
		roboYPos = 0;
		System.out.println(roboXPos + " " + roboYPos);

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
		currentEncoderPos = ((leftMotor.getEncoder().getPosition()
				- rightMotor.getEncoder().getPosition()) / 2.0);
		updateLineOdometryTele(gyro.getAngle(), currentEncoderPos);

		// System.out.println("currentstate: " + currentState);
		// System.out.println("xPos " + roboXPos);
		// System.out.println("yPos: " + roboYPos);

		switch (currentState) {

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

		// path 1
			case P1N1:
				if (Math.abs(roboX - Constants.P1X1) <= Constants.MOVE_THRESHOLD
					&& Math.abs(roboY) <= Constants.MOVE_THRESHOLD) {
					return FSMState.P1N2;
				} else {
					return FSMState.P1N1;
				}

			case P1N2:
				if (Math.abs(roboX - Constants.P1X2) <= Constants.MOVE_THRESHOLD
					&& Math.abs(roboY) <= Constants.MOVE_THRESHOLD) {
					return FSMState.P1N3;
				} else {
					return FSMState.P1N2;
				}

			case P1N3:
				if (Math.abs(roboX - Constants.P1X3) <= Constants.MOVE_THRESHOLD
					&& Math.abs(roboY) <= Constants.MOVE_THRESHOLD) {
					return null;
				} else {
					return FSMState.P1N3;
				}

			// path 2

			case P2N1:
				if (Math.abs(roboX - Constants.P2X1) <= Constants.MOVE_THRESHOLD
					&& Math.abs(roboY) <= Constants.MOVE_THRESHOLD) {
					return FSMState.P2N2;
				} else {
					return FSMState.P2N1;
				}

			case P2N2:
				if (Math.abs(roboX - Constants.P2X2) <= Constants.MOVE_THRESHOLD
					&& Math.abs(roboY) <= Constants.MOVE_THRESHOLD) {
					return null;
				} else {
					return FSMState.P2N2;
				}

			// path 3

			case P3N1:
				if (Math.abs(roboX - Constants.P3X1) <= Constants.MOVE_THRESHOLD
					&& Math.abs(roboY) <= Constants.MOVE_THRESHOLD) {
					return FSMState.P3N2;
				} else {
					return FSMState.P3N1;
				}

			case P3N2:
				if (Math.abs(roboX - Constants.P3X2) <= Constants.MOVE_THRESHOLD
					&& Math.abs(roboY) <= Constants.MOVE_THRESHOLD) {
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
				if (Math.abs(roboX - Constants.P3X4) <= Constants.MOVE_THRESHOLD
					&& Math.abs(roboY - Constants.P3Y4) <= Constants.MOVE_THRESHOLD) {
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
				if (Math.abs(roboX - Constants.P3X6) <= Constants.MOVE_THRESHOLD
					&& Math.abs(roboY - Constants.P3Y6) <= Constants.MOVE_THRESHOLD) {
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
	 * Tracks the robo's position on the field.
	 * @param gyroAngle robot's angle
	 * @param currentPos robot's current position
	 */
	public void updateLineOdometryTele(double gyroAngle, double currentPos) {

		// double currentEncoderPos = ((-leftEncoderPos + rightEncoderPos) / 2.0);
		double dEncoder = (currentPos - prevEncoderPos)
			/ Constants.REVOLUTIONS_PER_INCH;
		double dX = dEncoder * Math.cos(Math.toRadians(gyroAngleForOdo))
			* Constants.ODOMETRY_DX_CONSTANT;
		double dY = dEncoder * Math.sin(Math.toRadians(gyroAngleForOdo))
			* Constants.ODOMETRY_DY_CONSTANT;

		roboXPos += dX;
		roboYPos += dY;

		prevEncoderPos = currentEncoderPos;
		// return new Translation2d(robotPos.getX() + dX, robotPos.getY() + dY);
	}

	/**
	 * Turns the robot to a fixed angle.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 * @param degrees amount of degrees to turn
	 */
	public void handleTurnState(TeleopInput input, double degrees) {
		if (input != null) {
			return;
		}
		finishedTurning = false;
		System.out.println(getHeading());
		double error = degrees - getHeading();
		if (error > Constants.HALF_CIRCLE) {
			error -= Constants.FULL_CIRCLE;
		}
		if (error < -Constants.HALF_CIRCLE) {
			error += Constants.FULL_CIRCLE;
		}
		System.out.println("ERROR: " + error);
		if (Math.abs(error) <= Constants.TURN_ERROR_THRESHOLD_DEGREE) {
			System.out.println("DONE");
			finishedTurning = true;
			leftMotor.set(0);
			rightMotor.set(0);
			return;
		}
		double power = Math.abs(error) / Constants.TURN_ERROR_POWER_RATIO;
		if (power < Constants.MIN_TURN_POWER) {
			power = Constants.MIN_TURN_POWER;
		}
		power *= (error < 0 && error > -Constants.HALF_CIRCLE) ? -1 : 1;

		leftMotor.set(-power);
		rightMotor.set(-power);
		// turning right is positive and left is negative
	}

	/**
	* Gets the heading from the gyro.
	* @return the gyro heading
	*/
	public double getHeading() {
		double angle = gyro.getAngle() % Constants.FULL_CIRCLE;
		if (angle < 0) {
			angle += Constants.FULL_CIRCLE;
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
			leftMotor.set(-Constants.MOVE_POWER);
			rightMotor.set(Constants.MOVE_POWER);
		} else {
			leftMotor.set(Constants.MOVE_POWER);
			rightMotor.set(-Constants.MOVE_POWER);
		}
		if (Math.abs(roboX - x) <= Constants.MOVE_THRESHOLD
			&& Math.abs(roboY - y) <= Constants.MOVE_THRESHOLD) {
			leftMotor.set(0);
			rightMotor.set(0);
		}
	}
}
