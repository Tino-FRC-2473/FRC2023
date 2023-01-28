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
		path1,
		path2,
		path3,
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



	// turn state
	private boolean finishedTurning = false;


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

		currentState = FSMState.path3;

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
			case path1:
				path1(input);
				break;

			case path2:
				path2(input);
				break;

			case path3:
				path3(input);
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

			case path1:
				return FSMState.path1;

			case path2:
				return FSMState.path2;

			case path3:
				return FSMState.path3;

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
	 * Tracks the robo's position on the field.
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

	private int state = 0;
	/**
	 * Autonomus path 1.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void path1(TeleopInput input) {
		if (input != null) {
			return;
		}
		double roboX = -roboXPos;
		double roboY = roboYPos;
		System.out.println("x: " + roboX);
		System.out.println("y: " + roboY);
		System.out.println("state: " + state);
		if (state == 0) {
			leftMotor.set(-Constants.MOVE_POWER);
			rightMotor.set(Constants.MOVE_POWER);
			if (Math.abs(roboX - Constants.P1X1) <= Constants.MOVE_THRESHOLD) {
				leftMotor.set(0);
				rightMotor.set(0);
				state++;
			}
		} else if (state == 1) {
			System.out.println("entered 1");
			leftMotor.set(Constants.MOVE_POWER);
			rightMotor.set(-Constants.MOVE_POWER);
			if (Math.abs(roboX - Constants.P1X2) <= Constants.MOVE_THRESHOLD) {
				leftMotor.set(0);
				rightMotor.set(0);
				state++;
			}
		} else if (state == 2) {
			leftMotor.set(-Constants.MOVE_POWER);
			rightMotor.set(Constants.MOVE_POWER);
			if (Math.abs(roboX - Constants.P1X3) <= Constants.MOVE_THRESHOLD) {
				leftMotor.set(0);
				rightMotor.set(0);
				System.out.println("end");
				// state++;
			}
		}
	}

	/**
	 * Autonomus path 2.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void path2(TeleopInput input) {
		if (input != null) {
			return;
		}
		double roboX = -roboXPos;
		double roboY = roboYPos;
		System.out.println("x: " + roboX);
		System.out.println("y: " + roboY);
		System.out.println("state: " + state);
		if (state == 0) {
			leftMotor.set(-Constants.MOVE_POWER);
			rightMotor.set(Constants.MOVE_POWER);
			if (Math.abs(roboX - Constants.P2X1) <= Constants.MOVE_THRESHOLD) {
				leftMotor.set(0);
				rightMotor.set(0);
				state++;
			}
		} else if (state == 1) {
			System.out.println("entered 1");
			leftMotor.set(Constants.MOVE_POWER);
			rightMotor.set(-Constants.MOVE_POWER);
			if (Math.abs(roboX + Constants.P2X2) <= Constants.MOVE_THRESHOLD) {
				leftMotor.set(0);
				rightMotor.set(0);
				System.out.println("end");
				// state++;
			}
		}
	}

	/**
	 * Autonomus path 3.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void path3(TeleopInput input) {
		if (input != null) {
			return;
		}
		double roboX = -roboXPos;
		double roboY = roboYPos;
		System.out.println("x: " + roboX);
		System.out.println("y: " + roboY);
		System.out.println("state: " + state);
		System.out.println("angle: " + gyro.getAngle());

		// if (state == 0) { // turning right
		// 	handleTurnState(input, Constants.P3TURN_AMT1 * 0.913);
		// 	if (finishedTurning) {
		// 		state--;
		// 	}
		// }

		if (state == 0) {
			leftMotor.set(Constants.MOVE_POWER);
			rightMotor.set(-Constants.MOVE_POWER);
			if (Math.abs(roboX - Constants.P3X1) <= Constants.MOVE_THRESHOLD) {
				leftMotor.set(0);
				rightMotor.set(0);
				state++;
			}
		} else if (state == 1) {
			leftMotor.set(-Constants.MOVE_POWER);
			rightMotor.set(Constants.MOVE_POWER);
			if (Math.abs(roboX) <= Constants.MOVE_THRESHOLD) {
				leftMotor.set(0);
				rightMotor.set(0);
				state++;
			}
		} else if (state == 2) {
			leftMotor.set(Constants.MOVE_POWER);
			rightMotor.set(-Constants.MOVE_POWER);
			if (Math.abs(roboX - Constants.P3X2) <= Constants.MOVE_THRESHOLD) {
				leftMotor.set(0);
				rightMotor.set(0);
				state++;
			}
		} else if (state == Constants.S3) {
			handleTurnState(input, Constants.P3TURN_AMT1);
			if (finishedTurning) {
				state++;
			}
		} else if (state == Constants.S4) {
			leftMotor.set(Constants.MOVE_POWER);
			rightMotor.set(-Constants.MOVE_POWER);
			if (Math.abs(roboX - Constants.P3X3) <= Constants.MOVE_THRESHOLD
				&& Math.abs(roboY - Constants.P3Y3) <= Constants.MOVE_THRESHOLD) {
				leftMotor.set(0);
				rightMotor.set(0);
				state++;
			}
		} else if (state == Constants.S5) { // turning left
			handleTurnState(input, Constants.P3TURN_AMT2);
			if (finishedTurning) {
				state++;
			}
		} else if (state == Constants.S6) {
			leftMotor.set(Constants.MOVE_POWER);
			rightMotor.set(-Constants.MOVE_POWER);
			if (Math.abs(roboX - Constants.P3X4) <=  Constants.MOVE_THRESHOLD
				&& Math.abs(roboY - Constants.P3Y4) <= Constants.MOVE_THRESHOLD) {
				leftMotor.set(0);
				rightMotor.set(0);
				// state++;
			}
		}
	}
}
