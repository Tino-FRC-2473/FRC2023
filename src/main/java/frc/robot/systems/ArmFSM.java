package frc.robot.systems;



// WPILib Imports
import edu.wpi.first.math.controller.PIDController;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class ArmFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		IDLE,
		ARM_MOVEMENT,
		SHOOT_HIGH,
		SHOOT_MID
	}

	private static final float TELEARM_MOTOR_POWER = 0.1f;
	private static final float PIVOT_MOTOR_POWER = 0.1f;
	private static final double ARM_ENCODER_HIGH = 20;
	private static final double ARM_ENCODER_MID = 10;
	private static final double SHOOT_ANGLE_ENCODER = 10;
	private static final double BALANCE_ANGLE_ENCODER = 5;
	private static final double GRABBER_ANGLE_ENCODER = -5;
	private static final double ARM_ENCODER_GRAB = 10;
	private static final double PID_MAX_POWER = 0.3;
	private static final double PIVOT_ERROR_ARM = 0.3;
	private static final double PID_CONSTANT_P = 0.00022f;
	private static final double PID_CONSTANT_I = 0.000055f;
	private static final double PID_CONSTANT_D = 0.000008f;


	/* ======================== Private variables ======================== */
	private FSMState currentState;

	private CANSparkMax pivotMotor;
	private CANSparkMax teleArmMotor;
	private SparkMaxLimitSwitch pivotLimitSwitchHigh;
	private SparkMaxLimitSwitch pivotLimitSwitchLow;
	private PIDController pidController;
	/*
	 * Hardware Map each of the motors
	 *
	 */
	/**
	 * Creates an instance of an ArmFSM.
	 */
	public ArmFSM() {
		// Perform hardware init
		pivotMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_PIVOT,
										CANSparkMax.MotorType.kBrushless);
		pivotLimitSwitchHigh = pivotMotor.getForwardLimitSwitch(
								SparkMaxLimitSwitch.Type.kNormallyOpen);
		pivotLimitSwitchHigh.enableLimitSwitch(true);
		pivotLimitSwitchLow = pivotMotor.getReverseLimitSwitch(
								SparkMaxLimitSwitch.Type.kNormallyOpen);
		pivotLimitSwitchLow.enableLimitSwitch(true);
		teleArmMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_TELEARM,
										CANSparkMax.MotorType.kBrushless);
		pidController = new PIDController(PID_CONSTANT_P, PID_CONSTANT_I, PID_CONSTANT_D);
		// Reset state machine
		reset();
	}

	/**
	 * Get the current state.
	 *
	 * @return current state
	 */
	public FSMState getCurrentState() {
		return currentState;
	}

	/**
	 * On robot start set the start to IDLE state. Resets robot to original state.
	 */
	public void reset() {
		currentState = FSMState.IDLE;
		pivotMotor.getEncoder().setPosition(0);
		teleArmMotor.getEncoder().setPosition(0);

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Updates the current FSMState.
	 * @param input TeleopInput
	 */
	public void update(TeleopInput input) {
		if (input == null) {
			handleIdleState(input);
			return;
		}
		switch (currentState) {
			case IDLE:
				handleIdleState(input);
				break;
			case ARM_MOVEMENT:
				handleArmMechState(input);
				break;
			case SHOOT_HIGH:
				handleShootHighState(input);
				break;
			case SHOOT_MID:
				handleShootMidState(input);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		FSMState state = nextState(input);
		if (currentState != state) {
			System.out.println(state);
		}
		currentState = state;
	}

	/*
	 * When inputs are pressed, the states will change likewise
	 *
	 * @return expected state
	 */
	private FSMState nextState(TeleopInput input) {
		if (input == null) {
			return FSMState.IDLE;
		}
		switch (currentState) {
			case IDLE:
				if ((input.isExtendButtonPressed() || input.isRetractButtonPressed()
					|| input.isPivotIncreaseButtonPressed() || input.isPivotDecreaseButtonPressed())
					&& !input.isShootHighButtonPressed() && !input.isShootMidButtonPressed()) {
					return FSMState.ARM_MOVEMENT;
				} else if (input.isShootHighButtonPressed()) {
					return FSMState.SHOOT_HIGH;
				} else if (input.isShootMidButtonPressed()) {
					return FSMState.SHOOT_MID;
				}
				return FSMState.IDLE;
			case ARM_MOVEMENT:
				if (input.isShootHighButtonPressed()) {
					return FSMState.SHOOT_HIGH;
				} else if (input.isShootMidButtonPressed()) {
					return FSMState.SHOOT_MID;
				} else if (input.isExtendButtonPressed() || input.isRetractButtonPressed()
					|| input.isPivotIncreaseButtonPressed()
					|| input.isPivotDecreaseButtonPressed()) {
					return FSMState.ARM_MOVEMENT;
				}
				return FSMState.IDLE;
			case SHOOT_HIGH:
				if (input.isShootHighButtonPressed()) {
					return FSMState.SHOOT_HIGH;
				}
				return FSMState.IDLE;
			case SHOOT_MID:
				if (input.isShootMidButtonPressed()) {
					return FSMState.SHOOT_MID;
				}
				return FSMState.IDLE;
			default :
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/**
	 * Returns whether the difference between two rotations is within the error.
	 * @param a rotation value 1
	 * @param b rotation value 2
	 * @return whether they are within the error
	 */
	private boolean withinError(double a, double b) {
		return Math.abs(a - b) < PIVOT_ERROR_ARM;
	}

	private boolean isMaxHeight() {
		return pivotLimitSwitchHigh.isPressed();
	}

	private boolean isMinHeight() {
		return pivotLimitSwitchLow.isPressed();
	}
	/*
	 * What to do when in the IDLE state
	 */
	private void handleIdleState(TeleopInput input) {
		teleArmMotor.set(0);
		pivotMotor.set(0);
	}


	/*
	 * What to do when in the ARM_MOVEMENT state
	 */
	private void handleArmMechState(TeleopInput input) {
		if (input.isPivotIncreaseButtonPressed() && !isMaxHeight()) {
			//pivotMotor.set(PIVOT_MOTOR_POWER);
			double pow = pidController.calculate(pivotMotor.getEncoder().getVelocity(),
						PIVOT_MOTOR_POWER);
			if (pow > PID_MAX_POWER || pow < -PID_MAX_POWER) {
				return;
			}
			pivotMotor.set(pow);
		} else if (input.isPivotDecreaseButtonPressed() && !isMinHeight()) {
			//pivotMotor.set(-PIVOT_MOTOR_POWER);
			double pow = pidController.calculate(pivotMotor.getEncoder().getVelocity(),
						-PIVOT_MOTOR_POWER);
			if (pow > PID_MAX_POWER || pow < -PID_MAX_POWER) {
				return;
			}
			pivotMotor.set(pow);
		} else {
			pivotMotor.set(0);
		}

		if (input.isExtendButtonPressed()) {
			teleArmMotor.set(TELEARM_MOTOR_POWER);
		} else if (input.isRetractButtonPressed()) {
			teleArmMotor.set(-TELEARM_MOTOR_POWER);
		} else {
			teleArmMotor.set(0);
		}
	}

	private void handleShootHighState(TeleopInput input) {
		if (withinError(pivotMotor.getEncoder().getPosition(), SHOOT_ANGLE_ENCODER)) {
			pivotMotor.set(0);
		} else if (pivotMotor.getEncoder().getPosition() < SHOOT_ANGLE_ENCODER) {
			//pivotMotor.set(PIVOT_MOTOR_POWER);
			double pow = pidController.calculate(pivotMotor.getEncoder().getVelocity(),
						PIVOT_MOTOR_POWER);
			if (pow > PID_MAX_POWER || pow < -PID_MAX_POWER) {
				return;
			}
			pivotMotor.set(pow);
		} else if (pivotMotor.getEncoder().getPosition() > SHOOT_ANGLE_ENCODER) {
			//pivotMotor.set(-PIVOT_MOTOR_POWER);
			double pow = pidController.calculate(pivotMotor.getEncoder().getVelocity(),
						-PIVOT_MOTOR_POWER);
			if (pow > PID_MAX_POWER || pow < -PID_MAX_POWER) {
				return;
			}
			pivotMotor.set(pow);
		} else {
			pivotMotor.set(0);
		}
		if (teleArmMotor.getEncoder().getPosition() < ARM_ENCODER_HIGH) {
			teleArmMotor.set(TELEARM_MOTOR_POWER);
		} else {
			teleArmMotor.set(0);
		}
	}

	private void handleShootMidState(TeleopInput input) {
		if (withinError(pivotMotor.getEncoder().getPosition(), SHOOT_ANGLE_ENCODER)) {
			pivotMotor.set(0);
		} else if (pivotMotor.getEncoder().getPosition() < SHOOT_ANGLE_ENCODER) {
			//pivotMotor.set(PIVOT_MOTOR_POWER);
			double pow = pidController.calculate(pivotMotor.getEncoder().getVelocity(),
						PIVOT_MOTOR_POWER);
			if (pow > PID_MAX_POWER || pow < -PID_MAX_POWER) {
				return;
			}
			pivotMotor.set(pow);
		} else if (pivotMotor.getEncoder().getPosition() > SHOOT_ANGLE_ENCODER) {
			//pivotMotor.set(-PIVOT_MOTOR_POWER);
			double pow = pidController.calculate(pivotMotor.getEncoder().getVelocity(),
						-PIVOT_MOTOR_POWER);
			if (pow > PID_MAX_POWER || pow < -PID_MAX_POWER) {
				return;
			}
			pivotMotor.set(pow);
		} else {
			pivotMotor.set(0);
		}
		if (teleArmMotor.getEncoder().getPosition() < ARM_ENCODER_MID) {
			teleArmMotor.set(TELEARM_MOTOR_POWER);
		} else {
			teleArmMotor.set(0);
		}
	}

	/**
	 * Method to adjust the arm to go shoot on high height to use in autonomous.
	 */
	public void executeShootHigh() {
		while (!withinError(pivotMotor.getEncoder().getPosition(), SHOOT_ANGLE_ENCODER)
			&& teleArmMotor.getEncoder().getPosition() < ARM_ENCODER_HIGH) {
			if (withinError(pivotMotor.getEncoder().getPosition(), SHOOT_ANGLE_ENCODER)) {
				pivotMotor.set(0);
			} else if (pivotMotor.getEncoder().getPosition() < SHOOT_ANGLE_ENCODER) {
				pivotMotor.set(PIVOT_MOTOR_POWER);
			} else if (pivotMotor.getEncoder().getPosition() > SHOOT_ANGLE_ENCODER) {
				pivotMotor.set(-PIVOT_MOTOR_POWER);
			} else {
				pivotMotor.set(0);
			}
			if (teleArmMotor.getEncoder().getPosition() < ARM_ENCODER_HIGH) {
				teleArmMotor.set(TELEARM_MOTOR_POWER);
			} else {
				teleArmMotor.set(0);
			}
		}
		pivotMotor.set(0);
		teleArmMotor.set(0);
	}

	/**
	 * Method to adjust the arm to go shoot on mid height to use in autonomous.
	 */
	public void executeShootMid() {
		while (!withinError(pivotMotor.getEncoder().getPosition(), SHOOT_ANGLE_ENCODER)
			&& teleArmMotor.getEncoder().getPosition() < ARM_ENCODER_MID) {
			if (withinError(pivotMotor.getEncoder().getPosition(), SHOOT_ANGLE_ENCODER)) {
				pivotMotor.set(0);
			} else if (pivotMotor.getEncoder().getPosition() < SHOOT_ANGLE_ENCODER) {
				pivotMotor.set(PIVOT_MOTOR_POWER);
			} else if (pivotMotor.getEncoder().getPosition() > SHOOT_ANGLE_ENCODER) {
				pivotMotor.set(-PIVOT_MOTOR_POWER);
			} else {
				pivotMotor.set(0);
			}
			if (teleArmMotor.getEncoder().getPosition() < ARM_ENCODER_MID) {
				teleArmMotor.set(TELEARM_MOTOR_POWER);
			} else {
				teleArmMotor.set(0);
			}
		}
		pivotMotor.set(0);
		teleArmMotor.set(0);
	}

	/**
	 * Method to move the arm to the best positioning for balancing.
	 */
	public void executeBalanceArm() {
		while (!withinError(pivotMotor.getEncoder().getPosition(), BALANCE_ANGLE_ENCODER)
			&& teleArmMotor.getEncoder().getPosition() > 0) {
			if (withinError(pivotMotor.getEncoder().getPosition(), BALANCE_ANGLE_ENCODER)) {
				pivotMotor.set(0);
			} else if (pivotMotor.getEncoder().getPosition() < BALANCE_ANGLE_ENCODER) {
				pivotMotor.set(PIVOT_MOTOR_POWER);
			} else if (pivotMotor.getEncoder().getPosition() > BALANCE_ANGLE_ENCODER) {
				pivotMotor.set(-PIVOT_MOTOR_POWER);
			} else {
				pivotMotor.set(0);
			}
			if (teleArmMotor.getEncoder().getPosition() > 0) {
				teleArmMotor.set(-TELEARM_MOTOR_POWER);
			} else {
				teleArmMotor.set(0);
			}
		}
		pivotMotor.set(0);
		teleArmMotor.set(0);
	}

	/**
	 * Method to move arm for grabbing in autonomous.
	 */
	public void executeGrabberArm() {
		while (!withinError(pivotMotor.getEncoder().getPosition(), GRABBER_ANGLE_ENCODER)
			&& teleArmMotor.getEncoder().getPosition() < ARM_ENCODER_GRAB) {
			if (withinError(pivotMotor.getEncoder().getPosition(), GRABBER_ANGLE_ENCODER)) {
				pivotMotor.set(0);
			} else if (pivotMotor.getEncoder().getPosition() < GRABBER_ANGLE_ENCODER) {
				pivotMotor.set(PIVOT_MOTOR_POWER);
			} else if (pivotMotor.getEncoder().getPosition() > GRABBER_ANGLE_ENCODER) {
				pivotMotor.set(-PIVOT_MOTOR_POWER);
			} else {
				pivotMotor.set(0);
			}
			if (teleArmMotor.getEncoder().getPosition() < ARM_ENCODER_GRAB) {
				teleArmMotor.set(TELEARM_MOTOR_POWER);
			} else {
				teleArmMotor.set(0);
			}
		}
		pivotMotor.set(0);
		teleArmMotor.set(0);
	}

	/**
	 * Method to retract arm to minimum.
	 */
	public void executeRetractToMin() {
		while (teleArmMotor.getEncoder().getPosition() > 0) {
			teleArmMotor.set(-TELEARM_MOTOR_POWER);
		}
		teleArmMotor.set(0);
	}
}
