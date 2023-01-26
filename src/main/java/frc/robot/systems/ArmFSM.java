package frc.robot.systems;



// WPILib Imports
//import edu.wpi.first.math.controller.PIDController;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

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
		SHOOT_MID,
		SHOOT_LOW_FORWARD
	}

	private static final float TELEARM_MOTOR_POWER = 0.1f;
	private static final float PIVOT_MOTOR_POWER = 0.1f;
	private static final double ARM_ENCODER_HIGH_FORWARD = 20;
	private static final double ARM_ENCODER_HIGH_BACKWARD = 40;
	private static final double ARM_ENCODER_MID_FORWARD = 10;
	private static final double ARM_ENCODER_MID_BACKWARD = 10;
	private static final double ARM_ENCODER_LOW = 10;
	private static final double SHOOT_ANGLE_ENCODER_FORWARD = 10;
	private static final double SHOOT_ANGLE_ENCODER_BACKWARD = 30;
	private static final double SHOOT_LOW_ANGLE_ENCODER = 5;
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
	//private PIDController pidController;
	private SparkMaxPIDController pidController;
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
								SparkMaxLimitSwitch.Type.kNormallyClosed);
		pivotLimitSwitchHigh.enableLimitSwitch(true);
		pivotLimitSwitchLow = pivotMotor.getReverseLimitSwitch(
								SparkMaxLimitSwitch.Type.kNormallyClosed);
		pivotLimitSwitchLow.enableLimitSwitch(true);
		teleArmMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_TELEARM,
										CANSparkMax.MotorType.kBrushless);
		pidController = pivotMotor.getPIDController();
		pidController.setP(PID_CONSTANT_P);
		pidController.setI(PID_CONSTANT_I);
		pidController.setD(PID_CONSTANT_D);
		pidController.setIZone(0);
		pidController.setFF(0);
		pidController.setOutputRange(-PID_MAX_POWER, PID_MAX_POWER);
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
			case SHOOT_LOW_FORWARD:
				handleShootLowState(input);
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
			case SHOOT_LOW_FORWARD:
				if (input.isShootLowButtonPressed()) {
					return FSMState.SHOOT_LOW_FORWARD;
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
		if (input != null) {
			if (input.isPivotIncreaseButtonPressed() && !isMaxHeight()) {
				//pivotMotor.set(PIVOT_MOTOR_POWER);
				pidController.setReference(PIVOT_MOTOR_POWER, CANSparkMax.ControlType.kDutyCycle);
			} else if (input.isPivotDecreaseButtonPressed() && !isMinHeight()) {
				//pivotMotor.set(-PIVOT_MOTOR_POWER);
				pidController.setReference(-PIVOT_MOTOR_POWER, CANSparkMax.ControlType.kDutyCycle);
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
		} else {
			teleArmMotor.set(0);
			pivotMotor.set(0);
		}
	}

	private void handleShootHighState(TeleopInput input) {
		if (input != null) {
			if (input.isThrottleForward()) {
				if (withinError(pivotMotor.getEncoder().getPosition(),
					SHOOT_ANGLE_ENCODER_FORWARD)) {
					pivotMotor.set(0);
				} else if (pivotMotor.getEncoder().getPosition() < SHOOT_ANGLE_ENCODER_FORWARD) {
					//pivotMotor.set(PIVOT_MOTOR_POWER);
					double pow = pidController.calculate(pivotMotor.getEncoder().getVelocity(),
								PIVOT_MOTOR_POWER);
					if (pow > PID_MAX_POWER || pow < -PID_MAX_POWER) {
						return;
					}
					pivotMotor.set(pow);
				} else if (pivotMotor.getEncoder().getPosition() > SHOOT_ANGLE_ENCODER_FORWARD) {
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
				if (teleArmMotor.getEncoder().getPosition() < ARM_ENCODER_HIGH_FORWARD) {
					teleArmMotor.set(TELEARM_MOTOR_POWER);
				} else {
					teleArmMotor.set(0);
				}
			} else { //throttle is backward/false
				if (withinError(pivotMotor.getEncoder().getPosition(),
					SHOOT_ANGLE_ENCODER_BACKWARD)) {
					pivotMotor.set(0);
				} else if (pivotMotor.getEncoder().getPosition() < SHOOT_ANGLE_ENCODER_BACKWARD) {
					//pivotMotor.set(PIVOT_MOTOR_POWER);
					double pow = pidController.calculate(pivotMotor.getEncoder().getVelocity(),
								PIVOT_MOTOR_POWER);
					if (pow > PID_MAX_POWER || pow < -PID_MAX_POWER) {
						return;
					}
					pivotMotor.set(pow);
				} else if (pivotMotor.getEncoder().getPosition() > SHOOT_ANGLE_ENCODER_BACKWARD) {
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
				if (teleArmMotor.getEncoder().getPosition() < ARM_ENCODER_HIGH_BACKWARD) {
					teleArmMotor.set(TELEARM_MOTOR_POWER);
				} else {
					teleArmMotor.set(0);
				}
			}
		} else {
			teleArmMotor.set(0);
			pivotMotor.set(0);
		}
	}

	private void handleShootMidState(TeleopInput input) {
		if (input != null) {
			if (input.isThrottleForward()) {
				if (withinError(pivotMotor.getEncoder().getPosition(),
					SHOOT_ANGLE_ENCODER_FORWARD)) {
					pivotMotor.set(0);
				} else if (pivotMotor.getEncoder().getPosition() < SHOOT_ANGLE_ENCODER_FORWARD) {
					//pivotMotor.set(PIVOT_MOTOR_POWER);
					double pow = pidController.calculate(pivotMotor.getEncoder().getVelocity(),
								PIVOT_MOTOR_POWER);
					if (pow > PID_MAX_POWER || pow < -PID_MAX_POWER) {
						return;
					}
					pivotMotor.set(pow);
				} else if (pivotMotor.getEncoder().getPosition() > SHOOT_ANGLE_ENCODER_FORWARD) {
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
				if (teleArmMotor.getEncoder().getPosition() < ARM_ENCODER_MID_FORWARD) {
					teleArmMotor.set(TELEARM_MOTOR_POWER);
				} else {
					teleArmMotor.set(0);
				}
			} else { //throttle is backward/false
				if (withinError(pivotMotor.getEncoder().getPosition(),
					SHOOT_ANGLE_ENCODER_BACKWARD)) {
					pivotMotor.set(0);
				} else if (pivotMotor.getEncoder().getPosition()
					< SHOOT_ANGLE_ENCODER_BACKWARD) {
					//pivotMotor.set(PIVOT_MOTOR_POWER);
					double pow = pidController.calculate(pivotMotor.getEncoder().getVelocity(),
								PIVOT_MOTOR_POWER);
					if (pow > PID_MAX_POWER || pow < -PID_MAX_POWER) {
						return;
					}
					pivotMotor.set(pow);
				} else if (pivotMotor.getEncoder().getPosition()
						> SHOOT_ANGLE_ENCODER_BACKWARD) {
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
				if (teleArmMotor.getEncoder().getPosition() < ARM_ENCODER_MID_BACKWARD) {
					teleArmMotor.set(TELEARM_MOTOR_POWER);
				} else {
					teleArmMotor.set(0);
				}
			}
		} else {
			teleArmMotor.set(0);
			pivotMotor.set(0);
		}
	}

	private void handleShootLowState(TeleopInput input) {
		if (input != null) {
			if (withinError(pivotMotor.getEncoder().getPosition(),
				SHOOT_LOW_ANGLE_ENCODER_ROTATIONS)) {
				pivotMotor.set(0);
			} else if (pivotMotor.getEncoder().getPosition() < SHOOT_LOW_ANGLE_ENCODER_ROTATIONS) {
				//pivotMotor.set(PIVOT_MOTOR_POWER);
				pidController.setReference(PIVOT_MOTOR_POWER, CANSparkMax.ControlType.kDutyCycle);
			} else if (pivotMotor.getEncoder().getPosition() > SHOOT_LOW_ANGLE_ENCODER_ROTATIONS) {
				//pivotMotor.set(-PIVOT_MOTOR_POWER);
				pidController.setReference(-PIVOT_MOTOR_POWER, CANSparkMax.ControlType.kDutyCycle);
			} else {
				pivotMotor.set(0);
			}
			if (teleArmMotor.getEncoder().getPosition() < ARM_ENCODER_LOW_ROTATIONS) {
				teleArmMotor.set(TELEARM_MOTOR_POWER);
			} else {
				teleArmMotor.set(0);
			}
		} else {
			teleArmMotor.set(0);
			pivotMotor.set(0);
		}
	}
	/**
	 * Method to adjust the arm to go shoot on high height to use in autonomous.
	 */
	public void executeShootHigh() {
		while (!withinError(pivotMotor.getEncoder().getPosition(), SHOOT_ANGLE_ENCODER_FORWARD)
			&& teleArmMotor.getEncoder().getPosition() < ARM_ENCODER_HIGH_FORWARD) {
			if (withinError(pivotMotor.getEncoder().getPosition(), SHOOT_ANGLE_ENCODER_FORWARD)) {
				pivotMotor.set(0);
			} else if (pivotMotor.getEncoder().getPosition() < SHOOT_ANGLE_ENCODER_FORWARD) {
				pivotMotor.set(PIVOT_MOTOR_POWER);
			} else if (pivotMotor.getEncoder().getPosition() > SHOOT_ANGLE_ENCODER_FORWARD) {
				pivotMotor.set(-PIVOT_MOTOR_POWER);
			} else {
				pivotMotor.set(0);
			}
			if (teleArmMotor.getEncoder().getPosition() < ARM_ENCODER_HIGH_FORWARD) {
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
		while (!withinError(pivotMotor.getEncoder().getPosition(), SHOOT_ANGLE_ENCODER_FORWARD)
			&& teleArmMotor.getEncoder().getPosition() < ARM_ENCODER_MID_FORWARD) {
			if (withinError(pivotMotor.getEncoder().getPosition(), SHOOT_ANGLE_ENCODER_FORWARD)) {
				pivotMotor.set(0);
			} else if (pivotMotor.getEncoder().getPosition() < SHOOT_ANGLE_ENCODER_FORWARD) {
				pivotMotor.set(PIVOT_MOTOR_POWER);
			} else if (pivotMotor.getEncoder().getPosition() > SHOOT_ANGLE_ENCODER_FORWARD) {
				pivotMotor.set(-PIVOT_MOTOR_POWER);
			} else {
				pivotMotor.set(0);
			}
			if (teleArmMotor.getEncoder().getPosition() < ARM_ENCODER_MID_FORWARD) {
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
		while (!withinError(pivotMotor.getEncoder().getPosition(), BALANCE_ANGLE_ENCODER_ROTATIONS)
			&& teleArmMotor.getEncoder().getPosition() > 0) {
			if (withinError(pivotMotor.getEncoder().getPosition(),
				BALANCE_ANGLE_ENCODER_ROTATIONS)) {
				pivotMotor.set(0);
			} else if (pivotMotor.getEncoder().getPosition() < BALANCE_ANGLE_ENCODER_ROTATIONS) {
				pivotMotor.set(PIVOT_MOTOR_POWER);
			} else if (pivotMotor.getEncoder().getPosition() > BALANCE_ANGLE_ENCODER_ROTATIONS) {
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
		while (!withinError(pivotMotor.getEncoder().getPosition(), GRABBER_ANGLE_ENCODER_ROTATIONS)
			&& teleArmMotor.getEncoder().getPosition() < ARM_ENCODER_GRAB_ROTATIONS) {
			if (withinError(pivotMotor.getEncoder().getPosition(),
				GRABBER_ANGLE_ENCODER_ROTATIONS)) {
				pivotMotor.set(0);
			} else if (pivotMotor.getEncoder().getPosition() < GRABBER_ANGLE_ENCODER_ROTATIONS) {
				pivotMotor.set(PIVOT_MOTOR_POWER);
			} else if (pivotMotor.getEncoder().getPosition() > GRABBER_ANGLE_ENCODER_ROTATIONS) {
				pivotMotor.set(-PIVOT_MOTOR_POWER);
			} else {
				pivotMotor.set(0);
			}
			if (teleArmMotor.getEncoder().getPosition() < ARM_ENCODER_GRAB_ROTATIONS) {
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
