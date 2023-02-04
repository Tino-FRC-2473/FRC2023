package frc.robot.systems;



// WPILib Imports
//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
		HOMING_STATE,
		MOVING_TO_START_STATE,
		ARM_MOVEMENT,
		SHOOT_HIGH,
		SHOOT_MID,
		SHOOT_LOW_FORWARD,
		SUBSTATION_PICKUP
	}

	private static final float TELEARM_MOTOR_POWER = 0.1f;
	private static final float PIVOT_MOTOR_POWER = 0.1f;
	private static final double ARM_ENCODER_STARTING_ANGLE_ROTATIONS = 20;
	private static final double ARM_ENCODER_HIGH_FORWARD_CUBE_ROTATIONS = 20;
	private static final double ARM_ENCODER_HIGH_FORWARD_CONE_ROTATIONS = 20;
	private static final double ARM_ENCODER_HIGH_BACKWARD_ROTATIONS = 40;
	private static final double ARM_ENCODER_MID_FORWARD_ROTATIONS = 10;
	private static final double ARM_ENCODER_MID_BACKWARD_ROTATIONS = 10;
	private static final double ARM_ENCODER_SUBSTATION_FORWARD_ROTATIONS = 10;
	private static final double ARM_ENCODER_SUBSTATION_BACKWARD_ROTATIONS = 10;
	private static final double ARM_ENCODER_LOW_ROTATIONS = 10;
	private static final double SHOOT_MID_ANGLE_ENCODER_FORWARD_ROTATIONS = 30;
	private static final double SHOOT_HIGH_ANGLE_ENCODER_FORWARD_ROTATIONS = 30;
	private static final double SHOOT_MID_ANGLE_ENCODER_BACKWARD_ROTATIONS = 30;
	private static final double SHOOT_HIGH_ANGLE_ENCODER_BACKWARD_ROTATIONS = 30;
	private static final double SHOOT_LOW_ANGLE_ENCODER_ROTATIONS = -10;
	private static final double SUBSTATION_PICKUP_ANGLE_ENCODER_FORWARD_ROTATIONS = 30;
	private static final double SUBSTATION_PICKUP_ANGLE_ENCODER_BACKWARD_ROTATIONS = 50;
	private static final double BALANCE_ANGLE_ENCODER_ROTATIONS = 5;
	private static final double GRABBER_ANGLE_ENCODER_ROTATIONS = -5;
	private static final double ARM_ENCODER_GRAB_ROTATIONS = 10;
	private static final double PID_MAX_POWER = 0.2;
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
		SmartDashboard.putString("Current State", " " + currentState);
		SmartDashboard.putNumber("Pivot Motor Rotations", pivotMotor.getEncoder().getPosition());
		SmartDashboard.putNumber("Arm Motor Rotations", teleArmMotor.getEncoder().getPosition());
		SmartDashboard.putBoolean("At Max Height", isMaxHeight());
		SmartDashboard.putBoolean("At Min Height", isMinHeight());
		SmartDashboard.putBoolean("Is going Forward", input.isThrottleForward());
		SmartDashboard.putNumber("Throttle Value", input.getThrottle());
		switch (currentState) {
			case IDLE:
				handleIdleState(input);
				break;
			case HOMING_STATE:
				handleHomingState(input);
				break;
			case MOVING_TO_START_STATE:
				handleMovingToStartState(input);
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
			case SUBSTATION_PICKUP:
				handleSubstationPickupState(input);
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
					&& !input.isShootHighButtonPressed() && !input.isShootMidButtonPressed()
					&& !input.isHomingButtonPressed()) {
					return FSMState.ARM_MOVEMENT;
				} else if (input.isShootHighButtonPressed()) {
					return FSMState.SHOOT_HIGH;
				} else if (input.isShootMidButtonPressed()) {
					return FSMState.SHOOT_MID;
				} else if (input.isShootLowButtonPressed()) {
					return FSMState.SHOOT_LOW_FORWARD;
				} else if (input.isSubstationPickupButtonPressed()) {
					return FSMState.SUBSTATION_PICKUP;
				} else if (input.isHomingButtonPressed()) {
					return FSMState.HOMING_STATE;
				}
				return FSMState.IDLE;
			case HOMING_STATE:
				if (isMinHeight()) {
					return FSMState.MOVING_TO_START_STATE;
				} else {
					return FSMState.HOMING_STATE;
				}
			case MOVING_TO_START_STATE:
				if (withinError(pivotMotor.getEncoder().getPosition(),
					ARM_ENCODER_STARTING_ANGLE_ROTATIONS)) {
					return FSMState.IDLE;
				} else {
					return FSMState.MOVING_TO_START_STATE;
				}
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
			case SUBSTATION_PICKUP:
				if (input.isSubstationPickupButtonPressed()) {
					return FSMState.SUBSTATION_PICKUP;
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


	private void handleHomingState(TeleopInput input) {
		if (isMinHeight()) {
			pivotMotor.set(0);
		} else {
			pivotMotor.set(-PIVOT_MOTOR_POWER);
		}
	}

	private void handleMovingToStartState(TeleopInput input) {
		if (withinError(pivotMotor.getEncoder().getPosition(),
			ARM_ENCODER_STARTING_ANGLE_ROTATIONS)) {
			pivotMotor.set(0);
		} else {
			pidController.setReference(ARM_ENCODER_STARTING_ANGLE_ROTATIONS,
				CANSparkMax.ControlType.kPosition);
		}
	}
	/*
	 * What to do when in the ARM_MOVEMENT state
	 */
	private void handleArmMechState(TeleopInput input) {
		if (input != null) {
			if (input.isPivotIncreaseButtonPressed() && !isMaxHeight()) {
				pivotMotor.set(PIVOT_MOTOR_POWER);
				//pidController.setReference(PIVOT_MOTOR_POWER, CANSparkMax.ControlType.kDutyCycle);
			} else if (input.isPivotDecreaseButtonPressed() && !isMinHeight()) {
				pivotMotor.set(-PIVOT_MOTOR_POWER);
				//pidController.setReference(-PIVOT_MOTOR_POWER,
				//CANSparkMax.ControlType.kDutyCycle);
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
				if (SpinningIntakeFSM.getObjectType() != 1) {
					if (withinError(pivotMotor.getEncoder().getPosition(),
						SHOOT_HIGH_ANGLE_ENCODER_FORWARD_ROTATIONS)) {
						pivotMotor.set(0);
					} else {
						pidController.setReference(SHOOT_HIGH_ANGLE_ENCODER_FORWARD_ROTATIONS,
							CANSparkMax.ControlType.kPosition);
					}
					if (teleArmMotor.getEncoder().getPosition()
							< ARM_ENCODER_HIGH_FORWARD_CUBE_ROTATIONS) {
						teleArmMotor.set(TELEARM_MOTOR_POWER);
					} else {
						teleArmMotor.set(0);
					}
				} else {
					if (withinError(pivotMotor.getEncoder().getPosition(),
						SHOOT_HIGH_ANGLE_ENCODER_FORWARD_ROTATIONS)) {
						pivotMotor.set(0);
					} else {
						pidController.setReference(SHOOT_HIGH_ANGLE_ENCODER_FORWARD_ROTATIONS,
							CANSparkMax.ControlType.kPosition);
					}
					if (teleArmMotor.getEncoder().getPosition()
							< ARM_ENCODER_HIGH_FORWARD_CONE_ROTATIONS) {
						teleArmMotor.set(TELEARM_MOTOR_POWER);
					} else {
						teleArmMotor.set(0);
					}
				}
			} else { //throttle is backward/false
				if (withinError(pivotMotor.getEncoder().getPosition(),
					SHOOT_HIGH_ANGLE_ENCODER_BACKWARD_ROTATIONS)) {
					pivotMotor.set(0);
				} else {
					pidController.setReference(SHOOT_HIGH_ANGLE_ENCODER_BACKWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
				if (teleArmMotor.getEncoder().getPosition() < ARM_ENCODER_HIGH_BACKWARD_ROTATIONS) {
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
					SHOOT_MID_ANGLE_ENCODER_FORWARD_ROTATIONS)) {
					pivotMotor.set(0);
				} else {
					pidController.setReference(SHOOT_MID_ANGLE_ENCODER_FORWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
				if (teleArmMotor.getEncoder().getPosition() < ARM_ENCODER_MID_FORWARD_ROTATIONS) {
					teleArmMotor.set(TELEARM_MOTOR_POWER);
				} else {
					teleArmMotor.set(0);
				}
			} else { //throttle is backward/false
				if (withinError(pivotMotor.getEncoder().getPosition(),
					SHOOT_MID_ANGLE_ENCODER_BACKWARD_ROTATIONS)) {
					pivotMotor.set(0);
				} else {
					pidController.setReference(SHOOT_MID_ANGLE_ENCODER_BACKWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
				if (teleArmMotor.getEncoder().getPosition() < ARM_ENCODER_MID_BACKWARD_ROTATIONS) {
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
			} else {
				pidController.setReference(SHOOT_LOW_ANGLE_ENCODER_ROTATIONS,
					CANSparkMax.ControlType.kPosition);
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


	private void handleSubstationPickupState(TeleopInput input) {
		if (input != null) {
			if (input.isThrottleForward()) {
				if (withinError(pivotMotor.getEncoder().getPosition(),
					SUBSTATION_PICKUP_ANGLE_ENCODER_FORWARD_ROTATIONS)) {
					pivotMotor.set(0);
				} else {
					pidController.setReference(SUBSTATION_PICKUP_ANGLE_ENCODER_FORWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
				if (teleArmMotor.getEncoder().getPosition()
					< ARM_ENCODER_SUBSTATION_FORWARD_ROTATIONS) {
					teleArmMotor.set(TELEARM_MOTOR_POWER);
				} else {
					teleArmMotor.set(0);
				}
			} else { //throttle is backward/false
				if (withinError(pivotMotor.getEncoder().getPosition(),
					SUBSTATION_PICKUP_ANGLE_ENCODER_BACKWARD_ROTATIONS)) {
					pivotMotor.set(0);
				} else {
					pidController.setReference(SUBSTATION_PICKUP_ANGLE_ENCODER_BACKWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
				if (teleArmMotor.getEncoder().getPosition()
					< ARM_ENCODER_SUBSTATION_BACKWARD_ROTATIONS) {
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
	/**
	 * Method to adjust the arm to go shoot on high height to use in autonomous.
	 */
	public void executeShootHigh() {
		while (!withinError(pivotMotor.getEncoder().getPosition(),
			SHOOT_MID_ANGLE_ENCODER_FORWARD_ROTATIONS)
			&& teleArmMotor.getEncoder().getPosition() < ARM_ENCODER_HIGH_FORWARD_CUBE_ROTATIONS) {
			if (withinError(pivotMotor.getEncoder().getPosition(),
				SHOOT_MID_ANGLE_ENCODER_FORWARD_ROTATIONS)) {
				pivotMotor.set(0);
			} else if (pivotMotor.getEncoder().getPosition()
				< SHOOT_MID_ANGLE_ENCODER_FORWARD_ROTATIONS) {
				pivotMotor.set(PIVOT_MOTOR_POWER);
			} else if (pivotMotor.getEncoder().getPosition()
				> SHOOT_MID_ANGLE_ENCODER_FORWARD_ROTATIONS) {
				pivotMotor.set(-PIVOT_MOTOR_POWER);
			} else {
				pivotMotor.set(0);
			}
			if (teleArmMotor.getEncoder().getPosition() < ARM_ENCODER_HIGH_FORWARD_CUBE_ROTATIONS) {
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
		while (!withinError(pivotMotor.getEncoder().getPosition(),
			SHOOT_MID_ANGLE_ENCODER_FORWARD_ROTATIONS)
			&& teleArmMotor.getEncoder().getPosition() < ARM_ENCODER_MID_FORWARD_ROTATIONS) {
			if (withinError(pivotMotor.getEncoder().getPosition(),
				SHOOT_MID_ANGLE_ENCODER_FORWARD_ROTATIONS)) {
				pivotMotor.set(0);
			} else if (pivotMotor.getEncoder().getPosition()
				< SHOOT_MID_ANGLE_ENCODER_FORWARD_ROTATIONS) {
				pivotMotor.set(PIVOT_MOTOR_POWER);
			} else if (pivotMotor.getEncoder().getPosition()
				> SHOOT_MID_ANGLE_ENCODER_FORWARD_ROTATIONS) {
				pivotMotor.set(-PIVOT_MOTOR_POWER);
			} else {
				pivotMotor.set(0);
			}
			if (teleArmMotor.getEncoder().getPosition() < ARM_ENCODER_MID_FORWARD_ROTATIONS) {
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
