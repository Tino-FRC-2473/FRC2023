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
		SHOOT_HIGH_FORWARD,
		SHOOT_HIGH_BACKWARD,
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
	private static final double PID_PIVOT_MAX_POWER = 0.2;
	private static final double ERROR_ARM = 0.3;
	private static final double PID_CONSTANT_PIVOT_P = 0.00022f;
	private static final double PID_CONSTANT_PIVOT_I = 0.000055f;
	private static final double PID_CONSTANT_PIVOT_D = 0.000008f;
	private static final double PID_CONSTANT_ARM_P = 0.00022f;
	private static final double PID_CONSTANT_ARM_I = 0.000055f;
	private static final double PID_CONSTANT_ARM_D = 0.000008f;
	private static final double PID_ARM_MAX_POWER = 0.2;


	/* ======================== Private variables ======================== */
	private FSMState currentState;

	private CANSparkMax pivotMotor;
	private CANSparkMax teleArmMotor;
	private SparkMaxLimitSwitch pivotLimitSwitchHigh;
	private SparkMaxLimitSwitch pivotLimitSwitchLow;
	private SparkMaxPIDController pidControllerPivot;
	private SparkMaxPIDController pidControllerTeleArm;
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
		pidControllerPivot = pivotMotor.getPIDController();
		pidControllerPivot.setP(PID_CONSTANT_PIVOT_P);
		pidControllerPivot.setI(PID_CONSTANT_PIVOT_I);
		pidControllerPivot.setD(PID_CONSTANT_PIVOT_D);
		pidControllerPivot.setIZone(0);
		pidControllerPivot.setFF(0);
		pidControllerPivot.setOutputRange(-PID_PIVOT_MAX_POWER, PID_PIVOT_MAX_POWER);
		pidControllerTeleArm = teleArmMotor.getPIDController();
		pidControllerTeleArm.setP(PID_CONSTANT_ARM_P);
		pidControllerTeleArm.setI(PID_CONSTANT_ARM_I);
		pidControllerTeleArm.setD(PID_CONSTANT_ARM_D);
		pidControllerTeleArm.setIZone(0);
		pidControllerTeleArm.setFF(0);
		pidControllerTeleArm.setOutputRange(-PID_ARM_MAX_POWER, PID_ARM_MAX_POWER);
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
			handleIdleState();
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
				handleIdleState();
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
			case SHOOT_HIGH_FORWARD:
				handleShootHighForwardState(input);
				break;
			case SHOOT_HIGH_BACKWARD:
				handleShootHighBackwardState(input);
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

	/**
	 * Updates the Autonomous FSMState.
	 * @param state TeleopInput
	 */
	public void updateAuto(FSMState state) {
		SmartDashboard.putString("Current State", " " + currentState);
		SmartDashboard.putNumber("Pivot Motor Rotations", pivotMotor.getEncoder().getPosition());
		SmartDashboard.putNumber("Arm Motor Rotations", teleArmMotor.getEncoder().getPosition());
		SmartDashboard.putBoolean("At Max Height", isMaxHeight());
		SmartDashboard.putBoolean("At Min Height", isMinHeight());

		switch (currentState) {
			case IDLE:
				handleIdleState();
				break;
			case HOMING_STATE:
				handleHomingState(null);
				break;
			case MOVING_TO_START_STATE:
				handleMovingToStartState(null);
				break;
			case ARM_MOVEMENT:
				handleArmMechState(null);
				break;
			case SHOOT_HIGH_FORWARD:
				handleShootHighForwardState(null);
				break;
			case SHOOT_HIGH_BACKWARD:
				handleShootHighBackwardState(null);
			case SHOOT_MID:
				handleShootMidState(null);
				break;
			case SHOOT_LOW_FORWARD:
				handleShootLowState(null);
				break;
			case SUBSTATION_PICKUP:
				handleSubstationPickupState(null);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		FSMState newState = nextState(null);
		if (currentState != newState) {
			System.out.println(newState);
		}
		currentState = newState;
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
				if (input.isShootHighButtonPressed() && input.isThrottleForward()
						&& SpinningIntakeFSM.getObjectType() == SpinningIntakeFSM.ItemType.CUBE
						&& !atArmPosition(SHOOT_HIGH_ANGLE_ENCODER_FORWARD_ROTATIONS,
							ARM_ENCODER_HIGH_FORWARD_CUBE_ROTATIONS)
						|| input.isShootHighButtonPressed() && input.isThrottleForward()
						&& SpinningIntakeFSM.getObjectType() != SpinningIntakeFSM.ItemType.CUBE
						&& !atArmPosition(SHOOT_HIGH_ANGLE_ENCODER_FORWARD_ROTATIONS,
							ARM_ENCODER_HIGH_FORWARD_CONE_ROTATIONS)) {
					return FSMState.SHOOT_HIGH_FORWARD;
				} else if (input.isShootHighButtonPressed() && !input.isThrottleForward()
						&& !atArmPosition(SHOOT_HIGH_ANGLE_ENCODER_BACKWARD_ROTATIONS,
							ARM_ENCODER_HIGH_BACKWARD_ROTATIONS)) {
					return FSMState.SHOOT_HIGH_BACKWARD;
				} else if (input.isShootMidButtonPressed() && input.isThrottleForward()
						&& !atArmPosition(SHOOT_MID_ANGLE_ENCODER_FORWARD_ROTATIONS,
							ARM_ENCODER_MID_FORWARD_ROTATIONS)
						|| input.isShootMidButtonPressed() && !input.isThrottleForward()
						&& !atArmPosition(SHOOT_MID_ANGLE_ENCODER_BACKWARD_ROTATIONS,
							ARM_ENCODER_MID_BACKWARD_ROTATIONS)) {
					return FSMState.SHOOT_MID;
				} else if (input.isShootLowButtonPressed()
						&& !atArmPosition(SHOOT_LOW_ANGLE_ENCODER_ROTATIONS,
							ARM_ENCODER_LOW_ROTATIONS)) {
					return FSMState.SHOOT_LOW_FORWARD;
				} else if (input.isSubstationPickupButtonPressed() && !input.isThrottleForward()
						&& !atArmPosition(SUBSTATION_PICKUP_ANGLE_ENCODER_BACKWARD_ROTATIONS,
							ARM_ENCODER_SUBSTATION_BACKWARD_ROTATIONS)
						|| input.isSubstationPickupButtonPressed() && input.isThrottleForward()
						&& !atArmPosition(SUBSTATION_PICKUP_ANGLE_ENCODER_FORWARD_ROTATIONS,
							ARM_ENCODER_SUBSTATION_FORWARD_ROTATIONS)) {
					return FSMState.SUBSTATION_PICKUP;
				} else if (input.isHomingButtonPressed()) {
					return FSMState.HOMING_STATE;
				} else if (!isMovingAtLimit(input) && isArmMovementInputPressed(input)) {
					return FSMState.ARM_MOVEMENT;
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
				if (isArmMovementInputPressed(input) && !isMovingAtLimit(input)
					&& !isShootOrPickupButtonPressed(input)) {
					return FSMState.ARM_MOVEMENT;
				}
				return FSMState.IDLE;
			case SHOOT_HIGH_FORWARD:
				if (input.isShootHighButtonPressed() && input.isThrottleForward()
					&& SpinningIntakeFSM.getObjectType() == SpinningIntakeFSM.ItemType.CUBE
					&& !atArmPosition(SHOOT_HIGH_ANGLE_ENCODER_FORWARD_ROTATIONS,
						ARM_ENCODER_HIGH_FORWARD_CUBE_ROTATIONS)
					|| input.isShootHighButtonPressed() && input.isThrottleForward()
					&& SpinningIntakeFSM.getObjectType() != SpinningIntakeFSM.ItemType.CUBE
					&& !atArmPosition(SHOOT_HIGH_ANGLE_ENCODER_FORWARD_ROTATIONS,
						ARM_ENCODER_HIGH_FORWARD_CONE_ROTATIONS)) {
					return FSMState.SHOOT_HIGH_FORWARD;
				}
				return FSMState.IDLE;
			case SHOOT_HIGH_BACKWARD:
				if (input.isShootHighButtonPressed() && !input.isThrottleForward()
						&& !atArmPosition(SHOOT_HIGH_ANGLE_ENCODER_BACKWARD_ROTATIONS,
						ARM_ENCODER_HIGH_BACKWARD_ROTATIONS)) {
					return FSMState.SHOOT_HIGH_BACKWARD;
				}
				return FSMState.IDLE;
			case SHOOT_MID:
				if (input.isShootMidButtonPressed() && input.isThrottleForward()
					&& !atArmPosition(SHOOT_MID_ANGLE_ENCODER_FORWARD_ROTATIONS,
						ARM_ENCODER_MID_FORWARD_ROTATIONS)
					|| input.isShootMidButtonPressed() && !input.isThrottleForward()
					&& !atArmPosition(SHOOT_MID_ANGLE_ENCODER_BACKWARD_ROTATIONS,
						ARM_ENCODER_MID_BACKWARD_ROTATIONS)) {
					return FSMState.SHOOT_MID;
				}
				return FSMState.IDLE;
			case SHOOT_LOW_FORWARD:
				if (input.isShootLowButtonPressed()
					&& !atArmPosition(SHOOT_LOW_ANGLE_ENCODER_ROTATIONS,
						ARM_ENCODER_LOW_ROTATIONS)) {
					return FSMState.SHOOT_LOW_FORWARD;
				}
				return FSMState.IDLE;
			case SUBSTATION_PICKUP:
				if (input.isSubstationPickupButtonPressed() && !input.isThrottleForward()
					&& !atArmPosition(SUBSTATION_PICKUP_ANGLE_ENCODER_BACKWARD_ROTATIONS,
						ARM_ENCODER_SUBSTATION_BACKWARD_ROTATIONS)
					|| input.isSubstationPickupButtonPressed() && input.isThrottleForward()
					&& !atArmPosition(SUBSTATION_PICKUP_ANGLE_ENCODER_FORWARD_ROTATIONS,
						ARM_ENCODER_SUBSTATION_FORWARD_ROTATIONS)) {
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
		return Math.abs(a - b) < ERROR_ARM;
	}

	private boolean isMaxHeight() {
		return pivotLimitSwitchHigh.isPressed();
	}

	private boolean isMinHeight() {
		return pivotLimitSwitchLow.isPressed();
	}

	private boolean isArmMovementInputPressed(TeleopInput input) {
		if (input == null) {
			return false;
		}
		return input.isPivotDecreaseButtonPressed()
			|| input.isPivotIncreaseButtonPressed()
			|| input.isExtendButtonPressed()
			|| input.isRetractButtonPressed();
	}

	private boolean isShootOrPickupButtonPressed(TeleopInput input) {
		if (input == null) {
			return false;
		}
		return input.isShootHighButtonPressed()
			|| input.isShootMidButtonPressed()
			|| input.isShootLowButtonPressed()
			|| input.isSubstationPickupButtonPressed();
	}

	private boolean isMovingAtLimit(TeleopInput input) {
		if (input == null) {
			return false;
		}
		return input.isPivotDecreaseButtonPressed() && isMinHeight()
			|| input.isPivotIncreaseButtonPressed() && isMaxHeight();
	}

	private boolean atArmPosition(double pivotTarget, double armTarget) {
		return withinError(pivotMotor.getEncoder().getPosition(), pivotTarget)
			&& withinError(teleArmMotor.getEncoder().getPosition(), armTarget);
	}
	/*
	 * What to do when in the IDLE state
	 */
	private void handleIdleState() {
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
			pidControllerPivot.setReference(ARM_ENCODER_STARTING_ANGLE_ROTATIONS,
				CANSparkMax.ControlType.kPosition);
		}
	}
	/*
	 * What to do when in the ARM_MOVEMENT state
	 */
	private void handleArmMechState(TeleopInput input) {
		if (input != null) {
			if (input.isPivotIncreaseButtonPressed() && !isMaxHeight()) {
				//pivotMotor.set(PIVOT_MOTOR_POWER);
				pidControllerPivot.setReference(PIVOT_MOTOR_POWER,
					CANSparkMax.ControlType.kDutyCycle);
			} else if (input.isPivotDecreaseButtonPressed() && !isMinHeight()) {
				//pivotMotor.set(-PIVOT_MOTOR_POWER);
				pidControllerPivot.setReference(-PIVOT_MOTOR_POWER,
					CANSparkMax.ControlType.kDutyCycle);
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

	private void handleShootHighForwardState(TeleopInput input) {
		if (SpinningIntakeFSM.getObjectType() == SpinningIntakeFSM.ItemType.CUBE) {
			if (withinError(pivotMotor.getEncoder().getPosition(),
				SHOOT_HIGH_ANGLE_ENCODER_FORWARD_ROTATIONS)) {
				pivotMotor.set(0);
			} else {
				pidControllerPivot.setReference(SHOOT_HIGH_ANGLE_ENCODER_FORWARD_ROTATIONS,
					CANSparkMax.ControlType.kPosition);
			}
			if (withinError(teleArmMotor.getEncoder().getPosition(),
				ARM_ENCODER_HIGH_FORWARD_CUBE_ROTATIONS)) {
				teleArmMotor.set(0);
			} else {
				//teleArmMotor.set(TELEARM_MOTOR_POWER);
				pidControllerTeleArm.setReference(ARM_ENCODER_HIGH_FORWARD_CUBE_ROTATIONS,
					CANSparkMax.ControlType.kPosition);
			}
		} else {
			if (withinError(pivotMotor.getEncoder().getPosition(),
				SHOOT_HIGH_ANGLE_ENCODER_FORWARD_ROTATIONS)) {
				pivotMotor.set(0);
			} else {
				pidControllerPivot.setReference(SHOOT_HIGH_ANGLE_ENCODER_FORWARD_ROTATIONS,
					CANSparkMax.ControlType.kPosition);
			}
			if (withinError(teleArmMotor.getEncoder().getPosition(),
				ARM_ENCODER_HIGH_FORWARD_CONE_ROTATIONS)) {
				teleArmMotor.set(0);
			} else {
				//teleArmMotor.set(TELEARM_MOTOR_POWER);
				pidControllerTeleArm.setReference(ARM_ENCODER_HIGH_FORWARD_CONE_ROTATIONS,
					CANSparkMax.ControlType.kPosition);
			}
			teleArmMotor.set(0);
			pivotMotor.set(0);
		}
	}

	private void handleShootHighBackwardState(TeleopInput input) {
		if (input != null) {
			if (withinError(pivotMotor.getEncoder().getPosition(),
				SHOOT_HIGH_ANGLE_ENCODER_BACKWARD_ROTATIONS)) {
				pivotMotor.set(0);
			} else {
				pidControllerPivot.setReference(SHOOT_HIGH_ANGLE_ENCODER_BACKWARD_ROTATIONS,
					CANSparkMax.ControlType.kPosition);
			}
			if (withinError(teleArmMotor.getEncoder().getPosition(),
					ARM_ENCODER_HIGH_BACKWARD_ROTATIONS)) {
				teleArmMotor.set(0);
			} else {
					//teleArmMotor.set(TELEARM_MOTOR_POWER);
				pidControllerTeleArm.setReference(ARM_ENCODER_HIGH_BACKWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
			}
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
					pidControllerPivot.setReference(SHOOT_MID_ANGLE_ENCODER_FORWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
				if (withinError(teleArmMotor.getEncoder().getPosition(),
						ARM_ENCODER_MID_FORWARD_ROTATIONS)) {
					teleArmMotor.set(0);
				} else {
					//teleArmMotor.set(TELEARM_MOTOR_POWER);
					pidControllerTeleArm.setReference(ARM_ENCODER_MID_FORWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
			} else { //throttle is backward/false
				if (withinError(pivotMotor.getEncoder().getPosition(),
					SHOOT_MID_ANGLE_ENCODER_BACKWARD_ROTATIONS)) {
					pivotMotor.set(0);
				} else {
					pidControllerPivot.setReference(SHOOT_MID_ANGLE_ENCODER_BACKWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
				if (withinError(teleArmMotor.getEncoder().getPosition(),
						ARM_ENCODER_MID_BACKWARD_ROTATIONS)) {
					teleArmMotor.set(0);
				} else {
					//teleArmMotor.set(TELEARM_MOTOR_POWER);
					pidControllerTeleArm.setReference(ARM_ENCODER_MID_BACKWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
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
				pidControllerPivot.setReference(SHOOT_LOW_ANGLE_ENCODER_ROTATIONS,
					CANSparkMax.ControlType.kPosition);
			}
			if (withinError(teleArmMotor.getEncoder().getPosition(),
						ARM_ENCODER_LOW_ROTATIONS)) {
				teleArmMotor.set(0);
			} else {
				//teleArmMotor.set(TELEARM_MOTOR_POWER);
				pidControllerTeleArm.setReference(ARM_ENCODER_LOW_ROTATIONS,
					CANSparkMax.ControlType.kPosition);
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
					pidControllerPivot.setReference(
						SUBSTATION_PICKUP_ANGLE_ENCODER_FORWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
				if (withinError(teleArmMotor.getEncoder().getPosition(),
						ARM_ENCODER_SUBSTATION_FORWARD_ROTATIONS)) {
					teleArmMotor.set(0);
				} else {
					//teleArmMotor.set(TELEARM_MOTOR_POWER);
					pidControllerTeleArm.setReference(ARM_ENCODER_SUBSTATION_FORWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
			} else { //throttle is backward/false
				if (withinError(pivotMotor.getEncoder().getPosition(),
					SUBSTATION_PICKUP_ANGLE_ENCODER_BACKWARD_ROTATIONS)) {
					pivotMotor.set(0);
				} else {
					pidControllerPivot.setReference(
						SUBSTATION_PICKUP_ANGLE_ENCODER_BACKWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
				if (withinError(teleArmMotor.getEncoder().getPosition(),
					ARM_ENCODER_SUBSTATION_BACKWARD_ROTATIONS)) {
					teleArmMotor.set(0);
				} else {
					//teleArmMotor.set(TELEARM_MOTOR_POWER);
					pidControllerTeleArm.setReference(ARM_ENCODER_SUBSTATION_BACKWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
			}
		} else {
			teleArmMotor.set(0);
			pivotMotor.set(0);
		}
	}
	/**
	 * Method to adjust the arm to go shoot on high height to use in autonomous.
	 * @param isBackwards Boolean value for whether or not it should shoot backward or forward.
	 */
	public void executeShootHigh(boolean isBackwards) {
		if (isBackwards) {
			while (!withinError(pivotMotor.getEncoder().getPosition(),
				SHOOT_HIGH_ANGLE_ENCODER_BACKWARD_ROTATIONS)
				&& teleArmMotor.getEncoder().getPosition()
				< ARM_ENCODER_HIGH_BACKWARD_ROTATIONS) {
				if (withinError(pivotMotor.getEncoder().getPosition(),
					SHOOT_HIGH_ANGLE_ENCODER_BACKWARD_ROTATIONS)) {
					pivotMotor.set(0);
				} else {
					pidControllerPivot.setReference(SHOOT_HIGH_ANGLE_ENCODER_BACKWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
				if (withinError(teleArmMotor.getEncoder().getPosition(),
					ARM_ENCODER_HIGH_BACKWARD_ROTATIONS)) {
					pidControllerTeleArm.setReference(ARM_ENCODER_HIGH_BACKWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				} else {
					teleArmMotor.set(0);
				}
			}
		} else {
			while (!withinError(pivotMotor.getEncoder().getPosition(),
				SHOOT_HIGH_ANGLE_ENCODER_FORWARD_ROTATIONS)
				&& teleArmMotor.getEncoder().getPosition()
				< ARM_ENCODER_HIGH_FORWARD_CUBE_ROTATIONS) {
				if (withinError(pivotMotor.getEncoder().getPosition(),
					SHOOT_HIGH_ANGLE_ENCODER_FORWARD_ROTATIONS)) {
					pivotMotor.set(0);
				} else {
					pidControllerPivot.setReference(SHOOT_HIGH_ANGLE_ENCODER_FORWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
				if (withinError(teleArmMotor.getEncoder().getPosition(),
					ARM_ENCODER_HIGH_FORWARD_CUBE_ROTATIONS)) {
					pidControllerTeleArm.setReference(ARM_ENCODER_HIGH_FORWARD_CUBE_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				} else {
					teleArmMotor.set(0);
				}
			}
		}
		pivotMotor.set(0);
		teleArmMotor.set(0);
	}

	/**
	 * Method to adjust the arm to go shoot on mid height to use in autonomous.
	 * @param isBackwards Boolean value for whether or not it should shoot backward or forward.
	 */
	public void executeShootMid(boolean isBackwards) {
		if (isBackwards) {
			while (!withinError(pivotMotor.getEncoder().getPosition(),
				SHOOT_MID_ANGLE_ENCODER_BACKWARD_ROTATIONS)
				&& teleArmMotor.getEncoder().getPosition()
				< ARM_ENCODER_MID_BACKWARD_ROTATIONS) {
				if (withinError(pivotMotor.getEncoder().getPosition(),
					SHOOT_MID_ANGLE_ENCODER_BACKWARD_ROTATIONS)) {
					pivotMotor.set(0);
				} else {
					pidControllerPivot.setReference(SHOOT_MID_ANGLE_ENCODER_BACKWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
				if (withinError(teleArmMotor.getEncoder().getPosition(),
					ARM_ENCODER_MID_BACKWARD_ROTATIONS)) {
					pidControllerTeleArm.setReference(ARM_ENCODER_MID_BACKWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				} else {
					teleArmMotor.set(0);
				}
			}
		} else {
			while (!withinError(pivotMotor.getEncoder().getPosition(),
				SHOOT_MID_ANGLE_ENCODER_FORWARD_ROTATIONS)
				&& teleArmMotor.getEncoder().getPosition()
				< ARM_ENCODER_MID_FORWARD_ROTATIONS) {
				if (withinError(pivotMotor.getEncoder().getPosition(),
					SHOOT_MID_ANGLE_ENCODER_FORWARD_ROTATIONS)) {
					pivotMotor.set(0);
				} else {
					pidControllerPivot.setReference(SHOOT_MID_ANGLE_ENCODER_FORWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
				if (withinError(teleArmMotor.getEncoder().getPosition(),
					ARM_ENCODER_MID_FORWARD_ROTATIONS)) {
					pidControllerTeleArm.setReference(ARM_ENCODER_MID_FORWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				} else {
					teleArmMotor.set(0);
				}
			}
		}
		pivotMotor.set(0);
		teleArmMotor.set(0);
	}

	/**
	 * Method to adjust the arm to go shoot on low height to use in autonomous.
	 */
	public void executeShootLow() {
		while (!withinError(pivotMotor.getEncoder().getPosition(),
			SHOOT_LOW_ANGLE_ENCODER_ROTATIONS)
			&& teleArmMotor.getEncoder().getPosition()
			< ARM_ENCODER_LOW_ROTATIONS) {
			if (withinError(pivotMotor.getEncoder().getPosition(),
				SHOOT_LOW_ANGLE_ENCODER_ROTATIONS)) {
				pivotMotor.set(0);
			} else {
				pidControllerPivot.setReference(SHOOT_LOW_ANGLE_ENCODER_ROTATIONS,
					CANSparkMax.ControlType.kPosition);
			}
			if (withinError(teleArmMotor.getEncoder().getPosition(),
				ARM_ENCODER_LOW_ROTATIONS)) {
				pidControllerTeleArm.setReference(ARM_ENCODER_LOW_ROTATIONS,
					CANSparkMax.ControlType.kPosition);
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
		while (!withinError(pivotMotor.getEncoder().getPosition(),
			BALANCE_ANGLE_ENCODER_ROTATIONS)
			&& teleArmMotor.getEncoder().getPosition()
			> 0) {
			if (withinError(pivotMotor.getEncoder().getPosition(),
				BALANCE_ANGLE_ENCODER_ROTATIONS)) {
				pivotMotor.set(0);
			} else {
				pidControllerPivot.setReference(BALANCE_ANGLE_ENCODER_ROTATIONS,
					CANSparkMax.ControlType.kPosition);
			}
			if (withinError(teleArmMotor.getEncoder().getPosition(), 0)) {
				pidControllerTeleArm.setReference(0,
					CANSparkMax.ControlType.kPosition);
			} else {
				teleArmMotor.set(0);
			}
		}
		pivotMotor.set(0);
		teleArmMotor.set(0);
	}

	/**
	 * Method to move arm for grabbing from double substation in autonomous.
	 */
	public void executeGrabberArm() {
		while (!withinError(pivotMotor.getEncoder().getPosition(),
			GRABBER_ANGLE_ENCODER_ROTATIONS)
			&& teleArmMotor.getEncoder().getPosition()
			< ARM_ENCODER_GRAB_ROTATIONS) {
			if (withinError(pivotMotor.getEncoder().getPosition(),
				GRABBER_ANGLE_ENCODER_ROTATIONS)) {
				pivotMotor.set(0);
			} else {
				pidControllerPivot.setReference(GRABBER_ANGLE_ENCODER_ROTATIONS,
					CANSparkMax.ControlType.kPosition);
			}
			if (withinError(teleArmMotor.getEncoder().getPosition(),
				ARM_ENCODER_GRAB_ROTATIONS)) {
				pidControllerTeleArm.setReference(ARM_ENCODER_GRAB_ROTATIONS,
					CANSparkMax.ControlType.kPosition);
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
			pidControllerTeleArm.setReference(teleArmMotor.getEncoder().getPosition(),
				CANSparkMax.ControlType.kPosition);
		}
		teleArmMotor.set(0);
	}
}
