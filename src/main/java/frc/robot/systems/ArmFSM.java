package frc.robot.systems;



// WPILib Imports

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
	private static final int ARM_ENCODER_HIGH = 500;
	private static final int ARM_ENCODER_MID = 300;
	private static final int SHOOT_ANGLE_ENCODER = 300;


	/* ======================== Private variables ======================== */
	private FSMState currentState;

	private CANSparkMax pivotMotor;
	private CANSparkMax teleArmMotor;
	private SparkMaxLimitSwitch pivotLimitSwitchHigh;
	private SparkMaxLimitSwitch pivotLimitSwitchLow;
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
		switch (currentState) {
			case IDLE:
				handleIdleState(input);
				break;
			case ARM_MOVEMENT:
				handleArmMechState(input);
			case SHOOT_HIGH:
				handleShootHighState(input);
			case SHOOT_MID:
				handleShootMidState(input);
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		currentState = nextState(input);
	}

	/*
	 * When inputs are pressed, the states will change likewise
	 *
	 * @return expected state
	 */
	private FSMState nextState(TeleopInput input) {
		switch (currentState) {
			case IDLE:
				if (input != null && !input.isShootHighButtonPressed()
					&& !input.isShootMidButtonPressed()) {
					return FSMState.ARM_MOVEMENT;
				} else if (input.isShootHighButtonPressed()) {
					return FSMState.SHOOT_HIGH;
				} else if (input.isShootMidButtonPressed()) {
					return FSMState.SHOOT_MID;
				}
				return FSMState.IDLE;
			case ARM_MOVEMENT:
				if (input == null) {
					return FSMState.IDLE;
				} else if (input.isShootHighButtonPressed()) {
					return FSMState.SHOOT_HIGH;
				} else if (input.isShootMidButtonPressed()) {
					return FSMState.SHOOT_MID;
				}
				return FSMState.ARM_MOVEMENT;
			case SHOOT_HIGH:
				if (input.isShootHighButtonPressed()) {
					return FSMState.SHOOT_HIGH;
				}
				return FSMState.IDLE;
			case SHOOT_MID:
				if (input.isShootMidButtonPressed()) {
					return FSMState.SHOOT_HIGH;
				}
				return FSMState.IDLE;
			default :
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}


	private boolean atMaxHeight() {
		return pivotLimitSwitchHigh.isPressed();
	}

	private boolean atMinHeight() {
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
		if (input.isPivotIncreaseButtonPressed() && !atMaxHeight()) {
			pivotMotor.set(PIVOT_MOTOR_POWER);
		} else if (input.isPivotDecreaseButtonPressed() && !atMinHeight()) {
			pivotMotor.set(-PIVOT_MOTOR_POWER);
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
		if (pivotMotor.getEncoder().getPosition() < SHOOT_ANGLE_ENCODER) {
			pivotMotor.set(-PIVOT_MOTOR_POWER);
		} else if (pivotMotor.getEncoder().getPosition() > SHOOT_ANGLE_ENCODER) {
			pivotMotor.set(PIVOT_MOTOR_POWER);
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
		if (pivotMotor.getEncoder().getPosition() < SHOOT_ANGLE_ENCODER) {
			pivotMotor.set(-PIVOT_MOTOR_POWER);
		} else if (pivotMotor.getEncoder().getPosition() > SHOOT_ANGLE_ENCODER) {
			pivotMotor.set(PIVOT_MOTOR_POWER);
		} else {
			pivotMotor.set(0);
		}
		if (teleArmMotor.getEncoder().getPosition() < ARM_ENCODER_MID) {
			teleArmMotor.set(TELEARM_MOTOR_POWER);
		} else {
			teleArmMotor.set(0);
		}
	}

}
