package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
import com.revrobotics.SparkMaxLimitSwitch;

public class GroundMountFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		START_STATE,
		PIVOTING_UP,
		PIVOTED_UP,
		PIVOTING_DOWN,
		PIVOTED_DOWN,
		AUTONOMOUS_UP,
		AUTONOMOUS_DOWN,
		AUTONOMOUS_IDLE
	}
	//arbitrary constants, must test all of these
	private static final double PIVOT_DOWN_POWER = -0.05;
	private static final double PIVOT_UP_POWER = 0.05;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax pivotArmMotor;
	private SparkMaxLimitSwitch limitSwitchHigh;
	private SparkMaxLimitSwitch limitSwitchLow;


	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public GroundMountFSM() {
		// Perform hardware init
		if (HardwareMap.isTestBoardGroundMount()) {
			pivotArmMotor = new CANSparkMax(HardwareMap.CAN_ID_GROUND_MOUNT,
										CANSparkMax.MotorType.kBrushless);
		} else {
			pivotArmMotor = new CANSparkMax(HardwareMap.CAN_ID_GROUND_MOUNT,
										CANSparkMax.MotorType.kBrushless);
		}
		limitSwitchHigh = pivotArmMotor.getForwardLimitSwitch(
								SparkMaxLimitSwitch.Type.kNormallyClosed);
		limitSwitchHigh.enableLimitSwitch(true);
		limitSwitchLow = pivotArmMotor.getReverseLimitSwitch(
								SparkMaxLimitSwitch.Type.kNormallyClosed);
		limitSwitchLow.enableLimitSwitch(true);
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
		currentState = FSMState.START_STATE;
		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * updateAutonomous function.
	 * @param state that the autonomous is in.
	 * @return a boolean
	 */
	public boolean updateAutonomous(FSMState state) {
		SmartDashboard.putNumber("power", pivotArmMotor.get());
		SmartDashboard.putBoolean("limit switch low", isLimitSwitchLowPressed());
		SmartDashboard.putBoolean("limit switch high", isLimitSwitchHighPressed());

		switch (state) {
			case AUTONOMOUS_UP:
				handleAutonomousUpState();
				return isLimitSwitchHighPressed();
			case AUTONOMOUS_DOWN:
				handleAutonomousDownState();
				return isLimitSwitchLowPressed();
			case AUTONOMOUS_IDLE:
				handleAutonomousIdleState();
				return true;
			default: throw new IllegalStateException("Invalid state: " + state.toString());
		}
	}
	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		//System.out.println(itemType);
		SmartDashboard.putNumber("encoder", pivotArmMotor.getEncoder().getPosition());
		SmartDashboard.putString("state", currentState.toString());
		SmartDashboard.putNumber("power", pivotArmMotor.get());
		SmartDashboard.putBoolean("limit low", isLimitSwitchLowPressed());
		SmartDashboard.putBoolean("limit high", isLimitSwitchHighPressed());
		//System.out.println(distanceSensorObject.getValue() + " " + itemType);
		if (input == null) {
			return;
		}
		switch (currentState) {
			case START_STATE:
				handleStartState();
				break;
			case PIVOTED_UP:
				handlePivotedUpState();
				break;
			case PIVOTING_DOWN:
				handlePivotingDownState();
				break;
			case PIVOTED_DOWN:
				handlePivotedDownState();
				break;
			case PIVOTING_UP:
				handlePivotingUpState();
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
		if (input == null) {
			return FSMState.START_STATE;
		}

		switch (currentState) {
			case START_STATE:
				return FSMState.PIVOTING_UP;
			case PIVOTING_UP:
				if (input.isPivotButtonPressed()) {
					return FSMState.PIVOTING_DOWN;
				}
				if (isLimitSwitchHighPressed()) {
					return FSMState.PIVOTED_UP;
				}
				//means pivot button is not pressed and limit switch not activated, stay in state
				return FSMState.PIVOTING_UP;
			case PIVOTED_UP:
				if (input.isPivotButtonPressed()) {
					return FSMState.PIVOTING_DOWN;
				}
				if (!input.isPivotButtonPressed() && !isLimitSwitchHighPressed()) {
					return FSMState.PIVOTING_UP;
				}
				//means pivot button not pressed and limit switch activated, stay in state
				return FSMState.PIVOTED_UP;
			case PIVOTING_DOWN:
				if (!input.isPivotButtonPressed()) {
					return FSMState.PIVOTING_UP;
				}
				if (isLimitSwitchLowPressed() && input.isPivotButtonPressed()) {
					return FSMState.PIVOTED_DOWN;
				}
				//means limit switch low not activated and pivot button still pressed, stay in state
				return FSMState.PIVOTING_DOWN;
			case PIVOTED_DOWN:
				if (!input.isPivotButtonPressed()) {
					return FSMState.PIVOTING_UP;
				}
				if (input.isPivotButtonPressed() && !isLimitSwitchLowPressed()) {
					return FSMState.PIVOTING_DOWN;
				}
				//means pivot button is pressed and limit switch low active, stay in state
				return FSMState.PIVOTED_DOWN;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	private boolean isLimitSwitchHighPressed() {
		return limitSwitchHigh.isPressed();
	}
	private boolean isLimitSwitchLowPressed() {
		return limitSwitchLow.isPressed();
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in states.
	 */
	private void handleStartState() {
		//do nothing
	}
	private void handlePivotedUpState() {
		pivotArmMotor.set(0);
	}
	private void handlePivotingUpState() {
		pivotArmMotor.set(PIVOT_UP_POWER);
	}
	private void handlePivotedDownState() {
		pivotArmMotor.set(0);
	}
	private void handlePivotingDownState() {
		pivotArmMotor.set(PIVOT_DOWN_POWER);
	}

	/* AUTONOMOUS HANDLES */

	private void handleAutonomousDownState() {
		if (isLimitSwitchLowPressed()) {
			pivotArmMotor.set(0);
			return;
		}
		pivotArmMotor.set(PIVOT_DOWN_POWER);
	}

	private void handleAutonomousUpState() {
		if (isLimitSwitchHighPressed()) {
			pivotArmMotor.set(0);
			return;
		}
		pivotArmMotor.set(PIVOT_UP_POWER);
	}

	private void handleAutonomousIdleState() {
		pivotArmMotor.set(0);
	}
}
