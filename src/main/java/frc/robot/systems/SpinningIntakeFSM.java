package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj.I2C.Port;
// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class SpinningIntakeFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		START_STATE,
		IDLE_SPINNING,
		IDLE_STOP,
		RELEASE
	}
	//FIX VALUES
	private static final double INTAKE_SPEED = 0.1;
	private static final double RELEASE_SPEED = -0.1;
	private static final double RELEASE_TIME = 2;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax spinnerMotor;
	private SparkMaxLimitSwitch limitSwitchCone;
	private ColorSensorV3 colorSensorCube;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public SpinningIntakeFSM() {
		// Perform hardware init
		spinnerMotor = new CANSparkMax(HardwareMap.CAN_ID_SPINNER_MOTOR,
										CANSparkMax.MotorType.kBrushless);
		limitSwitchCone = spinnerMotor.getReverseLimitSwitch(
			SparkMaxLimitSwitch.Type.kNormallyOpen);
		limitSwitchCone.enableLimitSwitch(true);
		colorSensorCube = new ColorSensorV3(Port.kOnboard);
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
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		switch (currentState) {
			case START_STATE:
				handleStartState(input);
				break;
			case IDLE_SPINNING:
				handleIdleSpinningState(input);
				break;
			case IDLE_STOP:
				handleIdleStopState(input);
				break;
			case RELEASE:
				handleReleaseState(input);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		System.out.println(colorSensorCube.getProximity());
		currentState = nextState(input);
	}

	/*-------------------------NON HANDLER METHODS ------------------------- */
	private boolean isCubeDetected() {
		//FIX THIS LATER
		final int colorProx = 100;
		return colorSensorCube.getProximity() < colorProx;
	}
	private boolean isLimitSwitchConeActivated() {
		//FIX THIS LATER;
		return limitSwitchCone.isPressed();
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
			case START_STATE:
				return FSMState.IDLE_SPINNING;
			case IDLE_SPINNING:
				if (isCubeDetected() || isLimitSwitchConeActivated()) {
					return FSMState.IDLE_STOP;
				}
				return FSMState.IDLE_SPINNING;
			case IDLE_STOP:
				if (input.isReleaseButtonPressed()) {
					return FSMState.RELEASE;
				}
				return FSMState.IDLE_STOP;
			case RELEASE:
				if (input.isReleaseButtonReleased()) {
					return FSMState.IDLE_SPINNING;
				}
				return FSMState.RELEASE;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in START_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleStartState(TeleopInput input) {

	}
	private void handleIdleSpinningState(TeleopInput input) {
		spinnerMotor.set(INTAKE_SPEED);
	}
	private void handleIdleStopState(TeleopInput input) {
		spinnerMotor.set(0);
	}
	private void handleReleaseState(TeleopInput input) {
		spinnerMotor.set(RELEASE_SPEED);
	}
}
