package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C.Port;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.util.Color;

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
	private static final int COLOR_PROXIMITY_THRESHOLD = 100;

	//CUBE RGB THRESHOLD VALUES
	private static final double RED_AVG = 65 / 256f;
	private static final double GREEN_AVG = 97 / 256f;
	private static final double BLUE_AVG = 92 / 256f;
	private static final double TOLERANCE = 15 / 256f;


	private static final int HEX_BASE = 16;
	private static final int RED_START = 1;
	private static final int RED_END = 3;
	private static final int GREEN_START = 3;
	private static final int GREEN_END = 5;
	private static final int BLUE_START = 5;
	private static final int BLUE_END = 7;


	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax spinnerMotor;
	private DigitalInput limitSwitchCone;
	private ColorSensorV3 colorSensorCube;
	private ColorMatch colorMatch;

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
		limitSwitchCone = new DigitalInput(0);
		colorSensorCube = new ColorSensorV3(Port.kOnboard);
		colorMatch = new ColorMatch();
		colorMatch.addColorMatch(Color.kPurple);
		colorMatch.addColorMatch(Color.kYellow);

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
		if (input == null) {
			return;
		}
		switch (currentState) {
			case START_STATE:
				handleStartState();
				break;
			case IDLE_SPINNING:
				handleIdleSpinningState();
				break;
			case IDLE_STOP:
				handleIdleStopState();
				break;
			case RELEASE:
				handleReleaseState();
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}

		currentState = nextState(input);
	}

	/*-------------------------NON HANDLER METHODS ------------------------- */
	private boolean isCubeDetected() {
		boolean objectDetected = colorSensorCube.getProximity() > COLOR_PROXIMITY_THRESHOLD;
		double r =  colorSensorCube.getColor().red;
		double g =  colorSensorCube.getColor().green;
		double b =  colorSensorCube.getColor().blue;

		System.out.println(r + " " + g + " " + b);

		if (objectDetected && withinRange(r, RED_AVG)
			&& withinRange(g, GREEN_AVG) && withinRange(b, BLUE_AVG)) {
			System.out.println("THIS IS A CUBE");
		} else if (objectDetected) {
			System.out.println("THIS IS A CONE");
		}

		return false;
		//return !isCone && objectDetected;
	}

	private boolean withinRange(double a, double b) {
		return Math.abs(a - b) <= TOLERANCE;
	}
	private boolean isLimitSwitchConeActivated() {
		return limitSwitchCone.get();
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
	 * Handle behavior in states.
	 */
	private void handleStartState() {
	}
	private void handleIdleSpinningState() {
		spinnerMotor.set(INTAKE_SPEED);
	}
	private void handleIdleStopState() {
		spinnerMotor.set(0);
	}
	private void handleReleaseState() {
		spinnerMotor.set(RELEASE_SPEED);
	}
}
