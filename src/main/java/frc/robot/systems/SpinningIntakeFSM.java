package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
	// Distance definitions
	public enum ItemType {
		CUBE,
		CONE,
		EMPTY
	}
	//FIX VALUES
	private static final double INTAKE_SPEED = 0.1;
	private static final double RELEASE_SPEED = -0.1;
	//arbitrary constants for cube and cone
	//6 inches
	private static final int MIN_CONE_DISTANCE = 1240;
	//8 inches
	private static final int MIN_CUBE_DISTANCE = 970;
	//8.5 inches
	private static final int MAX_COLOR_MEASURE = 915;
	//9 inches
	private static final int MIN_COLOR_MEASURE = 860;
	//variable for armFSM, 0 means no object, 1 means cone, 2 means cube
	private static ItemType itemType = ItemType.EMPTY;

	//CUBE RGB THRESHOLD VALUES
	private static final double BLUE_THRESHOLD = 0.175;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax spinnerMotor;
	//private DigitalInput limitSwitchCone;
	private AnalogInput distanceSensorObject;
	private ColorSensorV3 colorSensor;

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
		distanceSensorObject = new AnalogInput(HardwareMap.ANALOGIO_ID_DISTANCE_SENSOR);
		colorSensor = new ColorSensorV3(Port.kOnboard);

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
		//System.out.println(itemType);
		SmartDashboard.putNumber("distance", distanceSensorObject.getValue());
		SmartDashboard.putNumber("r", colorSensor.getColor().red);
		SmartDashboard.putNumber("g", colorSensor.getColor().green);
		SmartDashboard.putNumber("b", colorSensor.getColor().blue);
		SmartDashboard.putString("item type", itemType.toString());
		//System.out.println(distanceSensorObject.getValue() + " " + itemType);
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
		FSMState previousState = currentState;
		currentState = nextState(input);
		if (previousState != currentState) {
			System.out.println(currentState);
		}
	}

	/*-------------------------NON HANDLER METHODS ------------------------- */
	/**
	 * Returns the type of object currently in the grabber.
	 * @return int 0 1 or 2 for nothing, cone, cube
	 */
	public static ItemType getObjectType() {
		return itemType;
	}
	private void updateItem() {
		double b =  colorSensor.getColor().blue;

		//System.out.println(r + " " + g + " " + b + " " + colorSensorCube.getProximity());
		if (b > BLUE_THRESHOLD) {
			itemType = ItemType.CUBE;
		} else {
			itemType = ItemType.CONE;
		}
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
				if (input.isReleaseButtonPressed()) {
					return FSMState.RELEASE;
				}
				if ((itemType == ItemType.CUBE && distanceSensorObject.getValue()
					> MIN_CUBE_DISTANCE) || distanceSensorObject.getValue() > MIN_CONE_DISTANCE) {
					return FSMState.IDLE_STOP;
				}
				return FSMState.IDLE_SPINNING;
			case IDLE_STOP:
				if (input.isReleaseButtonPressed()) {
					return FSMState.RELEASE;
				}
				return FSMState.IDLE_STOP;
			case RELEASE:
				if (!input.isReleaseButtonPressed()) {
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
		if (distanceSensorObject.getValue() < MAX_COLOR_MEASURE
			&& distanceSensorObject.getValue() > MIN_COLOR_MEASURE) {
			updateItem();
		}
		spinnerMotor.set(INTAKE_SPEED);
	}
	private void handleIdleStopState() {
		spinnerMotor.set(0);
	}
	private void handleReleaseState() {
		itemType = ItemType.EMPTY;
		spinnerMotor.set(RELEASE_SPEED);
	}
}
