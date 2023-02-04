package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C.Port;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class DistColorTester {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		TEST
	}
	//FIX VALUES
	private static final double INTAKE_SPEED = 0.1;
	private static final double RELEASE_SPEED = -0.1;
	//arbitrary constants for cube and cone
	private static final int MIN_CONE_DISTANCE = 2300;
	private static final int MIN_CUBE_DISTANCE = 1300;
	private static final int MAX_COLOR_MEASURE = 800;
	private static final int MIN_COLOR_MEASURE = 600;
	//variable for armFSM, 0 means no object, 1 means cone, 2 means cube
	private static int itemType = 0;

	//CUBE RGB THRESHOLD VALUES
	private static final double RED_THRESHOLD = 0.33f;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax spinnerMotor;
	//private DigitalInput limitSwitchCone;
	private AnalogInput distanceSensorObject;
	private ColorSensorV3 colorSensorCube;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public DistColorTester() {
		// Perform hardware init
		spinnerMotor = new CANSparkMax(HardwareMap.CAN_ID_SPINNER_MOTOR,
										CANSparkMax.MotorType.kBrushless);
		distanceSensorObject = new AnalogInput(HardwareMap.ANALOGIO_ID_DISTANCE_SENSOR);
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
		currentState = FSMState.TEST;
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
		//System.out.println(distanceSensorObject.getValue() + " " + itemType);
		if (input == null) {
			return;
		}
		switch (currentState) {
			case TEST:
				handleTestState();
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
	public static int getObjectType() {
		return itemType;
	}
	private boolean updateItem() {
		double r =  colorSensorCube.getColor().red;

		//System.out.println(r + " " + g + " " + b + " " + colorSensorCube.getProximity());
		if (r > RED_THRESHOLD)
			itemType = 2;
		else
			itemType = 1;
		return false;
		//return !isCone && objectDetected;
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
		return FSMState.TEST;
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in states.
	 */
	private void handleTestState() {
		//System.out.println(colorSensorCube.getColor().red + " "
		//	+ colorSensorCube.getColor().green + " " + colorSensorCube.getColor().blue);
		System.out.println(itemType);
		updateItem();
		//System.out.println(itemType);
		//spinnerMotor.set(INTAKE_SPEED);
	}
}
