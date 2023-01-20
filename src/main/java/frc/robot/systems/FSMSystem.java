// package frc.robot.systems;

// // WPILib Imports

// // Third party Hardware Imports
// import com.revrobotics.CANSparkMax;

// // Robot Imports
// import frc.robot.TeleopInput;
// import frc.robot.HardwareMap;

// /** FSMSystem class manages execution flow of different states.*/
// public class FSMSystem {
// 	/** Contains different states. */
// 	public enum FSMState {
// 		/** Start state. */
// 		START_STATE,
// 		/** Other state. */
// 		DRIVE_STATE
// 	}

// 	/** motor power. */
// 	private static final float MOTOR_RUN_POWER = 0.1f;

// 	/** current state. */
// 	private FSMState currentState;

// 	// Hardware devices should be owned by one and only
// 	//  one system. They must be private to their owner
// 	//  system and may not be used elsewhere.

// 	/** motor. */
// 	private CANSparkMax exampleMotor;

// 	/* ======================== Constructor ======================== */
// 	/**
// 	 * Create FSMSystem and initialize to starting state. Also perform any
// 	 * one-time initialization or configuration of hardware required. Note
// 	 * the constructor is called only once when the robot boots.
// 	 */
// 	public FSMSystem() {
// 		// Perform hardware init
// 		exampleMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_SHOOTER,
// 									CANSparkMax.MotorType.kBrushless);

// 		// Reset state machine
// 		reset();
// 	}

// 	/* ======================== Public methods ======================== */
// 	/**
// 	 * Return current FSM state.
// 	 * @return Current FSM state
// 	 */
// 	public FSMState getCurrentState() {
// 		return currentState;
// 	}

// 	/**
// 	 * Reset this system to its start state. This may be called from mode init
// 	 * when the robot is enabled.
// 	 *
// 	 * Note this is distinct from the one-time initialization in the constructor
// 	 * as it may be called multiple times in a boot cycle,
// 	 * Ex. if the robot is enabled, disabled, then reenabled.
// 	 */
// 	public void reset() {
// 		currentState = FSMState.DRIVE_STATE;

// 		// Call one tick of update to ensure outputs reflect start state
// 		update(null);
// 	}

// 	/**
// 	 * Update FSM based on new inputs. This function only calls the FSM state
// 	 * specific handlers.
// 	 * @param input Global TeleopInput if robot in teleop mode or null if
// 	 *        the robot is in autonomous mode.
// 	 */
// 	public void update(final TeleopInput input) {
// 		switch (currentState) {
// 			case START_STATE:
// 				handleStartState(input);
// 				break;
// 			case DRIVE_STATE:
// 				handleDriveState(input);
// 				break;
// 			default:
// 				throw new IllegalStateException(
// 							"Invalid state: " + currentState.toString());
// 		}
// 		currentState = nextState(input);
// 	}

// 	/* ======================== Private methods ======================== */
// 	/**
// 	 * Decide the next state to transition to. This is a function of the inputs
// 	 * and the current state of this FSM. This method should not have any side
// 	 * effects on outputs. In other words, this method should only read or get
// 	 * values to decide what state to go to.
// 	 * @param input Global TeleopInput if robot in teleop mode or null if
// 	 *              the robot is in autonomous mode.
// 	 * @return FSM state for the next iteration
// 	 */
// 	private FSMState nextState(final TeleopInput input) {
// 		switch (currentState) {
// 			case START_STATE:
// 				if (input != null) {
// 					return FSMState.OTHER_STATE;
// 				} else {
// 					return FSMState.START_STATE;
// 				}
// 			case OTHER_STATE:
// 				return FSMState.OTHER_STATE;
// 			default:
// 				throw new IllegalStateException(
// 								"Invalid state: " + currentState.toString());
// 		}
// 	}

// 	/* ------------------------ FSM state handlers ------------------------ */
// 	/**
// 	 * Handle behavior in START_STATE.
// 	 * @param input Global TeleopInput if robot in teleop mode or null if
// 	 *              the robot is in autonomous mode.
// 	 */
// 	private void handleStartState(final TeleopInput input) {
// 		exampleMotor.set(0);
// 	}

// 	/**
// 	 * Handle behavior in OTHER_STATE.
// 	 * @param input Global TeleopInput if robot in teleop mode or null if
// 	 *              the robot is in autonomous mode.
// 	 */
// 	private void handleDriveState(final TeleopInput input) {
// 		exampleMotor.set(MOTOR_RUN_POWER);
// 	}
// }
