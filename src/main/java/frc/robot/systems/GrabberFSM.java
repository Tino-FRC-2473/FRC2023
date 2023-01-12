package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
import edu.wpi.first.wpilibj.Servo;

public class GrabberFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		START_STATE,
		OPENING,
        CLOSING_CONE,
        CLOSING_CUBE,
		DONE
	}
	//FIX VALUES
	private static final double MOTOR_RUN_POWER = 0.1f;
	private static final double CONE_ENCODER_DISTANCE = -1;
	private static final double CUBE_MIN_ENCODER_DISTANCE = -1;
	private static final double CUBE_MAX_ENCODER_DISTANCE = -1;
	private static final double OPEN_ENCODER_DISTANCE = -1;
	private static final double FLIP_ANGLE = -1;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax GrabberMotor;
    private Servo ExtractionServo;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public GrabberFSM() {
		// Perform hardware init
		GrabberMotor = new CANSparkMax(HardwareMap.CAN_ID_GRABBER_MOTOR,
										CANSparkMax.MotorType.kBrushless);
		ExtractionServo = new Servo(0);
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
            case OPENING:
				handleOpeningState(input);
				break;
            case CLOSING_CONE:
				handleClosingConeState(input);
				break;
            case CLOSING_CUBE:
				handleClosingCubeState(input);
				break;
			case DONE:
				handleDoneState(input);
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
		switch (currentState) {
			case START_STATE:
				return FSMState.OPENING;
			case OPENING:
				if (input.getCubeButton()) {
					return FSMState.CLOSING_CUBE;
				}
				if (input.getConeButton()) {
					return FSMState.CLOSING_CONE;
				}
				if (GrabberMotor.getEncoder().getPosition() > OPEN_ENCODER_DISTANCE) {
					return FSMState.DONE;
				}
				return FSMState.OPENING;
			case CLOSING_CONE:
				if (input.getReleaseButton()) {
					return FSMState.OPENING;
				}
				if (input.getCubeButton()) {
					return FSMState.CLOSING_CUBE;
				}
				if (GrabberMotor.getEncoder().getPosition() < CONE_ENCODER_DISTANCE) {
					return FSMState.DONE;
				}
				return FSMState.CLOSING_CONE;
			case CLOSING_CUBE:
				if (input.getReleaseButton()) {
					return FSMState.OPENING;
				}
				if (input.getConeButton()) {
					return FSMState.CLOSING_CONE;
				}
				if (GrabberMotor.getEncoder().getPosition() < CUBE_MAX_ENCODER_DISTANCE &&
					GrabberMotor.getEncoder().getPosition() > CUBE_MIN_ENCODER_DISTANCE) {
					return FSMState.DONE;
				}
				return FSMState.CLOSING_CUBE;
			case DONE:
				if (input.getReleaseButton()) {
					return FSMState.OPENING;
				}
				if (input.getConeButton()) {
					return FSMState.CLOSING_CONE;
				}
				if (input.getCubeButton()) {
					return FSMState.CLOSING_CUBE;
				}
				return FSMState.DONE;
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
	private void handleOpeningState(TeleopInput input) {
		GrabberMotor.set(MOTOR_RUN_POWER);
	}
	private void handleClosingConeState(TeleopInput input) {
		GrabberMotor.set(-MOTOR_RUN_POWER);
	}
	private void handleClosingCubeState(TeleopInput input) {
		double encoderValue = GrabberMotor.getEncoder().getPosition();
		if (encoderValue < CUBE_MAX_ENCODER_DISTANCE)
			GrabberMotor.set(MOTOR_RUN_POWER);
		else
			GrabberMotor.set(-MOTOR_RUN_POWER);
	}
	private void handleDoneState(TeleopInput input) {

	}
}