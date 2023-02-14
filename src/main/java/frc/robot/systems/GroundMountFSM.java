package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.SparkMaxPIDController;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class GroundMountFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		START_STATE,
		LOWER_STATE,
		DONE
	}
	//arbitrary constants, must test all of these
	private static final double LOWER_ANGLE_ENCODER_FORWARD_ROTATIONS = 30;
	private static final double PIVOT_ERROR_GROUND_MOUNT = 0.3;
	private static final double PID_MAX_POWER = 0.2;
	private static final double PID_CONSTANT_P = 0.00022f;
	private static final double PID_CONSTANT_I = 0.000055f;
	private static final double PID_CONSTANT_D = 0.000008f;
	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax pivotArmMotor;
	private SparkMaxPIDController pidControllerPivotArm;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public GroundMountFSM() {
		// Perform hardware init
		if (HardwareMap.isTestBoardGroundMount()) {
			pivotArmMotor = new CANSparkMax(HardwareMap.CAN_ID_TEST_GROUND_MOUNT,
										CANSparkMax.MotorType.kBrushless);
		} else {
			pivotArmMotor = new CANSparkMax(HardwareMap.CAN_ID_GROUND_MOUNT,
										CANSparkMax.MotorType.kBrushless);
		}
		pidControllerPivotArm = pivotArmMotor.getPIDController();
		pidControllerPivotArm.setP(PID_CONSTANT_P);
		pidControllerPivotArm.setI(PID_CONSTANT_I);
		pidControllerPivotArm.setD(PID_CONSTANT_D);
		pidControllerPivotArm.setIZone(0);
		pidControllerPivotArm.setFF(0);
		pidControllerPivotArm.setOutputRange(-PID_MAX_POWER, PID_MAX_POWER);
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
		pivotArmMotor.getEncoder().setPosition(0);
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
		SmartDashboard.putNumber("encoder", pivotArmMotor.getEncoder().getPosition());
		SmartDashboard.putString("state", currentState.toString());
		SmartDashboard.putNumber("power", pivotArmMotor.get());
		//System.out.println(distanceSensorObject.getValue() + " " + itemType);
		if (input == null) {
			return;
		}
		switch (currentState) {
			case START_STATE:
				handleStartState();
				break;
			case LOWER_STATE:
				handleLowerState();
				break;
			case DONE:
				handleDoneState();
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
				if (input.isLowerButtonPressed()) {
					return FSMState.LOWER_STATE;
				}
				return FSMState.START_STATE;
			case LOWER_STATE:
				if (withinError(pivotArmMotor.getEncoder().getPosition(),
					LOWER_ANGLE_ENCODER_FORWARD_ROTATIONS)) {
					return FSMState.DONE;
				}
				return FSMState.LOWER_STATE;
			case DONE:
				return FSMState.DONE;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}
	private boolean withinError(double a, double b) {
		return Math.abs(a - b) < PIVOT_ERROR_GROUND_MOUNT;
	}
	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in states.
	 */
	private void handleStartState() {
	}
	private void handleLowerState() {
		pidControllerPivotArm.setReference(LOWER_ANGLE_ENCODER_FORWARD_ROTATIONS,
							CANSparkMax.ControlType.kPosition);
		//pivotArmMotor.set(0.1);
	}
	private void handleDoneState() {
		pivotArmMotor.set(0);
	}
}
