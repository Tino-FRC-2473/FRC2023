
package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.wpilibj.Timer;
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
	public enum GroundMountFSMState {
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
	private static final double PIVOT_UP_POWER = -0.1;
	private static final double MIN_POWER = -0.2;
	private static final double MAX_POWER = 0.1;
	private boolean zeroed = false;
	private static final double BOTTOM_ENCODER_LIMIT = 50.00; //ARBITRARY VALUE
	private static final double HOME_ENCODER_CONSTANT = -4;
	private static final double P_CONSTANT = 0.006;
	private static final double P_UP_CONSTANT = 0.009;
	private static final double ERROR = 5;

	/* ======================== Private variables ======================== */
	private GroundMountFSMState currentState;

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
		pivotArmMotor.setInverted(false);
		limitSwitchHigh = pivotArmMotor.getReverseLimitSwitch(
								SparkMaxLimitSwitch.Type.kNormallyClosed);
		limitSwitchHigh.enableLimitSwitch(true);
		limitSwitchLow = pivotArmMotor.getForwardLimitSwitch(
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
	public GroundMountFSMState getCurrentState() {
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
		zeroed = false;
		pivotArmMotor.getEncoder().setPosition(0);
		currentState = GroundMountFSMState.START_STATE;
		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}
	private boolean withinError(double a, double b) {
		return Math.abs(a - b) < ERROR;
	}
	/**
	 * updateAutonomous function.
	 * @param state that the autonomous is in.
	 * @return a boolean
	 */
	public boolean updateAutonomous(GroundMountFSMState state) {
		SmartDashboard.putNumber("power", pivotArmMotor.get());
		SmartDashboard.putBoolean("limit switch low", isLimitSwitchLowPressed());
		SmartDashboard.putBoolean("limit switch high", isLimitSwitchHighPressed());
		SmartDashboard.putString("state", state.toString());
		switch (state) {
			case AUTONOMOUS_UP:
				handleAutonomousUpState();
				return withinError(pivotArmMotor.getEncoder().getPosition(), 0);
			case AUTONOMOUS_DOWN:
				handleAutonomousDownState();
				return withinError(pivotArmMotor.getEncoder().getPosition(), BOTTOM_ENCODER_LIMIT)
					|| limitSwitchLow.isPressed();
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
		System.out.println("start time ground mount: " + Timer.getFPGATimestamp());
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
		System.out.println("end time ground mount: " + Timer.getFPGATimestamp());
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
	private GroundMountFSMState nextState(TeleopInput input) {
		if (input == null) {
			return GroundMountFSMState.START_STATE;
		}

		switch (currentState) {
			case START_STATE:
				if (zeroed) {
					return GroundMountFSMState.PIVOTED_UP;
				} else {
					return GroundMountFSMState.START_STATE;
				}
			case PIVOTING_UP:
				if (input.isPivotButtonPressed()) {
					return GroundMountFSMState.PIVOTING_DOWN;
				}
				if (isLimitSwitchHighPressed()) {
					return GroundMountFSMState.PIVOTED_UP;
				}
				//means pivot button is not pressed and limit switch not activated, stay in state
				return GroundMountFSMState.PIVOTING_UP;
			case PIVOTED_UP:
				if (input.isPivotButtonPressed()) {
					return GroundMountFSMState.PIVOTING_DOWN;
				}
				if (!input.isPivotButtonPressed() && !isLimitSwitchHighPressed()) {
					return GroundMountFSMState.PIVOTING_UP;
				}
				//means pivot button not pressed and limit switch activated, stay in state
				return GroundMountFSMState.PIVOTED_UP;
			case PIVOTING_DOWN:
				if (!input.isPivotButtonPressed()) {
					return GroundMountFSMState.PIVOTING_UP;
				}
				if (isLimitSwitchLowPressed() && input.isPivotButtonPressed()) {
					return GroundMountFSMState.PIVOTED_DOWN;
				}
				//means limit switch low not activated and pivot button still pressed, stay in state
				return GroundMountFSMState.PIVOTING_DOWN;
			case PIVOTED_DOWN:
				if (!input.isPivotButtonPressed()) {
					return GroundMountFSMState.PIVOTING_UP;
				}
				if (input.isPivotButtonPressed() && !isLimitSwitchLowPressed()) {
					return GroundMountFSMState.PIVOTING_DOWN;
				}
				//means pivot button is pressed and limit switch low active, stay in state
				return GroundMountFSMState.PIVOTED_DOWN;
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
	private double capMotorPower(double a) {
		if (a > MAX_POWER) {
			return MAX_POWER;
		} else if (a < MIN_POWER) {
			return MIN_POWER;
		}
		return a;
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in states.
	 */
	private void handleStartState() {
		pivotArmMotor.set(PIVOT_UP_POWER);
		if (limitSwitchHigh.isPressed()) {
			zeroed = true;
			pivotArmMotor.getEncoder().setPosition(HOME_ENCODER_CONSTANT);
		}
	}
	private void handlePivotedUpState() {
		pivotArmMotor.set(capMotorPower(-pivotArmMotor.getEncoder().getPosition() * P_UP_CONSTANT));
		System.out.println(-pivotArmMotor.getEncoder().getPosition() * P_UP_CONSTANT);
	}
	private void handlePivotingUpState() {
		pivotArmMotor.set(capMotorPower(-pivotArmMotor.getEncoder().getPosition() * P_UP_CONSTANT));
		System.out.println(-pivotArmMotor.getEncoder().getPosition() * P_UP_CONSTANT);
	}
	private void handlePivotedDownState() {
		pivotArmMotor.set(capMotorPower((BOTTOM_ENCODER_LIMIT
			- pivotArmMotor.getEncoder().getPosition()) * P_CONSTANT));
	}
	private void handlePivotingDownState() {
		pivotArmMotor.set(capMotorPower((BOTTOM_ENCODER_LIMIT
			- pivotArmMotor.getEncoder().getPosition()) * P_CONSTANT));
	}

	/* AUTONOMOUS HANDLES */

	private void handleAutonomousDownState() {
		pivotArmMotor.set(capMotorPower((BOTTOM_ENCODER_LIMIT
			- pivotArmMotor.getEncoder().getPosition()) * P_CONSTANT));
	}

	private void handleAutonomousUpState() {
		pivotArmMotor.set(capMotorPower(-pivotArmMotor.getEncoder().getPosition() * P_UP_CONSTANT));
	}

	private void handleAutonomousIdleState() {
		pivotArmMotor.set(0);
	}
}
