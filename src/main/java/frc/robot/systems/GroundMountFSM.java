package frc.robot.systems;

// WPILib Imports
// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

public class GroundMountFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum GroundMountFSMState {
		START_STATE,
		PIVOTING_UP,
		PIVOTING_DOWN,
		PIVOTING_MID,
		AUTONOMOUS_UP,
		AUTONOMOUS_DOWN,
		AUTONOMOUS_IDLE,
		AUTONOMOUS_MID
	}
	//arbitrary constants, must test all of these
	private static final double PIVOT_UP_POWER = -0.2;
	private static final double MIN_POWER = -0.3;
	private static final double MAX_POWER = 0.25;
	private boolean zeroed = false;
	private static final double BOTTOM_ENCODER_LIMIT = 55.00; //ARBITRARY VALUE
	private static final double MID_ENCODER = 25;
	private static final double PID_CONSTANT_PIVOT_P = 0.012;
	private static final double PID_CONSTANT_PIVOT_I = 0;
	private static final double PID_CONSTANT_PIVOT_D = 0;
	private static final double ERROR = 10;
	private static final double MAX_ACCEL = 0.01;
	private static final double MAX_DECEL = 0.10;
	private static final double PICKUP_ENCODER = 50;
	private static final double OVERRUN_THRESHOLD = 0.02;
	private static final double PIVOT_MID_ACCEL_THRESHOLD = 5;
	private SparkMaxPIDController pidControllerPivot;

	/* ======================== Private variables ======================== */
	private GroundMountFSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax pivotArmMotor;
	private SparkMaxLimitSwitch limitSwitchHigh;
	private SparkMaxLimitSwitch limitSwitchLow;
	private double lastPower;


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
		pidControllerPivot = pivotArmMotor.getPIDController();
		pidControllerPivot.setP(PID_CONSTANT_PIVOT_P);
		pidControllerPivot.setI(PID_CONSTANT_PIVOT_I);
		pidControllerPivot.setD(PID_CONSTANT_PIVOT_D);
		pidControllerPivot.setIZone(0);
		pidControllerPivot.setFF(0);
		pidControllerPivot.setOutputRange(MIN_POWER, MAX_POWER);
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
				return withinError(pivotArmMotor.getEncoder().getPosition(), 0)
					|| limitSwitchHigh.isPressed();
			case AUTONOMOUS_DOWN:
				handleAutonomousDownState();
				return withinError(pivotArmMotor.getEncoder().getPosition(), BOTTOM_ENCODER_LIMIT)
					|| limitSwitchLow.isPressed();
			case AUTONOMOUS_MID:
				handleAutonomousMidState();
				return withinError(pivotArmMotor.getEncoder().getPosition(), MID_ENCODER);
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
		//System.out.println("power: " + pivotArmMotor.get());
		double begin = Timer.getFPGATimestamp();
		//System.out.println(itemType);
		SmartDashboard.putNumber("encoder", pivotArmMotor.getEncoder().getPosition());
		SmartDashboard.putString("state", currentState.toString());
		SmartDashboard.putNumber("power", pivotArmMotor.get());
		SmartDashboard.putBoolean("limit low", isLimitSwitchLowPressed());
		SmartDashboard.putBoolean("limit high", isLimitSwitchHighPressed());
		if (input == null) {
			return;
		}
		switch (currentState) {
			case START_STATE:
				handleStartState();
				break;
			case PIVOTING_DOWN:
				handlePivotingDownState();
				break;
			case PIVOTING_UP:
				handlePivotingUpState();
				break;
			case PIVOTING_MID:
				handlePivotingMidState();
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		currentState = nextState(input);
		double tt = (Timer.getFPGATimestamp() - begin);
		//System.out.println("ground mount time taken: " + );
		if (tt > OVERRUN_THRESHOLD) {
			System.out.println("ALERT ALERT GROUND MOUNT " +  tt);
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
	private GroundMountFSMState nextState(TeleopInput input) {
		if (input == null) {
			return GroundMountFSMState.START_STATE;
		}

		switch (currentState) {
			case START_STATE:
				if (zeroed) {
					return GroundMountFSMState.PIVOTING_UP;
				} else {
					return GroundMountFSMState.START_STATE;
				}
			case PIVOTING_UP:
				if (input.isPivotButtonPressed()) {
					return GroundMountFSMState.PIVOTING_DOWN;
				}
				if (input.isGroundMountShootButtonPressed()) {
					return GroundMountFSMState.PIVOTING_MID;
				}
				//means pivot button is not pressed and limit switch not activated, stay in state
				return GroundMountFSMState.PIVOTING_UP;
			case PIVOTING_DOWN:
				if (!input.isPivotButtonPressed()
					&& !input.isGroundMountShootButtonPressed()) {
					return GroundMountFSMState.PIVOTING_UP;
				}
				if (input.isGroundMountShootButtonPressed()) {
					return GroundMountFSMState.PIVOTING_MID;
				}
				//means limit switch low not activated and pivot button still pressed, stay in state
				return GroundMountFSMState.PIVOTING_DOWN;
			case PIVOTING_MID:
				if (!input.isPivotButtonPressed()
					&& !input.isGroundMountShootButtonPressed()) {
					return GroundMountFSMState.PIVOTING_UP;
				}
				if (input.isPivotButtonPressed()) {
					return GroundMountFSMState.PIVOTING_DOWN;
				}
				//means limit switch low not activated and pivot button still pressed, stay in state
				return GroundMountFSMState.PIVOTING_MID;
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
	private double changePower(double target) {
		if (target > 0 && target < lastPower - MAX_DECEL) {
			return lastPower - MAX_DECEL;
		}
		if (target < 0 && target > lastPower + MAX_DECEL) {
			return lastPower + MAX_DECEL;
		}
		if (target > lastPower + MAX_ACCEL) {
			return lastPower + MAX_ACCEL;
		}
		if (target < lastPower - MAX_ACCEL) {
			return lastPower - MAX_ACCEL;
		}
		return target;
	}
	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in states.
	 */
	private void handleStartState() {
		pivotArmMotor.set(PIVOT_UP_POWER);
		if (limitSwitchHigh.isPressed()) {
			zeroed = true;
			pivotArmMotor.getEncoder().setPosition(0);
		}
	}
	private void handlePivotingUpState() {
		// if (lastPower > 0)
		// 	pivotArmMotor.set(lastPower-MAX_ACCEL);
		// else
		pidControllerPivot.setReference(0, CANSparkMax.ControlType.kPosition);
		/*if (limitSwitchHigh.isPressed()) {
			pivotArmMotor.getEncoder().setPosition(0);
		} else if (withinError(0, pivotArmMotor.getEncoder().getPosition())
			&& !limitSwitchHigh.isPressed()) {
			pivotArmMotor.set(PIVOT_UP_POWER);
		} else {
			lastPower = capMotorPower(changePower(
				-pivotArmMotor.getEncoder().getPosition() * P_UP_CONSTANT));
			pivotArmMotor.set(lastPower);
		}*/
		//System.out.println(-pivotArmMotor.getEncoder().getPosition() * P_UP_CONSTANT);
	}
	private void handlePivotingMidState() {
		/*double newMidEncoder = MID_ENCODER;
		if (pivotArmMotor.getEncoder().getPosition() > MID_ENCODER + PIVOT_MID_ACCEL_THRESHOLD) {
			newMidEncoder -= PIVOT_MID_ACCEL_THRESHOLD;
		}
		if (pivotArmMotor.getEncoder().getPosition() < MID_ENCODER - PIVOT_MID_ACCEL_THRESHOLD) {
			newMidEncoder += PIVOT_MID_ACCEL_THRESHOLD;
		}
		lastPower = capMotorPower(changePower((newMidEncoder
			- pivotArmMotor.getEncoder().getPosition()) * P_CONSTANT));
		pivotArmMotor.set(lastPower);*/
		pidControllerPivot.setReference(MID_ENCODER, CANSparkMax.ControlType.kPosition);
	}
	private void handlePivotingDownState() {
		/*double targetEncoder = BOTTOM_ENCODER_LIMIT;
		if (pivotArmMotor.getEncoder().getPosition() > PICKUP_ENCODER) {
			targetEncoder = PICKUP_ENCODER;
		}
		lastPower = capMotorPower(changePower((targetEncoder
			- pivotArmMotor.getEncoder().getPosition()) * P_CONSTANT));
		pivotArmMotor.set(lastPower);*/
		pidControllerPivot.setReference(BOTTOM_ENCODER_LIMIT, CANSparkMax.ControlType.kPosition);
	}

	/* AUTONOMOUS HANDLES */

	private void handleAutonomousDownState() {
		/*double targetEncoder = BOTTOM_ENCODER_LIMIT;
		if (pivotArmMotor.getEncoder().getPosition() > PICKUP_ENCODER) {
			targetEncoder = PICKUP_ENCODER;
		}
		lastPower = capMotorPower(changePower((targetEncoder
			- pivotArmMotor.getEncoder().getPosition()) * P_CONSTANT));
		pivotArmMotor.set(lastPower);*/
	}

	private void handleAutonomousUpState() {
		/*if (limitSwitchHigh.isPressed()) {
			pivotArmMotor.getEncoder().setPosition(0);
		} else if (withinError(0, pivotArmMotor.getEncoder().getPosition())
			&& !limitSwitchHigh.isPressed()) {
			pivotArmMotor.set(PIVOT_UP_POWER);
		} else {
			lastPower = capMotorPower(changePower(
				-pivotArmMotor.getEncoder().getPosition() * P_UP_CONSTANT));
			pivotArmMotor.set(lastPower);
		}*/
	}

	private void handleAutonomousMidState() {
		/*double newMidEncoder = MID_ENCODER;
		if (pivotArmMotor.getEncoder().getPosition() > MID_ENCODER + PIVOT_MID_ACCEL_THRESHOLD) {
			newMidEncoder -= PIVOT_MID_ACCEL_THRESHOLD;
		}
		if (pivotArmMotor.getEncoder().getPosition() < MID_ENCODER - PIVOT_MID_ACCEL_THRESHOLD) {
			newMidEncoder += PIVOT_MID_ACCEL_THRESHOLD;
		}
		lastPower = capMotorPower(changePower((newMidEncoder
			- pivotArmMotor.getEncoder().getPosition()) * P_CONSTANT));
		pivotArmMotor.set(lastPower);*/
	}

	private void handleAutonomousIdleState() {
		pivotArmMotor.set(0);
	}
}
