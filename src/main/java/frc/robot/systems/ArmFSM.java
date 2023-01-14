package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class ArmFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		IDLE,
		ARM_MOVEMENT, 
	}

	private static final float TELEARM_MOTOR = 0.3f;
	private static final float PIVOT_MOTOR = 0.3f;
	

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	private CANSparkMax pivotMotor;
	private CANSparkMax teleArmMotor;

	/*
	 * Hardware Map each of the motors
	 * 
	 */
	public ArmFSM() {
		// Perform hardware init
		pivotMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_PIVOT,
										CANSparkMax.MotorType.kBrushless);

		teleArmMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_TELEARM,
										CANSparkMax.MotorType.kBrushless);
		// Reset state machine
		reset();
	}

	/*
	 * Get the current state
	 * 
	 * @return current state
	 */
	public FSMState getCurrentState() {
		return currentState;
	}

	/*
	 * On robot start set the start to IDLE state
	 */
	public void reset() {
		currentState = FSMState.IDLE;

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/*
	 * what happens in each state
	 */
	public void update(TeleopInput input) {
		switch (currentState) {
			case IDLE:
				handleIdleState(input);
				break;
			case ARM_MOVEMENT:
				handleArmMechState(input);

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		currentState = nextState(input);
	}

	/*
	 * When inputs are pressed, the states will change likewise
	 * 
	 * @return expected state
	 */
	private FSMState nextState(TeleopInput input) {
		switch (currentState) {
			case IDLE:
				if(input != null){
					return FSMState.ARM_MOVEMENT;
				}
				return FSMState.IDLE;
			case ARM_MOVEMENT:
				if(input == null){
					return FSMState.IDLE;
				}
				return FSMState.ARM_MOVEMENT;


			default :
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/*
	 * What to do when in the IDLE state
	 */
	private void handleIdleState(TeleopInput input) {
		teleArmMotor.set(0);
		pivotMotor.set(0);
	}


	/*
	 * What to do when in the ARM_MOVEMENT state
	 */
	private void handleArmMechState(TeleopInput input) {
		if(input.isPivotIncreaseButtonPressed()){
			pivotMotor.set(PIVOT_MOTOR);
		}else if(input.isPivotDecreaseButtonPressed()){
			pivotMotor.set(-PIVOT_MOTOR);
		}

		if(input.isExtendButtonPressed()){
			teleArmMotor.set(TELEARM_MOTOR);
		}else if(input.isRetractButtonPressed()){
			pivotMotor.set(-TELEARM_MOTOR);
		}
	}

}
