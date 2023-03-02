package frc.robot.systems;


// WPILib Imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class ArmFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		UNHOMED_STATE,
		IDLE,
		AUTONOMOUS_RETRACT,
		HOMING_STATE,
		MOVING_TO_START_STATE,
		ARM_MOVEMENT,
		SHOOT_HIGH_FORWARD,
		SHOOT_HIGH_BACKWARD,
		SHOOT_MID_FORWARD,
		SHOOT_MID_BACKWARD,
		SHOOT_LOW_FORWARD,
		SUBSTATION_PICKUP_FORWARD,
		SUBSTATION_PICKUP_BACKWARD
	}


	//starts at 71 inches to 33 inches
	//encoder over angle
	private static final double ENCODER_TICKS_TO_ARM_ANGLE_DEGREES_CONSTANT = -320.641 / 171;
	private static final double ENCODER_TICKS_TO_ARM_LENGTH_INCHES_CONSTANT = 530 / 39;
	private static final float TELEARM_MOTOR_POWER = 0.4f;
	private static final float TELEARM_MOTOR_POWER_FINE_TUNING = 0.05f;
	private static final float PIVOT_MOTOR_POWER = 0.1f;
	private static final float PIVOT_MOTOR_SLOW_DOWN_POWER = 0.05f;
	private static final float PIVOT_MOTOR_POWER_FINE_TUNING = 0.05f;

	//20 inches
	private static final double ARM_ENCODER_MAX_LENGTH_ROTATIONS = 20
		* ENCODER_TICKS_TO_ARM_LENGTH_INCHES_CONSTANT;

	//19 inches
	private static final double ARM_ENCODER_HIGH_FORWARD_CUBE_ROTATIONS = 19
		* ENCODER_TICKS_TO_ARM_LENGTH_INCHES_CONSTANT;
	//11 inches
	private static final double ARM_ENCODER_HIGH_FORWARD_CONE_ROTATIONS = 11
		* ENCODER_TICKS_TO_ARM_LENGTH_INCHES_CONSTANT;
	//12 inches
	private static final double ARM_ENCODER_HIGH_BACKWARD_ROTATIONS = 12
		* ENCODER_TICKS_TO_ARM_LENGTH_INCHES_CONSTANT;
	//3 inches
	private static final double ARM_ENCODER_MID_FORWARD_ROTATIONS = 3
		* ENCODER_TICKS_TO_ARM_LENGTH_INCHES_CONSTANT;
	//4 inches
	private static final double ARM_ENCODER_MID_BACKWARD_ROTATIONS = 4
		* ENCODER_TICKS_TO_ARM_LENGTH_INCHES_CONSTANT;
	//0 inches(retracted)
	private static final double ARM_ENCODER_SUBSTATION_FORWARD_ROTATIONS = 0
		* ENCODER_TICKS_TO_ARM_LENGTH_INCHES_CONSTANT;
	//3 inches
	private static final double ARM_ENCODER_SUBSTATION_BACKWARD_ROTATIONS = 3
		* ENCODER_TICKS_TO_ARM_LENGTH_INCHES_CONSTANT;
	//0 inches(retracted)
	private static final double ARM_ENCODER_LOW_ROTATIONS = 0
		* ENCODER_TICKS_TO_ARM_LENGTH_INCHES_CONSTANT;

	//All encoder measures are from minimum limit switch
	//Angle measure in comments are assuming 0 is horizon not min limit switch
	private static final double ENCODER_TICKS_SLOW_DOWN_RANGE_MAX =
		ENCODER_TICKS_TO_ARM_ANGLE_DEGREES_CONSTANT * (75 + 13);
	private static final double ENCODER_TICKS_SLOW_DOWN_RANGE_MIN =
		ENCODER_TICKS_TO_ARM_ANGLE_DEGREES_CONSTANT * (105 + 13);
	//90 degrees
	private static final double ARM_ENCODER_VERTICAL_ANGLE_ROTATIONS = (90 + 13)
		* ENCODER_TICKS_TO_ARM_ANGLE_DEGREES_CONSTANT;
	//111.8 degrees
	private static final double ARM_ENCODER_STARTING_ANGLE_ROTATIONS = (111.8 + 13)
		* ENCODER_TICKS_TO_ARM_ANGLE_DEGREES_CONSTANT;
	//36.762 degrees
	private static final double SHOOT_MID_ANGLE_ENCODER_FORWARD_ROTATIONS = (36.762 + 13)
		* ENCODER_TICKS_TO_ARM_ANGLE_DEGREES_CONSTANT;
	//40.368 degrees
	private static final double SHOOT_HIGH_ANGLE_ENCODER_FORWARD_ROTATIONS = (40.368 + 13)
		* ENCODER_TICKS_TO_ARM_ANGLE_DEGREES_CONSTANT;
	//150.98 degrees
	private static final double SHOOT_MID_ANGLE_ENCODER_BACKWARD_ROTATIONS = (150.98 + 13)
		* ENCODER_TICKS_TO_ARM_ANGLE_DEGREES_CONSTANT;
	//147.976 degrees
	private static final double SHOOT_HIGH_ANGLE_ENCODER_BACKWARD_ROTATIONS = (147.976 + 13)
		* ENCODER_TICKS_TO_ARM_ANGLE_DEGREES_CONSTANT;
	//-12.837 degrees
	private static final double SHOOT_LOW_ANGLE_ENCODER_ROTATIONS = 0.163
		* ENCODER_TICKS_TO_ARM_ANGLE_DEGREES_CONSTANT;
	//41.2 degrees
	private static final double SUBSTATION_PICKUP_ANGLE_ENCODER_FORWARD_ROTATIONS = (41.2 + 13)
		* ENCODER_TICKS_TO_ARM_ANGLE_DEGREES_CONSTANT;
	//142.46 degrees
	private static final double SUBSTATION_PICKUP_ANGLE_ENCODER_BACKWARD_ROTATIONS =
		(142.46 + 13) * ENCODER_TICKS_TO_ARM_ANGLE_DEGREES_CONSTANT;

	private static final double PID_PIVOT_MAX_POWER = 0.1;
	private static final double PID_PIVOT_SLOW_DOWN_MAX_POWER = 0.05;
	private static final double ERROR_ARM_ROTATIONS = 0.3;
	private static final double PID_CONSTANT_PIVOT_P = 0.00022f;
	private static final double PID_CONSTANT_PIVOT_I = 0.000055f;
	private static final double PID_CONSTANT_PIVOT_D = 0.000008f;
	private static final double PID_CONSTANT_ARM_P = 0.00022f;
	private static final double PID_CONSTANT_ARM_I = 0.000055f;
	private static final double PID_CONSTANT_ARM_D = 0.000008f;
	private static final double PID_ARM_MAX_POWER = 0.1;
	private static final double JOYSTICK_DRIFT_Y = 0.03;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	private CANSparkMax pivotMotor;
	private CANSparkMax teleArmMotor;
	private SparkMaxLimitSwitch pivotLimitSwitchHigh;
	private SparkMaxLimitSwitch pivotLimitSwitchLow;
	private SparkMaxPIDController pidControllerPivot;
	private SparkMaxPIDController pidControllerTeleArm;
	private SparkMaxLimitSwitch teleArmLimitSwitch;

	private double pivotEncoderRotationsIntoIdle = 0;
	private double pivotEncoderRotationsAfterPivot = 0;
	private boolean isFineTuning = false;
	/**
	 * Creates an instance of an ArmFSM.
	 */
	public ArmFSM() {
		// Perform hardware init
		pivotMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_PIVOT,
									CANSparkMax.MotorType.kBrushless);
		pivotLimitSwitchHigh = pivotMotor.getReverseLimitSwitch(
								SparkMaxLimitSwitch.Type.kNormallyClosed);
		pivotLimitSwitchHigh.enableLimitSwitch(true);
		pivotLimitSwitchLow = pivotMotor.getForwardLimitSwitch(
								SparkMaxLimitSwitch.Type.kNormallyClosed);
		pivotLimitSwitchLow.enableLimitSwitch(true);
		pivotMotor.setInverted(true);
		pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		teleArmMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_TELEARM,
										CANSparkMax.MotorType.kBrushless);
		teleArmLimitSwitch = teleArmMotor.getReverseLimitSwitch(
								SparkMaxLimitSwitch.Type.kNormallyClosed);
		teleArmLimitSwitch.enableLimitSwitch(true);
		teleArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		pidControllerPivot = pivotMotor.getPIDController();
		pidControllerPivot.setP(PID_CONSTANT_PIVOT_P);
		pidControllerPivot.setI(PID_CONSTANT_PIVOT_I);
		pidControllerPivot.setD(PID_CONSTANT_PIVOT_D);
		pidControllerPivot.setIZone(0);
		pidControllerPivot.setFF(0);
		pidControllerPivot.setOutputRange(-PID_PIVOT_MAX_POWER, PID_PIVOT_MAX_POWER);
		pidControllerTeleArm = teleArmMotor.getPIDController();
		pidControllerTeleArm.setP(PID_CONSTANT_ARM_P);
		pidControllerTeleArm.setI(PID_CONSTANT_ARM_I);
		pidControllerTeleArm.setD(PID_CONSTANT_ARM_D);
		pidControllerTeleArm.setIZone(0);
		pidControllerTeleArm.setFF(0);
		pidControllerTeleArm.setOutputRange(-PID_ARM_MAX_POWER, PID_ARM_MAX_POWER);
		// Reset state machine
		reset();
	}

	/**
	 * Get the current state.
	 *
	 * @return current state
	 */
	public FSMState getCurrentState() {
		return currentState;
	}

	/**
	 * On robot start set the start to IDLE state. Resets robot to original state.
	 */
	public void reset() {
		currentState = FSMState.IDLE;
		pivotMotor.getEncoder().setPosition(0);
		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Updates the current FSMState.
	 * @param input TeleopInput
	 */
	public void update(TeleopInput input) {
		if (input == null) {
			handleIdleState();
			return;
		}
		SmartDashboard.putString("Current State", " " + currentState);
		SmartDashboard.putNumber("Pivot Motor Rotations", pivotMotor.getEncoder().getPosition());
		SmartDashboard.putNumber("Arm Motor Rotations", teleArmMotor.getEncoder().getPosition());
		SmartDashboard.putBoolean("At Max Height", isMaxHeight());
		SmartDashboard.putBoolean("At Min Height", isMinHeight());
		SmartDashboard.putBoolean("At Arm Min", teleArmLimitSwitch.isPressed());
		SmartDashboard.putBoolean("Is going Forward", input.isThrottleForward());
		SmartDashboard.putNumber("Throttle Value", input.getThrottle());
		SmartDashboard.putNumber("Extension Fraction", teleArmMotor.getEncoder().getPosition()
			/ ARM_ENCODER_MAX_LENGTH_ROTATIONS);
		System.out.println(pivotMotor.getEncoder().getPosition());
		if (input.isFineTuningButtonPressed()) {
			isFineTuning = !isFineTuning;
		}
		if (pivotMotor.getEncoder().getPosition() < ENCODER_TICKS_SLOW_DOWN_RANGE_MAX
			&& pivotMotor.getEncoder().getPosition() > ENCODER_TICKS_SLOW_DOWN_RANGE_MIN) {
			pidControllerPivot.setOutputRange(-PID_PIVOT_SLOW_DOWN_MAX_POWER,
				PID_PIVOT_SLOW_DOWN_MAX_POWER);
		} else {
			pidControllerPivot.setOutputRange(-PID_PIVOT_MAX_POWER,
				PID_PIVOT_MAX_POWER);
		}
		if (isMinHeight()) {
			pivotMotor.getEncoder().setPosition(0);
		}
		if (teleArmLimitSwitch.isPressed()) {
			teleArmMotor.getEncoder().setPosition(0);
		}
		switch (currentState) {
			case UNHOMED_STATE:
				handleUnhomedState();
				break;
			case IDLE:
				handleIdleState();
				break;
			case HOMING_STATE:
				handleHomingState(input);
				break;
			case MOVING_TO_START_STATE:
				handleMovingToStartState(input);
				break;
			case ARM_MOVEMENT:
				handleArmMechState(input);
				break;
			case SHOOT_HIGH_FORWARD:
				handleShootHighForwardState(input);
				break;
			case SHOOT_HIGH_BACKWARD:
				handleShootHighBackwardState(input);
				break;
			case SHOOT_MID_FORWARD:
				handleShootMidForwardState(input);
				break;
			case SHOOT_MID_BACKWARD:
				handleShootMidBackwardState(input);
				break;
			case SHOOT_LOW_FORWARD:
				handleShootLowState(input);
				break;
			case SUBSTATION_PICKUP_FORWARD:
				handleSubstationPickupForwardState(input);
				break;
			case SUBSTATION_PICKUP_BACKWARD:
				handleSubstationPickupBackwardState(input);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		FSMState state = nextState(input);
		if (currentState != state) {
			if (state == FSMState.IDLE) {
				pivotEncoderRotationsIntoIdle = pivotMotor.getEncoder().getPosition();
			}
		}
		currentState = state;
	}

	/**
	 * Updates the Autonomous FSMState.
	 * @return finished
	 * @param state TeleopInput
	 */
	public boolean updateAuto(FSMState state) {
		SmartDashboard.putString("Current State", " " + currentState);
		SmartDashboard.putNumber("Pivot Motor Rotations", pivotMotor.getEncoder().getPosition());
		SmartDashboard.putNumber("Arm Motor Rotations", teleArmMotor.getEncoder().getPosition());
		SmartDashboard.putBoolean("At Max Height", isMaxHeight());
		SmartDashboard.putBoolean("At Min Height", isMinHeight());
		if (pivotMotor.getEncoder().getPosition() < ENCODER_TICKS_SLOW_DOWN_RANGE_MAX
			&& pivotMotor.getEncoder().getPosition() > ENCODER_TICKS_SLOW_DOWN_RANGE_MIN) {
			System.out.println("HIGH");
			pidControllerPivot.setOutputRange(-PID_PIVOT_SLOW_DOWN_MAX_POWER,
				PID_PIVOT_SLOW_DOWN_MAX_POWER);
		} else {
			pidControllerPivot.setOutputRange(-PID_PIVOT_MAX_POWER,
				PID_PIVOT_MAX_POWER);
		}
		if (isMinHeight()) {
			pivotMotor.getEncoder().setPosition(0);
		}
		if (teleArmLimitSwitch.isPressed()) {
			teleArmMotor.getEncoder().setPosition(0);
		}
		switch (state) {
			case IDLE:
				handleIdleState();
				return true;
			case AUTONOMOUS_RETRACT:
				handleAutonomousRetractState(null);
				return teleArmLimitSwitch.isPressed();
			case SHOOT_HIGH_FORWARD:
				handleShootHighForwardState(null);
				return atArmPosition(SHOOT_HIGH_ANGLE_ENCODER_FORWARD_ROTATIONS,
									ARM_ENCODER_HIGH_FORWARD_CUBE_ROTATIONS);
			case SHOOT_HIGH_BACKWARD:
				handleShootHighBackwardState(null);
				return atArmPosition(SHOOT_HIGH_ANGLE_ENCODER_BACKWARD_ROTATIONS,
									ARM_ENCODER_HIGH_BACKWARD_ROTATIONS);
			case SHOOT_MID_FORWARD:
				handleShootMidForwardState(null);
				return atArmPosition(SHOOT_MID_ANGLE_ENCODER_FORWARD_ROTATIONS,
									ARM_ENCODER_MID_FORWARD_ROTATIONS);
			case SHOOT_MID_BACKWARD:
				handleShootMidBackwardState(null);
				return atArmPosition(SHOOT_MID_ANGLE_ENCODER_BACKWARD_ROTATIONS,
									ARM_ENCODER_MID_BACKWARD_ROTATIONS);
			case SHOOT_LOW_FORWARD:
				handleShootLowState(null);
				return atArmPosition(SHOOT_LOW_ANGLE_ENCODER_ROTATIONS, ARM_ENCODER_LOW_ROTATIONS);
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}
	/*
	 * When inputs are pressed, the states will change likewise
	 *
	 * @return expected state
	 */
	private FSMState nextState(TeleopInput input) {
		if (input == null) {
			return FSMState.IDLE;
		}
		switch (currentState) {
			case UNHOMED_STATE:
				if (input.isHomingButtonPressed()) {
					return FSMState.HOMING_STATE;
				}
				return FSMState.UNHOMED_STATE;
			case IDLE:
				if (input.isShootHighButtonPressed() && input.isThrottleForward()
						&& SpinningIntakeFSM.getObjectType() == SpinningIntakeFSM.ItemType.CUBE
						&& !atArmPosition(SHOOT_HIGH_ANGLE_ENCODER_FORWARD_ROTATIONS,
							ARM_ENCODER_HIGH_FORWARD_CUBE_ROTATIONS)
						|| input.isShootHighButtonPressed() && input.isThrottleForward()
						&& SpinningIntakeFSM.getObjectType() != SpinningIntakeFSM.ItemType.CUBE
						&& !atArmPosition(SHOOT_HIGH_ANGLE_ENCODER_FORWARD_ROTATIONS,
							ARM_ENCODER_HIGH_FORWARD_CONE_ROTATIONS)) {
					return FSMState.SHOOT_HIGH_FORWARD;
				} else if (input.isShootHighButtonPressed() && !input.isThrottleForward()
						&& !atArmPosition(SHOOT_HIGH_ANGLE_ENCODER_BACKWARD_ROTATIONS,
							ARM_ENCODER_HIGH_BACKWARD_ROTATIONS)) {
					return FSMState.SHOOT_HIGH_BACKWARD;
				} else if (input.isShootMidButtonPressed() && input.isThrottleForward()
						&& !atArmPosition(SHOOT_MID_ANGLE_ENCODER_FORWARD_ROTATIONS,
							ARM_ENCODER_MID_FORWARD_ROTATIONS)) {
					return FSMState.SHOOT_MID_FORWARD;
				} else if (input.isShootMidButtonPressed() && !input.isThrottleForward()
						&& !atArmPosition(SHOOT_MID_ANGLE_ENCODER_BACKWARD_ROTATIONS,
							ARM_ENCODER_MID_BACKWARD_ROTATIONS)) {
					return FSMState.SHOOT_MID_BACKWARD;
				} else if (input.isShootLowButtonPressed()
						&& !atArmPosition(SHOOT_LOW_ANGLE_ENCODER_ROTATIONS,
							ARM_ENCODER_LOW_ROTATIONS)) {
					return FSMState.SHOOT_LOW_FORWARD;
				} else if (input.isSubstationPickupButtonPressed() && input.isThrottleForward()
						&& !atArmPosition(SUBSTATION_PICKUP_ANGLE_ENCODER_FORWARD_ROTATIONS,
							ARM_ENCODER_SUBSTATION_FORWARD_ROTATIONS)) {
					return FSMState.SUBSTATION_PICKUP_FORWARD;
				} else if (input.isSubstationPickupButtonPressed() && !input.isThrottleForward()
						&& !atArmPosition(SUBSTATION_PICKUP_ANGLE_ENCODER_BACKWARD_ROTATIONS,
							ARM_ENCODER_SUBSTATION_BACKWARD_ROTATIONS)) {
					return FSMState.SUBSTATION_PICKUP_BACKWARD;
				} else if (!isMovingAtLimit(input) && isArmMovementInputPressed(input)) {
					return FSMState.ARM_MOVEMENT;
				}
				return FSMState.IDLE;
			case HOMING_STATE:
				if (isMinHeight()) {
					return FSMState.MOVING_TO_START_STATE;
				} else if (input.isHomingButtonPressed()) {
					return FSMState.HOMING_STATE;
				}
				return FSMState.UNHOMED_STATE;
			case MOVING_TO_START_STATE:
				if (withinError(pivotMotor.getEncoder().getPosition(),
					ARM_ENCODER_STARTING_ANGLE_ROTATIONS)) {
					return FSMState.IDLE;
				} else if (input.isHomingButtonPressed()) {
					return FSMState.MOVING_TO_START_STATE;
				}
				return FSMState.UNHOMED_STATE;
			case ARM_MOVEMENT:
				if (isArmMovementInputPressed(input) && !isMovingAtLimit(input)
					&& !isShootOrPickupButtonPressed(input)) {
					return FSMState.ARM_MOVEMENT;
				}
				return FSMState.IDLE;
			case SHOOT_HIGH_FORWARD:
				if (input.isShootHighButtonPressed() && input.isThrottleForward()
					&& SpinningIntakeFSM.getObjectType() == SpinningIntakeFSM.ItemType.CUBE
					&& !atArmPosition(SHOOT_HIGH_ANGLE_ENCODER_FORWARD_ROTATIONS,
						ARM_ENCODER_HIGH_FORWARD_CUBE_ROTATIONS)
					|| input.isShootHighButtonPressed() && input.isThrottleForward()
					&& SpinningIntakeFSM.getObjectType() != SpinningIntakeFSM.ItemType.CUBE
					&& !atArmPosition(SHOOT_HIGH_ANGLE_ENCODER_FORWARD_ROTATIONS,
						ARM_ENCODER_HIGH_FORWARD_CONE_ROTATIONS)) {
					return FSMState.SHOOT_HIGH_FORWARD;
				}
				return FSMState.IDLE;
			case SHOOT_HIGH_BACKWARD:
				if (input.isShootHighButtonPressed() && !input.isThrottleForward()
						&& !atArmPosition(SHOOT_HIGH_ANGLE_ENCODER_BACKWARD_ROTATIONS,
						ARM_ENCODER_HIGH_BACKWARD_ROTATIONS)) {
					return FSMState.SHOOT_HIGH_BACKWARD;
				}
				return FSMState.IDLE;
			case SHOOT_MID_FORWARD:
				if (input.isShootMidButtonPressed() && input.isThrottleForward()
					&& !atArmPosition(SHOOT_MID_ANGLE_ENCODER_FORWARD_ROTATIONS,
						ARM_ENCODER_MID_FORWARD_ROTATIONS)) {
					return FSMState.SHOOT_MID_FORWARD;
				}
				return FSMState.IDLE;
			case SHOOT_MID_BACKWARD:
				if (input.isShootMidButtonPressed() && !input.isThrottleForward()
					&& !atArmPosition(SHOOT_MID_ANGLE_ENCODER_BACKWARD_ROTATIONS,
						ARM_ENCODER_MID_BACKWARD_ROTATIONS)) {
					return FSMState.SHOOT_MID_BACKWARD;
				}
				return FSMState.IDLE;
			case SHOOT_LOW_FORWARD:
				if (input.isShootLowButtonPressed()
					&& !atArmPosition(SHOOT_LOW_ANGLE_ENCODER_ROTATIONS,
						ARM_ENCODER_LOW_ROTATIONS)) {
					return FSMState.SHOOT_LOW_FORWARD;
				}
				return FSMState.IDLE;
			case SUBSTATION_PICKUP_FORWARD:
				if (input.isSubstationPickupButtonPressed() && input.isThrottleForward()
					&& !atArmPosition(SUBSTATION_PICKUP_ANGLE_ENCODER_FORWARD_ROTATIONS,
						ARM_ENCODER_SUBSTATION_FORWARD_ROTATIONS)) {
					return FSMState.SUBSTATION_PICKUP_FORWARD;
				}
				return FSMState.IDLE;
			case SUBSTATION_PICKUP_BACKWARD:
				if (input.isSubstationPickupButtonPressed() && !input.isThrottleForward()
					&& !atArmPosition(SUBSTATION_PICKUP_ANGLE_ENCODER_BACKWARD_ROTATIONS,
						ARM_ENCODER_SUBSTATION_BACKWARD_ROTATIONS)) {
					return FSMState.SUBSTATION_PICKUP_BACKWARD;
				}
				return FSMState.IDLE;
			default :
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/**
	 * Returns whether the difference between two rotations is within the error.
	 * @param a rotation value 1
	 * @param b rotation value 2
	 * @return whether they are within the error
	 */
	private boolean withinError(double a, double b) {
		return Math.abs(a - b) < ERROR_ARM_ROTATIONS;
	}

	private boolean isMaxHeight() {
		return pivotLimitSwitchHigh.isPressed();
	}

	private boolean isMinHeight() {
		return pivotLimitSwitchLow.isPressed();
	}

	private boolean isArmMovementInputPressed(TeleopInput input) {
		if (input == null) {
			return false;
		}
		return input.isPivotDecreaseButtonPressed()
			|| input.isPivotIncreaseButtonPressed()
			|| Math.abs(input.getmechJoystickY()) > JOYSTICK_DRIFT_Y;
	}

	private boolean isShootOrPickupButtonPressed(TeleopInput input) {
		if (input == null) {
			return false;
		}
		return input.isShootHighButtonPressed()
			|| input.isShootMidButtonPressed()
			|| input.isShootLowButtonPressed()
			|| input.isSubstationPickupButtonPressed();
	}

	private boolean isMovingAtLimit(TeleopInput input) {
		if (input == null) {
			return false;
		}
		return input.isPivotDecreaseButtonPressed() && isMinHeight()
			|| input.isPivotIncreaseButtonPressed() && isMaxHeight();
	}

	private boolean atArmPosition(double pivotTarget, double armTarget) {
		return withinError(pivotMotor.getEncoder().getPosition(), pivotTarget)
			&& withinError(teleArmMotor.getEncoder().getPosition(), armTarget);
	}

	private void handleUnhomedState() {
		teleArmMotor.set(0);
		pivotMotor.set(0);
	}
	/*
	 * What to do when in the IDLE state
	 */
	private void handleIdleState() {
		teleArmMotor.set(0);
		pidControllerPivot.setReference(pivotEncoderRotationsIntoIdle,
			CANSparkMax.ControlType.kPosition);
	}

	private void handleAutonomousRetractState(TeleopInput input) {
		if (teleArmMotor.getEncoder().getPosition() > 0) {
			pidControllerTeleArm.setReference(0, CANSparkMax.ControlType.kPosition);
		} else {
			teleArmMotor.set(0);
		}
	}

	private void handleHomingState(TeleopInput input) {
		if (isMinHeight()) {
			pivotMotor.set(0);
		} else {
			pivotMotor.set(PIVOT_MOTOR_POWER);
		}
		if (teleArmLimitSwitch.isPressed()) {
			teleArmMotor.set(0);
		} else {
			teleArmMotor.set(-TELEARM_MOTOR_POWER);
		}
	}

	private void handleMovingToStartState(TeleopInput input) {
		if (withinError(pivotMotor.getEncoder().getPosition(),
			ARM_ENCODER_STARTING_ANGLE_ROTATIONS)) {
			pivotMotor.set(0);
		} else {
			pidControllerPivot.setReference(ARM_ENCODER_STARTING_ANGLE_ROTATIONS,
				CANSparkMax.ControlType.kPosition);
		}
	}
	/*
	 * What to do when in the ARM_MOVEMENT state
	 */
	private void handleArmMechState(TeleopInput input) {
		if (input != null) {
			if (!isFineTuning) {
				if (input.isPivotIncreaseButtonPressed() && !isMaxHeight()) {
					if (pivotMotor.getEncoder().getPosition() < ENCODER_TICKS_SLOW_DOWN_RANGE_MAX
						&& pivotMotor.getEncoder().getPosition()
							> ENCODER_TICKS_SLOW_DOWN_RANGE_MIN) {
						pidControllerPivot.setReference(-PIVOT_MOTOR_SLOW_DOWN_POWER,
							CANSparkMax.ControlType.kDutyCycle);
						pivotEncoderRotationsAfterPivot = pivotMotor.getEncoder().getPosition();
					} else {
						pidControllerPivot.setReference(-PIVOT_MOTOR_POWER,
							CANSparkMax.ControlType.kDutyCycle);
						pivotEncoderRotationsAfterPivot = pivotMotor.getEncoder().getPosition();
					}
				} else if (input.isPivotDecreaseButtonPressed() && !isMinHeight()) {
					if (pivotMotor.getEncoder().getPosition() < ENCODER_TICKS_SLOW_DOWN_RANGE_MAX
						&& pivotMotor.getEncoder().getPosition()
						> ENCODER_TICKS_SLOW_DOWN_RANGE_MIN) {
						pidControllerPivot.setReference(PIVOT_MOTOR_SLOW_DOWN_POWER,
							CANSparkMax.ControlType.kDutyCycle);
						pivotEncoderRotationsAfterPivot = pivotMotor.getEncoder().getPosition();
					} else {
						pidControllerPivot.setReference(PIVOT_MOTOR_POWER,
							CANSparkMax.ControlType.kDutyCycle);
						pivotEncoderRotationsAfterPivot = pivotMotor.getEncoder().getPosition();
					}
				} else {
					if (pivotEncoderRotationsAfterPivot != 0) {
						pidControllerPivot.setReference(pivotEncoderRotationsAfterPivot,
							CANSparkMax.ControlType.kPosition);
					} else {
						pivotMotor.set(0);
					}
				}
				teleArmMotor.set(-input.getmechJoystickY() * TELEARM_MOTOR_POWER);
			} else {
				if (input.isPivotIncreaseButtonPressed() && !isMaxHeight()) {
					pidControllerPivot.setReference(-PIVOT_MOTOR_POWER_FINE_TUNING,
						CANSparkMax.ControlType.kDutyCycle);
					pivotEncoderRotationsAfterPivot = pivotMotor.getEncoder().getPosition();
				} else if (input.isPivotDecreaseButtonPressed() && !isMinHeight()) {
					pidControllerPivot.setReference(PIVOT_MOTOR_POWER_FINE_TUNING,
						CANSparkMax.ControlType.kDutyCycle);
					pivotEncoderRotationsAfterPivot = pivotMotor.getEncoder().getPosition();
				} else {
					if (pivotEncoderRotationsAfterPivot != 0) {
						pidControllerPivot.setReference(pivotEncoderRotationsAfterPivot,
							CANSparkMax.ControlType.kPosition);
					} else {
						pivotMotor.set(0);
					}
				}
				teleArmMotor.set(-input.getmechJoystickY() * TELEARM_MOTOR_POWER_FINE_TUNING);
			}
		} else {
			teleArmMotor.set(0);
			pivotMotor.set(0);
		}
	}

	private void handleShootHighForwardState(TeleopInput input) {
		if (input != null) {
			if (SpinningIntakeFSM.getObjectType() == SpinningIntakeFSM.ItemType.CUBE) {
				if (withinError(pivotMotor.getEncoder().getPosition(),
					SHOOT_HIGH_ANGLE_ENCODER_FORWARD_ROTATIONS) || isMaxHeight() || isMinHeight()) {
					pivotMotor.set(0);
					if (withinError(teleArmMotor.getEncoder().getPosition(),
						ARM_ENCODER_HIGH_FORWARD_CUBE_ROTATIONS)) {
						teleArmMotor.set(0);
					} else {
						pidControllerTeleArm.setReference(ARM_ENCODER_HIGH_FORWARD_CUBE_ROTATIONS,
							CANSparkMax.ControlType.kPosition);
					}
				} else {
					pidControllerPivot.setReference(SHOOT_HIGH_ANGLE_ENCODER_FORWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
				if (pivotEncoderRotationsIntoIdle > ARM_ENCODER_VERTICAL_ANGLE_ROTATIONS) {
					if (withinError(teleArmMotor.getEncoder().getPosition(),
						ARM_ENCODER_HIGH_FORWARD_CUBE_ROTATIONS)) {
						teleArmMotor.set(0);
					} else {
						pidControllerTeleArm.setReference(ARM_ENCODER_HIGH_FORWARD_CUBE_ROTATIONS,
							CANSparkMax.ControlType.kPosition);
					}
				}
			} else {
				if (withinError(pivotMotor.getEncoder().getPosition(),
					SHOOT_HIGH_ANGLE_ENCODER_FORWARD_ROTATIONS) || isMaxHeight() || isMinHeight()) {
					pivotMotor.set(0);
					if (withinError(teleArmMotor.getEncoder().getPosition(),
						ARM_ENCODER_HIGH_FORWARD_CONE_ROTATIONS)) {
						teleArmMotor.set(0);
					} else {
						pidControllerTeleArm.setReference(ARM_ENCODER_HIGH_FORWARD_CONE_ROTATIONS,
							CANSparkMax.ControlType.kPosition);
					}
				} else {
					pidControllerPivot.setReference(SHOOT_HIGH_ANGLE_ENCODER_FORWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
				if (pivotEncoderRotationsIntoIdle > ARM_ENCODER_VERTICAL_ANGLE_ROTATIONS) {
					if (withinError(teleArmMotor.getEncoder().getPosition(),
						ARM_ENCODER_HIGH_FORWARD_CONE_ROTATIONS)) {
						teleArmMotor.set(0);
					} else {
						pidControllerTeleArm.setReference(ARM_ENCODER_HIGH_FORWARD_CONE_ROTATIONS,
							CANSparkMax.ControlType.kPosition);
					}
				}
			}
		} else {
			if (withinError(pivotMotor.getEncoder().getPosition(),
					SHOOT_HIGH_ANGLE_ENCODER_FORWARD_ROTATIONS)) {
				pivotMotor.set(0);
				if (withinError(teleArmMotor.getEncoder().getPosition(),
					ARM_ENCODER_HIGH_FORWARD_CUBE_ROTATIONS)) {
					teleArmMotor.set(0);
				} else {
					pidControllerTeleArm.setReference(ARM_ENCODER_HIGH_FORWARD_CUBE_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
			} else {
				pidControllerPivot.setReference(SHOOT_HIGH_ANGLE_ENCODER_FORWARD_ROTATIONS,
					CANSparkMax.ControlType.kPosition);
			}
		}
	}

	private void handleShootHighBackwardState(TeleopInput input) {
		if (input != null) {
			if (withinError(pivotMotor.getEncoder().getPosition(),
				SHOOT_HIGH_ANGLE_ENCODER_BACKWARD_ROTATIONS)) {
				pivotMotor.set(0);
				if (withinError(teleArmMotor.getEncoder().getPosition(),
					ARM_ENCODER_HIGH_BACKWARD_ROTATIONS)) {
					teleArmMotor.set(0);
				} else {
					pidControllerTeleArm.setReference(ARM_ENCODER_HIGH_BACKWARD_ROTATIONS,
							CANSparkMax.ControlType.kPosition);
				}
			} else {
				pidControllerPivot.setReference(SHOOT_HIGH_ANGLE_ENCODER_BACKWARD_ROTATIONS,
					CANSparkMax.ControlType.kPosition);
			}
			if (pivotEncoderRotationsIntoIdle <= ARM_ENCODER_VERTICAL_ANGLE_ROTATIONS) {
				if (withinError(teleArmMotor.getEncoder().getPosition(),
						ARM_ENCODER_HIGH_BACKWARD_ROTATIONS)) {
					teleArmMotor.set(0);
				} else {
					pidControllerTeleArm.setReference(ARM_ENCODER_HIGH_BACKWARD_ROTATIONS,
							CANSparkMax.ControlType.kPosition);
				}
			}
		} else {
			if (withinError(pivotMotor.getEncoder().getPosition(),
				SHOOT_HIGH_ANGLE_ENCODER_BACKWARD_ROTATIONS)) {
				pivotMotor.set(0);
				if (withinError(teleArmMotor.getEncoder().getPosition(),
					ARM_ENCODER_HIGH_BACKWARD_ROTATIONS)) {
					teleArmMotor.set(0);
				} else {
					pidControllerTeleArm.setReference(ARM_ENCODER_HIGH_BACKWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
			} else {
				pidControllerPivot.setReference(SHOOT_HIGH_ANGLE_ENCODER_BACKWARD_ROTATIONS,
					CANSparkMax.ControlType.kPosition);
			}
		}
	}


	private void handleShootMidForwardState(TeleopInput input) {
		if (input != null) {
			if (withinError(pivotMotor.getEncoder().getPosition(),
				SHOOT_MID_ANGLE_ENCODER_FORWARD_ROTATIONS)) {
				pivotMotor.set(0);
				if (withinError(teleArmMotor.getEncoder().getPosition(),
					ARM_ENCODER_MID_FORWARD_ROTATIONS)) {
					teleArmMotor.set(0);
				} else {
					pidControllerTeleArm.setReference(ARM_ENCODER_MID_FORWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
			} else {
				pidControllerPivot.setReference(SHOOT_MID_ANGLE_ENCODER_FORWARD_ROTATIONS,
					CANSparkMax.ControlType.kPosition);
			}
			if (pivotEncoderRotationsIntoIdle > ARM_ENCODER_VERTICAL_ANGLE_ROTATIONS) {
				if (withinError(teleArmMotor.getEncoder().getPosition(),
						ARM_ENCODER_MID_FORWARD_ROTATIONS)) {
					teleArmMotor.set(0);
				} else {
					pidControllerTeleArm.setReference(ARM_ENCODER_MID_FORWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
			}
		} else {
			if (withinError(pivotMotor.getEncoder().getPosition(),
				SHOOT_MID_ANGLE_ENCODER_FORWARD_ROTATIONS)) {
				pivotMotor.set(0);
				if (withinError(teleArmMotor.getEncoder().getPosition(),
					ARM_ENCODER_MID_FORWARD_ROTATIONS)) {
					teleArmMotor.set(0);
				} else {
					pidControllerTeleArm.setReference(ARM_ENCODER_MID_FORWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
			} else {
				pidControllerPivot.setReference(SHOOT_MID_ANGLE_ENCODER_FORWARD_ROTATIONS,
					CANSparkMax.ControlType.kPosition);
			}
		}
	}

	private void handleShootMidBackwardState(TeleopInput input) {
		if (input != null) {
			if (withinError(pivotMotor.getEncoder().getPosition(),
				SHOOT_MID_ANGLE_ENCODER_BACKWARD_ROTATIONS)) {
				pivotMotor.set(0);
				if (withinError(teleArmMotor.getEncoder().getPosition(),
					ARM_ENCODER_MID_BACKWARD_ROTATIONS)) {
					teleArmMotor.set(0);
				} else {
					pidControllerTeleArm.setReference(ARM_ENCODER_MID_BACKWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
			} else {
				pidControllerPivot.setReference(SHOOT_MID_ANGLE_ENCODER_BACKWARD_ROTATIONS,
					CANSparkMax.ControlType.kPosition);
			}
			if (pivotEncoderRotationsIntoIdle <= ARM_ENCODER_VERTICAL_ANGLE_ROTATIONS) {
				if (withinError(teleArmMotor.getEncoder().getPosition(),
						ARM_ENCODER_MID_BACKWARD_ROTATIONS)) {
					teleArmMotor.set(0);
				} else {
					pidControllerTeleArm.setReference(ARM_ENCODER_MID_BACKWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
			}
		} else {
			if (withinError(pivotMotor.getEncoder().getPosition(),
				SHOOT_MID_ANGLE_ENCODER_BACKWARD_ROTATIONS)) {
				pivotMotor.set(0);
				if (withinError(teleArmMotor.getEncoder().getPosition(),
					ARM_ENCODER_MID_BACKWARD_ROTATIONS)) {
					teleArmMotor.set(0);
				} else {
					pidControllerTeleArm.setReference(ARM_ENCODER_MID_BACKWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
			} else {
				pidControllerPivot.setReference(SHOOT_MID_ANGLE_ENCODER_BACKWARD_ROTATIONS,
					CANSparkMax.ControlType.kPosition);
			}
		}
	}

	private void handleShootLowState(TeleopInput input) {
		if (input != null) {
			if (withinError(pivotMotor.getEncoder().getPosition(),
				SHOOT_LOW_ANGLE_ENCODER_ROTATIONS)) {
				pivotMotor.set(0);
				if (withinError(teleArmMotor.getEncoder().getPosition(),
						ARM_ENCODER_LOW_ROTATIONS)) {
					teleArmMotor.set(0);
				} else {
					pidControllerTeleArm.setReference(ARM_ENCODER_LOW_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
			} else {
				pidControllerPivot.setReference(SHOOT_LOW_ANGLE_ENCODER_ROTATIONS,
					CANSparkMax.ControlType.kPosition);
			}
			if (pivotEncoderRotationsIntoIdle > ARM_ENCODER_VERTICAL_ANGLE_ROTATIONS) {
				if (withinError(teleArmMotor.getEncoder().getPosition(),
							ARM_ENCODER_LOW_ROTATIONS)) {
					teleArmMotor.set(0);
				} else {
					pidControllerTeleArm.setReference(ARM_ENCODER_LOW_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
			}
		} else {
			if (withinError(pivotMotor.getEncoder().getPosition(),
				SHOOT_LOW_ANGLE_ENCODER_ROTATIONS)) {
				pivotMotor.set(0);
				if (withinError(teleArmMotor.getEncoder().getPosition(),
					ARM_ENCODER_LOW_ROTATIONS)) {
					teleArmMotor.set(0);
				} else {
					pidControllerTeleArm.setReference(ARM_ENCODER_LOW_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
			} else {
				pidControllerPivot.setReference(SHOOT_LOW_ANGLE_ENCODER_ROTATIONS,
					CANSparkMax.ControlType.kPosition);
			}
		}
	}


	private void handleSubstationPickupForwardState(TeleopInput input) {
		if (input != null) {
			if (withinError(pivotMotor.getEncoder().getPosition(),
				SUBSTATION_PICKUP_ANGLE_ENCODER_FORWARD_ROTATIONS)) {
				pivotMotor.set(0);
				if (withinError(teleArmMotor.getEncoder().getPosition(),
					ARM_ENCODER_SUBSTATION_FORWARD_ROTATIONS)) {
					teleArmMotor.set(0);
				} else {
					pidControllerTeleArm.setReference(ARM_ENCODER_SUBSTATION_FORWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
			} else {
				pidControllerPivot.setReference(
					SUBSTATION_PICKUP_ANGLE_ENCODER_FORWARD_ROTATIONS,
					CANSparkMax.ControlType.kPosition);
			}
			if (pivotEncoderRotationsIntoIdle > ARM_ENCODER_VERTICAL_ANGLE_ROTATIONS) {
				if (withinError(teleArmMotor.getEncoder().getPosition(),
						ARM_ENCODER_SUBSTATION_FORWARD_ROTATIONS)) {
					teleArmMotor.set(0);
				} else {
					pidControllerTeleArm.setReference(ARM_ENCODER_SUBSTATION_FORWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
			}
		} else {
			teleArmMotor.set(0);
			pivotMotor.set(0);
		}
	}

	private void handleSubstationPickupBackwardState(TeleopInput input) {
		if (input != null) {
			if (withinError(pivotMotor.getEncoder().getPosition(),
				SUBSTATION_PICKUP_ANGLE_ENCODER_BACKWARD_ROTATIONS)) {
				pivotMotor.set(0);
				if (withinError(teleArmMotor.getEncoder().getPosition(),
					ARM_ENCODER_SUBSTATION_BACKWARD_ROTATIONS)) {
					teleArmMotor.set(0);
				} else {
					pidControllerTeleArm.setReference(ARM_ENCODER_SUBSTATION_BACKWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
			} else {
				pidControllerPivot.setReference(SUBSTATION_PICKUP_ANGLE_ENCODER_BACKWARD_ROTATIONS,
					CANSparkMax.ControlType.kPosition);
			}
			if (pivotEncoderRotationsIntoIdle <= ARM_ENCODER_VERTICAL_ANGLE_ROTATIONS) {
				if (withinError(teleArmMotor.getEncoder().getPosition(),
					ARM_ENCODER_SUBSTATION_BACKWARD_ROTATIONS)) {
					teleArmMotor.set(0);
				} else {
					pidControllerTeleArm.setReference(ARM_ENCODER_SUBSTATION_BACKWARD_ROTATIONS,
						CANSparkMax.ControlType.kPosition);
				}
			}
		} else {
			teleArmMotor.set(0);
			pivotMotor.set(0);
		}
	}
}
