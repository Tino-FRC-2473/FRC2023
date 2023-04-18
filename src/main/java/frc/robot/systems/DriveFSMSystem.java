package frc.robot.systems;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.Constants;
import frc.robot.HardwareMap;

// Java Imports
import java.util.*;

public class DriveFSMSystem {

	// FSM state definitions
	public enum FSMState {
		PURE_PERSUIT,
		IDLE
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax leftMotorBack;
	private CANSparkMax rightMotorFront;
	private CANSparkMax leftMotorFront;
	private CANSparkMax rightMotorBack;

	private double roboXPos = 0;
	private double roboYPos = 0;
	private double currentEncoderPos;
	private double prevEncoderPos = 0;
	private double gyroAngleForOdo = 0;
	private AHRS gyro;

	// pure pursuit
	private int stateCounter = 1;
	private double[][] waypoints = new double[2][Constants.PARTITIONS + 1];
	private int target = -1;
	private int direction = 1;
	private int pointNum = 0;
	private boolean firstRun = true;
	

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public DriveFSMSystem() {
		// Perform hardware init
		leftMotorBack = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_LEFT_BACK,
										CANSparkMax.MotorType.kBrushless);
		rightMotorFront = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_RIGHT_FRONT,
										CANSparkMax.MotorType.kBrushless);
		leftMotorFront = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_LEFT_FRONT,
										CANSparkMax.MotorType.kBrushless);
		rightMotorBack = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_RIGHT_BACK,
										CANSparkMax.MotorType.kBrushless);

		rightMotorFront.getEncoder().setPosition(0);
		leftMotorBack.getEncoder().setPosition(0);
		rightMotorBack.getEncoder().setPosition(0);
		leftMotorFront.getEncoder().setPosition(0);

		rightMotorFront.set(0);
		leftMotorBack.set(0);
		rightMotorBack.set(0);
		leftMotorFront.set(0);

		gyro = new AHRS(SPI.Port.kMXP);

		gyro.reset();
		gyro.zeroYaw();
		gyroAngleForOdo = 0;
		roboXPos = 0;
		roboYPos = 0;
		updateLineOdometryTele(0);

		// Reset state machine
		resetAutonomous();
		resetTeleop();

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
	public void resetAutonomous() {

		rightMotorFront.getEncoder().setPosition(0);
		leftMotorBack.getEncoder().setPosition(0);
		rightMotorBack.getEncoder().setPosition(0);
		leftMotorFront.getEncoder().setPosition(0);

		rightMotorFront.set(0);
		leftMotorBack.set(0);
		rightMotorBack.set(0);
		leftMotorFront.set(0);

		gyro.zeroYaw();
		gyroAngleForOdo = 0;
		currentEncoderPos = 0;
		prevEncoderPos = 0;
		roboXPos = 0;
		roboYPos = 0;
		updateLineOdometryTele(0);

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void resetTeleop() {

		rightMotorFront.getEncoder().setPosition(0);
		leftMotorBack.getEncoder().setPosition(0);
		rightMotorBack.getEncoder().setPosition(0);
		leftMotorFront.getEncoder().setPosition(0);

		rightMotorFront.set(0);
		leftMotorBack.set(0);
		rightMotorBack.set(0);
		leftMotorFront.set(0);

		gyro.zeroYaw();
		gyroAngleForOdo = 0;
		currentEncoderPos = 0;
		prevEncoderPos = 0;
		roboXPos = 0;
		roboYPos = 0;
		updateLineOdometryTele(0);

		currentState = FSMState.IDLE;

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot in autonomous mode.
	 */
	public void update(TeleopInput input) {
		gyroAngleForOdo = gyro.getAngle() * Constants.GYRO_MULTIPLER_TELOP;

		currentEncoderPos = ((leftMotorBack.getEncoder().getPosition()
			- rightMotorFront.getEncoder().getPosition()) / 2.0);
		updateLineOdometryTele(gyroAngleForOdo);
		SmartDashboard.putString("Drive state", currentState.toString());
		SmartDashboard.putNumber("Robot X", roboXPos);
		SmartDashboard.putNumber("Robot Y", roboYPos);
		
		switch (currentState) {

			case IDLE:
				handleIdleState(input);
				break;

			case PURE_PERSUIT:
				handlePurePursuit(input, 0, 0, 5, 5, 10, 10);
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
		
		default: throw new IllegalStateException("Invalid state: " + currentState.toString()); }
	}

	/**
	 * Handle behavior in PURE_PERSUIT.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */

	
	public void handleIdleState(TeleopInput input) {

		if (input == null) {
			System.out.println("Autonomus");
		} else {
			System.out.println("Teleop");
		}

		rightMotorFront.set(0);
		rightMotorBack.set(0);
		leftMotorFront.set(0);
		leftMotorBack.set(0);
	}

	/**
	 * Handle behavior in PURE_PERSUIT.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 * @param x1 starting x position
	 * @param y1 starting y position
	 * @param mx middle x position
	 * @param my middle y position
	 * @param x2 ending x position
	 * @param y2 ending y position
	 */
	public void handlePurePursuit(TeleopInput input, double x1, double y1, double mx, double my, double x2, double y2) {

		if (input != null) {
			System.out.println("Teleop");
			rightMotorFront.set(0);
			rightMotorBack.set(0);
			leftMotorFront.set(0);
			leftMotorBack.set(0);
			return;			
		}

		double roboX = -roboXPos;
		double roboY = roboYPos;
		double currentAngle = (-gyro.getAngle()) % 360;
		System.out.println("x: " + roboX + " y: " + roboY);
		double innerVelocity = 10;
		
		if (firstRun) {
			calculateWaypoints(x1, y1, mx, my, x2, y2);
			firstRun = false;
		}

		if (roboX < x2 + Constants.AUTONOMUS_MOVE_THRESHOLD && roboX > x2 - Constants.AUTONOMUS_MOVE_THRESHOLD && roboY < y2 + Constants.AUTONOMUS_MOVE_THRESHOLD && roboY > y2 - Constants.AUTONOMUS_MOVE_THRESHOLD) { // within range of endpoint
			System.out.println("STOP");
			resetPurePursuitProperties();
		}

		if (target != findTargetPoint(roboX, roboY)) { // when there is a new target point

			pointNum++; // robot has advanced to a new point
			target = findTargetPoint(roboX, roboY);
			innerVelocity = calculateInnerCurveVelocity(currentAngle, roboX, roboY, waypoints[0][target], waypoints[1][target], Constants.OUTER_VELOCITY);
		}

		// set motor powers (note: velcoties must be between [-7.9, +7.9])
		if (direction == -1) { // turning left
			leftMotorFront.set(-innerVelocity / Constants.PURE_PURSUIT_VELOCITY_CONSTANT);
			leftMotorBack.set(-innerVelocity / Constants.PURE_PURSUIT_VELOCITY_CONSTANT);
			rightMotorFront.set(Constants.OUTER_VELOCITY / Constants.PURE_PURSUIT_VELOCITY_CONSTANT);
			rightMotorBack.set(Constants.OUTER_VELOCITY / Constants.PURE_PURSUIT_VELOCITY_CONSTANT);
		} else if (direction == 1) { // turning right
			leftMotorFront.set(-Constants.OUTER_VELOCITY / Constants.PURE_PURSUIT_VELOCITY_CONSTANT);
			leftMotorBack.set(-Constants.OUTER_VELOCITY / Constants.PURE_PURSUIT_VELOCITY_CONSTANT);
			rightMotorFront.set(innerVelocity / Constants.PURE_PURSUIT_VELOCITY_CONSTANT);
			rightMotorBack.set(innerVelocity / Constants.PURE_PURSUIT_VELOCITY_CONSTANT);
		}
	}

	/**
	 * Calculate waypoints for Pure Pursuit.
	 * @param x1 starting x position
	 * @param y1 starting y position
	 * @param mx middle x position
	 * @param my middle y position
	 * @param x2 ending x position
	 * @param y2 ending y position
	 */
	public void calculateWaypoints(double x1, double y1, double mx, double my, double x2, double y2) {
		double dx1 = 2 * (mx - x1) / Constants.PARTITIONS;
		double dy1 = 2 * (my - y1) / Constants.PARTITIONS;
		for (int i = 0; i <= Constants.PARTITIONS / 2; i++) {
			waypoints[0][i] = x1 + i * dx1;
			waypoints[1][i] = y1 + i * dy1;
		}
		double dx2 = 2 * (x2 - mx) / Constants.PARTITIONS;
		double dy2 = 2 * (y2 - my) / Constants.PARTITIONS;
		for (int i = Constants.PARTITIONS / 2 + 1; i <= Constants.PARTITIONS; i++) {
			waypoints[0][i] = mx + (i - Constants.PARTITIONS / 2) * dx2;
			waypoints[1][i] = my + (i - Constants.PARTITIONS / 2) * dy2;
		}
		System.out.println(Arrays.deepToString(waypoints));
	}

	/**
	 * Finds the target waypoint given the current position.
	 * @param x x position
	 * @param y y position
	 * @return target point index
	*/
	public int findTargetPoint(double x, double y) {
		double closestDist = Integer.MAX_VALUE;
		int target = -1; // index of target point
		for (int i = pointNum; i < waypoints[0].length; i++) {
			double dist = Math.hypot(waypoints[0][i] - x, waypoints[1][i] - y); // distance between current pos and potential target point
			if (Math.abs(Constants.LOOK_DISTANCE - dist) <= closestDist) {
				closestDist = Math.abs(Constants.LOOK_DISTANCE - dist);
				target = i;
			}
		}
		System.out.println(waypoints[0][target] + " " + waypoints[1][target]);
		return target;
	}

	/**
	 * Calculates velocity of inner wheel.
	 * @param startAngle starting angle
	 * @param x1 starting x position
	 * @param y1 starting y position
	 * @param x2 ending x position
	 * @param y2 ending y position
	 * @param outerVelocity velocity of outer wheel
	 * @return inner wheel velocity
	 */
	public double calculateInnerCurveVelocity(double startAngle, double x1, double y1, double x2, double y2, double outerVelocity) {
		double theta = Math.atan2(y2 - y1, x2 - x1) - Math.toRadians(startAngle);
		if (theta == 0) return outerVelocity; // innerVelocity = outerVelocity (going straight)
		double radius = (Math.tan(theta) + (1 / Math.tan(theta))) * Math.hypot(y2 - y1, x2 - x1) / 2;
		double arcRatio;
		if (radius > 0) {
			direction = -1; // left
			arcRatio = (radius - Constants.ROBOT_WIDTH) / (radius + Constants.ROBOT_WIDTH); // ratio of inner and outer arc lengths
		} else {
			direction = 1; // right
			arcRatio = (radius + Constants.ROBOT_WIDTH) / (radius - Constants.ROBOT_WIDTH);
		}
		double innerVelocity = outerVelocity * (arcRatio); // (ensures that inner velcoity must be <= outer velocity)
		System.out.println(innerVelocity);
		return innerVelocity;
	}

	/**
	 * Reset method for pure pursuit.
	 */
	public void resetPurePursuitProperties() {
		waypoints = new double[2][Constants.PARTITIONS + 1];
		target = -1;
		pointNum = 0;
		firstRun = true;
		stateCounter++;
	}


	/**
	 * Tracks the robo's position on the field.
	 * @param gyroAngle robot's angle
	 */
	public void updateLineOdometryTele(double gyroAngle) {

		double dEncoder = (currentEncoderPos - prevEncoderPos)
			/ Constants.REVOLUTIONS_PER_INCH;
		double dX = dEncoder * Math.cos(Math.toRadians(gyroAngleForOdo))
			* Constants.DX_INCHES_CONST;
		double dY = dEncoder * Math.sin(Math.toRadians(gyroAngleForOdo))
			* Constants.DY_INCHES_CONST;

		roboXPos += dX;
		roboYPos += dY;

		prevEncoderPos = this.currentEncoderPos;
	}
}
