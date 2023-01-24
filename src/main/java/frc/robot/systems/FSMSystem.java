package frc.robot.systems;

// Third party Hardware Imports
import edu.wpi.first.wpilibj.SPI;
import com.revrobotics.CANSparkMax;

import java.util.Arrays;

import com.kauailabs.navx.frc.AHRS;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
import frc.robot.Constants;

public class FSMSystem {


	// FSM state definitions
	public enum FSMState {
		path1,
		path2,
		path3,
		PURE_PERSUIT,
		P1,
		P2,
		P3,
		P4
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax leftMotor;
	private CANSparkMax rightMotor;
	private CANSparkMax leftMotor2;
	private CANSparkMax rightMotor2;

	private double roboXPos = 0;
	private double roboYPos = 0;
	private double currentEncoderPos;
	private double prevEncoderPos = 0;
	private double gyroAngleForOdo = 0;
	private AHRS gyro;

	//pure pursuit
	private int stateCounter = 1;
	private int partitions = 12; // should be even
	private double[][] waypoints = new double[2][partitions + 1];
	private int target = -1;
	private double innerVelocity = 0.5; // in/s
	private double outerVelocity = 0.5; // in/s
	private int direction = 1;
	private int pointNum = 0;
	private double lookAheadDistance = 20;
	private boolean firstRun = true;

	//go to pos
	private boolean turning = true;
	private boolean moving = true;
	private boolean complete = false;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public FSMSystem() {
		// Perform hardware init
		rightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_RIGHT1,
										CANSparkMax.MotorType.kBrushless);
		rightMotor2 = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_RIGHT2,
										CANSparkMax.MotorType.kBrushless);
		leftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_LEFT1,
										CANSparkMax.MotorType.kBrushless);
		leftMotor2 = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_LEFT2,
										CANSparkMax.MotorType.kBrushless);
		gyro = new AHRS(SPI.Port.kMXP);

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

		rightMotor.getEncoder().setPosition(0);
		leftMotor.getEncoder().setPosition(0);

		gyro.reset();
		gyro.zeroYaw();
		gyroAngleForOdo = 0;

		currentState = FSMState.path3;

		roboXPos = 0;
		roboYPos = 0;
		System.out.println(roboXPos + " " + roboYPos);

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

		gyroAngleForOdo = gyro.getAngle();
		currentEncoderPos = ((leftMotor.getEncoder().getPosition()
				- rightMotor.getEncoder().getPosition()) / 2.0);
		updateLineOdometryTele(gyro.getAngle(), currentEncoderPos);

		// System.out.println("currentstate: " + currentState);
		// System.out.println("xPos " + roboXPos);
		// System.out.println("yPos: " + roboYPos);

		switch (currentState) {
			case path1:
				path1(input);
				break;

			case path2:
				path2(input);
				break;

			case path3:
				path3(input);
				break;

			case P1:
				goToPos(input, 30.876, 0);
				break;

			case P2:
				goToPos(input, -105.617, 0);
				break;

			case P3:
				goToPos(input, 56.814, 117.516);
				break;

			case P4:
				goToPos(input, 0, 0);
				break;	

			case PURE_PERSUIT:
				handlePurePursuit(input, 0, 0, 50, 0, 90, -30); 
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
			
			case path1:
				return FSMState.path1;

			case path2:
				return FSMState.path2;
			
			case path3:
				return FSMState.path3;

			case P1:
				if (stateCounter == 1) {
					return FSMState.P1;
				} else if (stateCounter == 2) {
					return FSMState.P2;
				} else if (stateCounter == 3) {
					return FSMState.P3;
				} else if (stateCounter == 4) {
					return FSMState.P4;
				}

			case P2:
				if (stateCounter == 1) {
					return FSMState.P1;
				} else if (stateCounter == 2) {
					return FSMState.P2;
				} else if (stateCounter == 3) {
					return FSMState.P3;
				} else if (stateCounter == 4) {
					return FSMState.P4;
			}

			case PURE_PERSUIT:
				if (stateCounter == 0) {
					return FSMState.PURE_PERSUIT;
				}

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */


	/**
	 * Handle behavior in PURE_PERSUIT.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void goToPos(TeleopInput input, double x, double y) {
		if (input != null) {
			return;
		}

		System.out.println("t " + turning + " m " + moving);

		double roboX = -roboXPos;
		double roboY = roboYPos;
		double deltaX = (x - roboX);
		double deltaY = (y - roboY);
		if (deltaX > -1 && deltaX < 1) deltaX = 0;
		if (deltaY > -1 && deltaY < 1) deltaY = 0;
		System.out.println("dx " + deltaX + " dy " + deltaY);

		// assume unit circle angles (east = 0, positive counterclockwise)
		double currentAngle = (-gyro.getAngle()) % 360;
		System.out.println("current angle " + currentAngle);

		// calculates turn angle
		double angle;
		if (deltaX == 0 && deltaY >= 0) {
			angle = 90;
		} else if (deltaX == 0 && deltaY < 0) {
			angle = 270;
		} else {
			angle = Math.toDegrees(Math.atan(deltaY / deltaX));
		}

		if (deltaX < 0) {
			angle += 180;
		}
		if (deltaX > 0 && deltaY < 0) {
			angle += 360;
		}

		System.out.println("turn angle " + angle);

		// calculate turn amount
		double turnAmount = angle - currentAngle;

		if (Math.abs(turnAmount - 360) < Math.abs(turnAmount)) {
			turnAmount -= 360;
		}

		System.out.println("turn amount: " + turnAmount);

		// calculates distance
		double dist = Math.sqrt(deltaY * deltaY + deltaX * deltaX);
		System.out.println("dist: " + dist);
		System.out.println("curX: " + roboX + " curY: " + roboYPos);

		// set motor power
		if ((turnAmount < -Constants.TURN_THRESHOLD || turnAmount > Constants.TURN_THRESHOLD) && !complete && turning) {
			System.out.println("turning");
			if (turnAmount > 0) {
				leftMotor.set(Constants.TURN_POWER);
				rightMotor.set(Constants.TURN_POWER);
			} else if (turnAmount < 0) {
				leftMotor.set(-Constants.TURN_POWER);
				rightMotor.set(-Constants.TURN_POWER);
			}
		} else  if (dist > Constants.MOVE_THRESHOLD && !complete && moving) { 
			System.out.println("moving");
			leftMotor.set(-Constants.MOVE_POWER);
			rightMotor.set(Constants.MOVE_POWER);
		} else if (!complete) {
			leftMotor.set(0);
			rightMotor.set(0);
			System.out.println("STOP");
			complete = true;
			turning = true;
			moving = true;
			stateCounter++;
		}

		// complete or not
		if (((Math.abs(roboY) > Math.abs(y) + Math.abs(deltaX) / 2 || Math.abs(roboX) > Math.abs(x) + Math.abs(deltaY) / 2)) && !complete) {
			moving = false;
			//System.out.println("HERE");
		} else if (!(turnAmount >= -Constants.TURN_THRESHOLD && turnAmount <= Constants.TURN_THRESHOLD)) {
			complete = false;
		} else if (dist > Constants.MOVE_THRESHOLD) {
			complete = false;
			turning = false;
		} else {
			complete = true;
			turning = true;
			moving = true;
		}
	}
	
	/**
	 * Handle behavior in PURE_PERSUIT.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void handlePurePursuit(TeleopInput input, double x1, double y1, double mx, double my, double x2, double y2) {

		if (input != null) {
			System.out.println("Teleop");
			leftMotor.set(0);
			leftMotor2.set(0);
			rightMotor.set(0);
			rightMotor2.set(0);
			return;			
		}

		double roboX = -roboXPos;
		double roboY = roboYPos;
		double currentAngle = (-gyro.getAngle()) % 360;
		System.out.println("x: " + roboX + " y: " + roboY);
		
		if (firstRun) {
			calculateWaypoints(x1, y1, mx, my, x2, y2);
			firstRun = false;
		}

		if (roboX < x2 + 4 && roboX > x2 - 4 && roboY < y2 + 4 && roboY > y2 - 4) { //within range of endpoint
			System.out.println("STOP");
			resetPurePursuitProperties(8, 10, 0);
		}

		if (target != findTargetPoint(roboX, roboY)) { // when there is a new target point

			pointNum++; // robot has advanced to a new point
			target = findTargetPoint(roboX, roboY);
			innerVelocity = calculateInnerCurveVelocity(currentAngle, roboX, roboY, waypoints[0][target], waypoints[1][target], outerVelocity);
		}

		// set motor powers (note: velcoties must be between [-7.9, +7.9])
		if (direction == -1) { // turning left
			leftMotor.set(-innerVelocity / Constants.PURE_PURSUIT_VELOCITY_CONSTANT);
			leftMotor2.set(-innerVelocity / Constants.PURE_PURSUIT_VELOCITY_CONSTANT);
			rightMotor.set(outerVelocity / Constants.PURE_PURSUIT_VELOCITY_CONSTANT);
			rightMotor2.set(outerVelocity / Constants.PURE_PURSUIT_VELOCITY_CONSTANT);
		} else if (direction == 1) { // turning right
			leftMotor.set(-outerVelocity / Constants.PURE_PURSUIT_VELOCITY_CONSTANT);
			leftMotor2.set(-outerVelocity / Constants.PURE_PURSUIT_VELOCITY_CONSTANT);
			rightMotor.set(innerVelocity / Constants.PURE_PURSUIT_VELOCITY_CONSTANT);
			rightMotor2.set(innerVelocity / Constants.PURE_PURSUIT_VELOCITY_CONSTANT);
		}
	}

	public void calculateWaypoints(double x1, double y1, double mx, double my, double x2, double y2) {
		double dx1 = 2 * (mx - x1) / partitions;
		double dy1 = 2 * (my - y1) / partitions;
		for (int i = 0; i <= partitions / 2; i++) {
			waypoints[0][i] = x1 + i * dx1;
			waypoints[1][i] = y1 + i * dy1;
		}
		double dx2 = 2 * (x2 - mx) / partitions;
		double dy2 = 2 * (y2 - my) / partitions;
		for (int i = partitions / 2 + 1; i <= partitions; i++) {
			waypoints[0][i] = mx + (i - partitions / 2) * dx2;
			waypoints[1][i] = my + (i - partitions / 2) * dy2;
		}
		System.out.println(Arrays.deepToString(waypoints));
	}

	public int findTargetPoint(double x, double y) {
		double closestDist = 1000000;
		int target = -1; // index of target point
		for (int i = pointNum; i < waypoints[0].length; i++) {
			double dist = Math.hypot(waypoints[0][i] - x, waypoints[1][i] - y); // distance between current pos and potential target point
			if (Math.abs(lookAheadDistance - dist) <= closestDist) {
				closestDist = Math.abs(lookAheadDistance - dist);
				target = i;
			}
		}
		System.out.println(waypoints[0][target] + " " + waypoints[1][target]);
		return target;
	}

	public double calculateInnerCurveVelocity(double startAngle, double x1, double y1, double x2, double y2, double outerVelocity) {
		double theta = Math.atan2(y2 - y1, x2 - x1) - Math.toRadians(startAngle);
		System.out.println("atan" + Math.toDegrees(Math.atan2(y2 - y1, x2 - x1)));
		System.out.println("theta:" + Math.toDegrees(theta));
		System.out.println("start angle:" + startAngle);
		if (theta == 0) return outerVelocity; // innerVelocity = outerVelocity (going straight)

		double radius = (Math.tan(theta) + (1 / Math.tan(theta))) * Math.hypot(y2 - y1, x2 - x1) / 2;
		System.out.println("radius: " + radius);
		double arcRatio;
		if (radius > 0) {
			direction = -1; //left
			arcRatio = (radius - Constants.ROBOT_WIDTH)/(radius + Constants.ROBOT_WIDTH); // ratio of inner and outer arc lengths
		} else {
			direction = 1; //right
			arcRatio = (radius + Constants.ROBOT_WIDTH)/(radius - Constants.ROBOT_WIDTH);
		}
		double innerVelocity = outerVelocity * (arcRatio); // (ensures that inner velcoity must be <= outer velocity)
		System.out.println(innerVelocity);
		return innerVelocity;
	}

	public void resetPurePursuitProperties(int partitions, double lookAheadDistance, double outerVelocity) {
		this.partitions = partitions; // should be even
		waypoints = new double[2][partitions + 1];
		this.lookAheadDistance = lookAheadDistance;
		target = -1;
		pointNum = 0;
		innerVelocity = outerVelocity;; // in/s
		this.outerVelocity = outerVelocity; // in/s
		firstRun = true;
		stateCounter++;
	}

	/**
	 * Tracks the robo's position on the field.
	 * @param gyroAngle robot's angle
	 * @param currentEncoderPos robot's current position
	 */
	public void updateLineOdometryTele(double gyroAngle, double currentEncoderPos) {

		// double currentEncoderPos = ((-leftEncoderPos + rightEncoderPos) / 2.0);
		double dEncoder = (currentEncoderPos - prevEncoderPos)
			/ Constants.REVOLUTIONS_PER_INCH;
		double dX = dEncoder * Math.cos(Math.toRadians(gyroAngleForOdo)) * 0.8880486672;
		double dY = dEncoder * Math.sin(Math.toRadians(gyroAngleForOdo)) * 1.1742067733;

		roboXPos += dX;
		roboYPos += dY;

		prevEncoderPos = this.currentEncoderPos;
		// return new Translation2d(robotPos.getX() + dX, robotPos.getY() + dY);
	}

	boolean finishedTurning = false;

	public void handleTurnState(TeleopInput input, double degrees) {
        if (input != null) {
            return;
        }
		finishedTurning = false;
        degrees *= 0.987;
        System.out.println(getHeading());
        double error = degrees - getHeading();
        if (error > 180) {
            error -= 360;
        }
        if (Math.abs(error) <= Constants.TURN_ERROR_THRESHOLD_DEGREE) {
            finishedTurning = true;
            leftMotor.set(0);
            rightMotor.set(0);
            return;
        }
        double power = Math.abs(error) / Constants.TURN_ERROR_POWER_RATIO;
        if (power < Constants.MIN_TURN_POWER) {
            power = Constants.MIN_TURN_POWER;
        }
        power *= (error < 0 && error > -180) ? -1 : 1;
        leftMotor.set(power);
        rightMotor.set(power);
    }

	double startAngle = 0;

	/**
    * Gets the heading from the gyro.
    * @return the gyro heading
    */
    public double getHeading() {
        // double angle = startAngle - gyro.getYaw();
        double angle = startAngle - gyro.getAngle();
        if (angle < 0) {
            angle += 360;
        }
        if (angle > 360) {
            angle -= 360;
        }
        return angle;
    }
	
	int state = 0;
		public void path1(TeleopInput input) {
		if (input != null) {
		    return;
		}
		double roboX = -roboXPos;
		double roboY = roboYPos;
		System.out.println("x: " + roboX);
		System.out.println("y: " + roboY);
		System.out.println("state: " + state);
		if (state == 0) {
			leftMotor.set(-Constants.MOVE_POWER);
			rightMotor.set(Constants.MOVE_POWER);
			if (Math.abs(roboX - 30.8) <= 5) {
				leftMotor.set(0);
				rightMotor.set(0);
				state++;
			}
		} else if (state == 1) {
			System.out.println("entered 1");
			leftMotor.set(Constants.MOVE_POWER);
			rightMotor.set(-Constants.MOVE_POWER);
			if (Math.abs(roboX + 105.6) <= 5) {
				leftMotor.set(0);
				rightMotor.set(0);
				state++;
			}
		} else if (state == 2) {
			leftMotor.set(-Constants.MOVE_POWER);
			rightMotor.set(Constants.MOVE_POWER);
			if (Math.abs(roboX + 50.1) <= 5) {
				leftMotor.set(0);
				rightMotor.set(0);
				System.out.println("end");
				//state++;
			}
		}
	}

	public void path2(TeleopInput input) {
		if (input != null) {
			return;
		}
		double roboX = -roboXPos;
		double roboY = roboYPos;
		System.out.println("x: " + roboX);
		System.out.println("y: " + roboY);
		//System.out.println("state: " + state);
		System.out.println("dist " + Math.abs(roboX - 49.4));
		if (state == 0) {
			leftMotor.set(-Constants.MOVE_POWER);
			rightMotor.set(Constants.MOVE_POWER);
			
			if (Math.abs(roboX - 49.4) <= 5) {
				leftMotor.set(0);
				rightMotor.set(0);
				state++;
			}
		} else if (state == 1) {
			System.out.println("entered 1");
			leftMotor.set(Constants.MOVE_POWER);
			rightMotor.set(-Constants.MOVE_POWER);
			System.out.println("dist " + Math.abs(roboX + 47.7));
			if (Math.abs(roboX + 47.7) <= 5) {
				leftMotor.set(0);
				rightMotor.set(0);
				System.out.println("end");
				//state++;
			}
		}
	}

	public void path3(TeleopInput input) {
		if (input != null) {
			return;
		}
		double roboX = -roboXPos;
		double roboY = roboYPos;
		System.out.println("x: " + roboX);
		System.out.println("y: " + roboY);
		System.out.println(gyro.getAngle());
	
		if (state == 0) {
			handleTurnState(input, (180 - 63.9));
			if (finishedTurning) state--;
		}
		else if (state == 0) {
			leftMotor.set(Constants.MOVE_POWER);
			rightMotor.set(-Constants.MOVE_POWER);		
			if (Math.abs(roboX + 191) <= 1.5) {
				leftMotor.set(0);
				rightMotor.set(0);
				state++;
			}
		} else if (state == 1) {
			System.out.println("entered 1");
			leftMotor.set(-Constants.MOVE_POWER);
			rightMotor.set(Constants.MOVE_POWER);
			if (Math.abs(roboX) <= 1.5) {
				leftMotor.set(0);
				rightMotor.set(0);
				state++;
			}
		} else if (state == 2) {
			System.out.println("back 3");
            		leftMotor.set(Constants.MOVE_POWER);
			    rightMotor.set(-Constants.MOVE_POWER);
			    if (Math.abs(roboX + 9) <= 1.5) {
				leftMotor.set(0);
				rightMotor.set(0);
				state++;
			    }
      		} else if (state == 3) { //turning right
            		handleTurnState(input, 243.9);
			if (finishedTurning) state++;
		} else if (state == 4) {
			leftMotor.set(Constants.MOVE_POWER);
			rightMotor.set(-Constants.MOVE_POWER);
			if (Math.abs(roboX + 31.5) <= 5 && Math.abs(roboY - 63.1) <= 5) {
				leftMotor.set(0);
				rightMotor.set(0);
				state++;
			}
		} else if (state == 5) { //turning left
			handleTurnState(input, 63.9);
			if (finishedTurning) state++;
		} else if (state == 6) {
			leftMotor.set(Constants.MOVE_POWER);
            rightMotor.set(-Constants.MOVE_POWER);
			if (Math.abs(roboX + 83.9) <= 5 && Math.abs(roboY - 66.1) <= 5) {
				leftMotor.set(0);
				rightMotor.set(0);
				//state++;
            }
		}
	}
}