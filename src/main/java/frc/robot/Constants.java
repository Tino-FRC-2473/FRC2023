package frc.robot;


public class Constants {
	//Drive Constants
	public static final double WHEEL_DIAMETER_INCHES = 4.0; //7.65
	public static final double MAX_POWER = 1;
	public static final double REDUCED_MAX_POWER = 0.5;
	public static final double TELEOP_MIN_TURN_POWER = 0.03;
	public static final double TELEOP_MIN_MOVE_POWER = 0.02;
	public static final double TELEOP_ACCELERATION_CONSTANT = 0.05;
	public static final double TELEOP_ACCELERATION_MIN = 0.1;
	public static final double TURNING_IN_PLACE_THRESHOLD = 0.05;
	public static final double ENCODER_CONSTANT = 1.0814;
	public static final double COUNTS_PER_MOTOR_REVOLUTION = 42;
	public static final double GEAR_RATIO = 8.0; //26.0 * 4.67 / 12.0;
	public static final double REVOLUTIONS_PER_INCH
		= GEAR_RATIO / (Math.PI * WHEEL_DIAMETER_INCHES * ENCODER_CONSTANT);
	public static final double MIN_TURN_POWER = 0.125;
	public static final double TURN_ERROR_THRESHOLD_DEGREE = 1.0;
	public static final double TURN_ERROR_POWER_RATIO = 360;
	public static final double TURN_POWER = 0.05;
	public static final double MOVE_POWER = 0.1;
	public static final double TURN_THRESHOLD = 5;
	public static final double MOVE_THRESHOLD = 1.5;
	public static final double ROBOT_WIDTH = 20;
	public static final double PURE_PURSUIT_VELOCITY_CONSTANT = 7.95867322835;
	public static final int PARTITIONS = 8;
	public static final int LOOK_DISTANCE = 10;
	public static final double OUTER_VELOCITY = 10;
	public static final double HALF_CIRCLE = 180;
	public static final double FULL_CIRCLE = 360;
	public static final double QUARTER_CIRCLE = 90;
	public static final double THREE_QUARTER_CIRCLE = 270;
	public static final double P1X1 = 30.8;
	public static final double P1X2 = -105.6;
	public static final double P1X3 = -50.1;
	public static final double P2X1 = 49.4;
	public static final double P2X2 = -47.7;
	public static final double P3TURN_AMT = 63.9;

	// PFC Constants
	public static final double DX_INCHES_CONST = 0.8880486672;
	public static final double DY_INCHES_CONST = 1.1742067733;
	public static final double SHOOTING_CIRCLE_RADIUS = 120.000;
	public static final double HUB_X_COORDINATE = 40;
	public static final double HUB_Y_COORDINATE = 0;

	public static final double QUARTER_PI = Math.PI / 4;


}