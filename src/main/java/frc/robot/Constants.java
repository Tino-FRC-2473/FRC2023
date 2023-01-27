package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
	public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(20);
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
	public static final double ACCELERATION_CONSTANT_FOR_TURN = 2.8;
	public static final double SPEED_CONSTANT_FOR_TURN = 0.7;
	public static final double ONE_REVOLUTION_DEGREES = 360;
	public static final double HALF_REVOLUTION_DEGREES = 180;
	public static final double GYRO_TURN_MULTIPLER_BELOW_90 = 0.987;
	public static final double CHARGING_STATION_LEVELED_ERROR_DEGREES = 2;
	public static final double CHARGING_STATION_BALANCE_CONSTANT_PID_P = 200;


	// ODO Constants
	public static final double DX_INCHES_CONST = 0.8880486672;
	public static final double DY_INCHES_CONST = 1.1742067733;
	public static final double SHOOTING_CIRCLE_RADIUS = 120.000;
	public static final double HUB_X_COORDINATE = 40;
	public static final double HUB_Y_COORDINATE = 0;

	public static final double QUARTER_PI = Math.PI / 4;

	static class VisionConstants {
		static final double CAM_OFFSET_X_METERS = Units.inchesToMeters(0);
		static final double CAM_OFFSET_Y_METERS = Units.inchesToMeters(0);
		static final double CAM_HEIGHT_METERS = Units.inchesToMeters(21);
		static final double CAM_PITCH_RADIANS = Units.degreesToRadians(0);
		static final double CUBE_HEIGHT_METERS = Units.inchesToMeters(9.5);
		static final double CONE_HEIGHT_METERS = Units.inchesToMeters(12.8125);
	}
	static class AprilTagConstants {
		static final double APRILTAG_1_X_METERS = Units.inchesToMeters(0);
		static final double APRILTAG_1_Y_METERS = Units.inchesToMeters(0);
		static final double APRILTAG_1_HEIGHT_METERS = Units.inchesToMeters(17.5);
		static final double APRILTAG_1_ANGLE_RADIANS = Units.degreesToRadians(180);
	}
}
