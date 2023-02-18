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
	public static final double AUTONOMUS_MOVE_POWER = 0.05;
	public static final double AUTONOMUS_MOVE_THRESHOLD = 5;

	// Path points
	public static final double P1X1 = 28.271;
	public static final double P1X2 = -108.947;
	public static final double P1X3 = -53.342;
	public static final double P2X1 = 42.751;
	public static final double P2X2 = -47.7;
	public static final double P3X1 = -186.645;
	public static final double P3X2 = -9;
	public static final double P3X4 = -33.5;
	public static final double P3Y4 = 65.37;
	public static final double P3X6 = -81.047;
	public static final double P3Y6 = 65.37;
	public static final double P3A3 = 69.454 * 0.8807; // 69.454
	public static final double P3A5 = 9; // 0
	public static final double P4A1 = (360 - 6.235); // unchanged
	public static final double P4X2 = 146.122;
	public static final double P4Y2 = 15.768;
	public static final double P4A3 = 0; // back to 0
	public static final double P4X4 = 178.07;
	public static final double P4Y4 = 15.768;
	public static final double P4X5 = 146.122;
	public static final double P4Y5 = 15.768;
	public static final double P4A6 = (180 + 2.358); // unchanged
	public static final double P4Y7 = 21.449;
	public static final double P4A8 =  180;


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
		static final double CAM_HEIGHT_METERS = Units.inchesToMeters(20.5);
		static final double CAM_PITCH_RADIANS = Units.degreesToRadians(0);
		static final double CUBE_HEIGHT_METERS = Units.inchesToMeters(9.5);
		static final double CONE_HEIGHT_METERS = Units.inchesToMeters(12.8125);
		static final double HIGH_TAPE_HEIGHT_METERS = Units.inchesToMeters(43.75);
		static final double LOW_TAPE_HEIGHT_METERS = Units.inchesToMeters(24);
		static final double CAM_OFFSET_INCHES = 0;

	}
	static class AprilTagConstants {
		static final double APRILTAG_1_X_METERS = Units.inchesToMeters(610.77);
		static final double APRILTAG_1_Y_METERS = Units.inchesToMeters(42.19);
		static final double APRILTAG_1_HEIGHT_METERS = Units.inchesToMeters(17.5);
		static final double APRILTAG_1_ANGLE_RADIANS = Units.degreesToRadians(180);

		static final double APRILTAG_2_X_METERS = Units.inchesToMeters(0);
		static final double APRILTAG_2_Y_METERS = Units.inchesToMeters(0);
		static final double APRILTAG_2_HEIGHT_METERS = Units.inchesToMeters(17.5);
		static final double APRILTAG_2_ANGLE_RADIANS = Units.degreesToRadians(180);
	}
}
