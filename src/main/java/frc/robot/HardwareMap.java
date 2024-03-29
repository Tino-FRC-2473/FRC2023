package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * HardwareMap provides a centralized spot for constants related to the hardware
 * configuration of the robot.
 */
public final class HardwareMap {
	// ID numbers for drive motors
	public static final int CAN_ID_SPARK_DRIVE_LEFT_BACK = 31; // 1
	public static final int CAN_ID_SPARK_DRIVE_LEFT_FRONT = 32; // 2
	public static final int CAN_ID_SPARK_DRIVE_RIGHT_FRONT = 33; // 1
	public static final int CAN_ID_SPARK_DRIVE_RIGHT_BACK = 34; // 2

	//grabber constants
	public static final int CAN_ID_SPINNER_MOTOR = 40;
	public static final int ANALOGIO_ID_DISTANCE_SENSOR = 0;

	public static final int CAN_ID_SPINNER_MOTOR_GROUND_MOUNT = 45;

	//arm robot constants
	public static final int CAN_ID_SPARK_PIVOT = 5;
	public static final int CAN_ID_SPARK_TELEARM = 35;
	//ground mount constants
	public static final int CAN_ID_GROUND_MOUNT = 10;

	//Place jumper into DIO pin 0 on the robot in order to switch software from ground mount to arm
	private static final int DIO_ROBOT_GROUND_MOUNT = 0;
	private static DigitalInput robotPinGroundMount =
		new DigitalInput(HardwareMap.DIO_ROBOT_GROUND_MOUNT);

	// Place jumper from DIO pin 9 to GND to indicate this is a test setup
	private static final int DIO_TEST_SETUP_CHANNEL_ARM = 8;
	private static final int DIO_TEST_SETUP_CHANNEL_GRABBER = 9;
	private static final int DIO_TEST_SETUP_CHANNEL_GROUND_MOUNT = 7;
	private static final int DIO_TEST_SETUP_CHANNEL_ARM_GRABBER = 6;
	private static DigitalInput testBoardPinArm =
		new DigitalInput(HardwareMap.DIO_TEST_SETUP_CHANNEL_ARM);
	private static DigitalInput testBoardPinGrabber =
		new DigitalInput(HardwareMap.DIO_TEST_SETUP_CHANNEL_GRABBER);
	private static DigitalInput testBoardPinGroundMount =
		new DigitalInput(HardwareMap.DIO_TEST_SETUP_CHANNEL_GROUND_MOUNT);
	private static DigitalInput testBoardPinArmGrabber =
		new DigitalInput(HardwareMap.DIO_TEST_SETUP_CHANNEL_ARM_GRABBER);


	/**
	 * Check if the current RoboRIO is in the ground mount configuration or the arm configuration.
	 * @return true if the current setup is ground mount configuration
	 */
	public static boolean isRobotGroundMount() {
		return !HardwareMap.robotPinGroundMount.get();
	}
	/**
	 * Check if the current RoboRIO is part of a ground mount test setup or real robot.
	 * @return true if the current setup is a test setup
	 */
	public static boolean isTestBoardArm() {
		return !HardwareMap.testBoardPinArm.get();
	}
	/**
	 * Check if the current RoboRIO is part of a grabber test setup or real robot.
	 * @return true if the current setup is a test setup
	 */
	public static boolean isTestBoardGrabber() {
		return !HardwareMap.testBoardPinGrabber.get();
	}
	/**
	 * Check if the current RoboRIO is part of a ground mount test setup or real robot.
	 * @return true if the current setup is a test setup
	 */
	public static boolean isTestBoardGroundMount() {
		return !HardwareMap.testBoardPinGroundMount.get();
	}

	/**Check if the current RoboRIO.
	 * @return true if
	 */
	public static boolean isTestBoardArmGrabber() {
		return !HardwareMap.testBoardPinArmGrabber.get();
	}
}
