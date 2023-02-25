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

	//arm robot constants
	public static final int CAN_ID_SPARK_PIVOT = 35;
	public static final int CAN_ID_SPARK_TELEARM = 13;
	//ground mount constants
	public static final int CAN_ID_GROUND_MOUNT = 8;

	// Place jumper from DIO pin 9 to GND to indicate this is a test setup
	private static final int DIO_TEST_SETUP_CHANNEL_ARM = 8;
	private static final int DIO_TEST_SETUP_CHANNEL_GRABBER = 9;
	private static final int DIO_TEST_SETUP_CHANNEL_GROUND_MOUNT = 7;
	private static DigitalInput testBoardPinArm =
		new DigitalInput(HardwareMap.DIO_TEST_SETUP_CHANNEL_ARM);
	private static DigitalInput testBoardPinGrabber =
		new DigitalInput(HardwareMap.DIO_TEST_SETUP_CHANNEL_GRABBER);
	private static DigitalInput testBoardPinGroundMount =
		new DigitalInput(HardwareMap.DIO_TEST_SETUP_CHANNEL_GROUND_MOUNT);

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
}
