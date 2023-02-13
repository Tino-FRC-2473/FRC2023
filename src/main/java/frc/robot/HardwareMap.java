package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * HardwareMap provides a centralized spot for constants related to the hardware
 * configuration of the robot.
 */
public final class HardwareMap {
	// ID numbers for devices on the CAN bus
	public static final int CAN_ID_SPARK_PIVOT = 5;
	public static final int CAN_ID_SPARK_TELEARM = 10;
	public static final int CAN_ID_SPARK_DRIVE_LEFT1 = 31;
	public static final int CAN_ID_SPARK_DRIVE_LEFT2 = 32;
	public static final int CAN_ID_SPARK_DRIVE_RIGHT1 = 33;
	public static final int CAN_ID_SPARK_DRIVE_RIGHT2 = 34;


	//grabber testing constants
	public static final int CAN_ID_SPINNER_MOTOR = 5;
	public static final int ANALOGIO_ID_DISTANCE_SENSOR = 0;

	// Place jumper from DIO pin 9 to GND to indicate this is a test setup
	private static final int DIO_TEST_SETUP_CHANNEL_ARM = 8;
	private static final int DIO_TEST_SETUP_CHANNEL_GRABBER = 9;
	private static DigitalInput testBoardPinArm =
		new DigitalInput(HardwareMap.DIO_TEST_SETUP_CHANNEL_ARM);
	private static DigitalInput testBoardPinGrabber =
		new DigitalInput(HardwareMap.DIO_TEST_SETUP_CHANNEL_GRABBER);

	/**
	 * Check if the current RoboRIO is part of a arm test setup or real robot.
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
}
