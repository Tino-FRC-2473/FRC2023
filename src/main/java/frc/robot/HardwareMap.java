package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * HardwareMap provides a centralized spot for constants related to the hardware
 * configuration of the robot.
 */
public final class HardwareMap {
	// ID numbers for devices on the CAN bus
	public static final int CAN_ID_SPARK_PIVOT = 32;
	public static final int CAN_ID_SPARK_TELEARM = 5;
	public static final int CAN_ID_SPARK_DRIVE_LEFT1 = 31;
	public static final int CAN_ID_SPARK_DRIVE_LEFT2 = 32;
	public static final int CAN_ID_SPARK_DRIVE_RIGHT1 = 33;
	public static final int CAN_ID_SPARK_DRIVE_RIGHT2 = 34;


	//grabber testing constants
	public static final int CAN_ID_SPINNER_MOTOR = 5;
	public static final int ANALOGIO_ID_DISTANCE_SENSOR = 5;
	public static final int DIO_ID_BREAK_BEAM = 5;

	// Place jumper from DIO pin 9 to GND to indicate this is a test setup
	private static final int DIO_TEST_SETUP_CHANNEL = 9;
	private static DigitalInput testBoardPin = new DigitalInput(HardwareMap.DIO_TEST_SETUP_CHANNEL);
	/**
	 * Check if the current RoboRIO is part of a test setup or real robot.
	 * @return true if the current setup is a test setup
	 */
	public static boolean isTestBoard() {
		return !HardwareMap.testBoardPin.get();
	}
}
