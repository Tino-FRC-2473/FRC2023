package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * HardwareMap provides a centralized spot for constants related to the hardware
 * configuration of the robot.
 */

public final class HardwareMap {
	/** Create new HardwareMap object. */
	private HardwareMap() {

	}
	// ID numbers for devices on the CAN bus
	/** front right drive. */
	public static final int CAN_ID_SPARK_DRIVE_FRONT_RIGHT = 1;
	/** back right drive. */
	public static final int CAN_ID_SPARK_DRIVE_BACK_RIGHT = 2;
	/** front left drive. */
	public static final int CAN_ID_SPARK_DRIVE_FRONT_LEFT = 3;
	/** back left drive. */
	public static final int CAN_ID_SPARK_DRIVE_BACK_LEFT = 4;
	/** shooter. */
	public static final int CAN_ID_SPARK_SHOOTER = 5;

	// Pneumatics channel numbers
	/** intake cylinder forward. */
	public static final int PCM_CHANNEL_INTAKE_CYLINDER_FORWARD = 1;
	/** intake cylinder reverse. */
	public static final int PCM_CHANNEL_INTAKE_CYLINDER_REVERSE = 2;

	// Place jumper from DIO pin 9 to GND to indicate this is a test setup
	/** test setup channel. */
	private static final int DIO_TEST_SETUP_CHANNEL = 9;
	/** test board pin. */
	private static DigitalInput testBoardPin = new DigitalInput(
		HardwareMap.DIO_TEST_SETUP_CHANNEL);

	/**
	 * Check if the current RoboRIO is part of a test setup or real robot.
	 * @return true if the current setup is a test setup
	 */
	public static boolean isTestBoard() {
		return !HardwareMap.testBoardPin.get();
	}
}
