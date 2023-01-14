package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.Joystick;

/**
 * Common class for providing driver inputs during Teleop.
 *
 * This class is the sole owner of WPILib input objects and is responsible for
 * polling input values. Systems may query TeleopInput via its getter methods
 * for inputs by value, but may not access the internal input objects.
 */
public class TeleopInput {
	/* ======================== Constants ======================== */
	private static final int LEFT_JOYSTICK_PORT = 0;
	private static final int RIGHT_JOYSTICK_PORT = 1;

	private static final int PIVOT_INCREASE_ID = 1;
	private static final int PIVOT_DECREASE_ID = 2;
	private static final int EXTEND_ID = 3;
	private static final int RETRACT_ID = 4;
	private static final int AIM_HIGH_ID = 5;
	private static final int AIM_MID_ID = 6;


	/* ======================== Private variables ======================== */
	// Input objects
	private Joystick leftJoystick;
	private Joystick rightJoystick;

	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		leftJoystick = new Joystick(LEFT_JOYSTICK_PORT);

		rightJoystick = new Joystick(RIGHT_JOYSTICK_PORT);
	}

	/* ======================== Public methods ======================== */
	// Getter methods for fetch input values should be defined here.
	// Method names should be descriptive of the behavior, so the
	// control mapping is hidden from other classes.

	/* ------------------------ Left Joystick ------------------------ */
	/**
	 * Get X axis of Left Joystick.
	 * @return Axis value
	 */
	public double getLeftJoystickX() {
		return leftJoystick.getX();
	}
	/**
	 * Get Y axis of Left Joystick.
	 * @return Axis value
	 */
	public double getLeftJoystickY() {
		return leftJoystick.getY();
	}
	/**
	 * Get the value of the Pivot Increase button.
	 * @return True if button is pressed
	 */
	public boolean isPivotIncreaseButtonPressed() {
		return leftJoystick.getRawButton(PIVOT_INCREASE_ID);
	}
	/**
	 * Get the value of the Pivot Decrease button.
	 * @return True if button is pressed
	 */
	public boolean isPivotDecreaseButtonPressed() {
		return leftJoystick.getRawButton(PIVOT_DECREASE_ID);
	}

	/**
	 * Get the value of the Extend button.
	 * @return True if button is pressed
	 */
	public boolean isExtendButtonPressed() {
		return leftJoystick.getRawButton(EXTEND_ID);
	}

	/**
	 * Get the value of the Retract button.
	 * @return True if button is pressed
	 */
	public boolean isRetractButtonPressed() {
		return leftJoystick.getRawButton(RETRACT_ID);
	}

	/**
	 * Get the value of the Shoot High button.
	 * @return True if button is pressed
	 */
	public boolean isShootHighButtonPressed() {
		return leftJoystick.getRawButton(AIM_HIGH_ID);
	}

	/**
	 * Get the value of the Shoot Mid button.
	 * @return True if button is pressed
	 */
	public boolean isShootMidButtonPressed() {
		return leftJoystick.getRawButton(AIM_MID_ID);
	}

	/* ------------------------ Right Joystick ------------------------ */
	/**
	 * Get X axis of Right Joystick.
	 * @return Axis value
	 */
	public double getRightJoystickX() {
		return rightJoystick.getX();
	}
	/**
	 * Get Y axis of Right Joystick.
	 * @return Axis value
	 */
	public double getRightJoystickY() {
		return rightJoystick.getY();
	}

	/* ======================== Private methods ======================== */

}
