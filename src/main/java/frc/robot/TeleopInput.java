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
	private static final int MECH_JOYSTICK_PORT = 0;
	private static final int DRIVE_JOYSTICK_PORT = 1;

	private static final int RELEASE_BUTTON = 3;
	private static final int PIVOT_INCREASE_ID = 1;
	private static final int PIVOT_DECREASE_ID = 2;
	private static final int EXTEND_ID = 3;
	private static final int RETRACT_ID = 4;
	private static final int AIM_HIGH_ID = 5;
	private static final int AIM_MID_ID = 6;
	/* ======================== Private variables ======================== */
	// Input objects
	private Joystick mechJoystick;
	private Joystick driveJoystick;

	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		mechJoystick = new Joystick(MECH_JOYSTICK_PORT);
		driveJoystick = new Joystick(DRIVE_JOYSTICK_PORT);
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
	public double getmechJoystickX() {
		return mechJoystick.getX();
	}
	/**
	 * Get Y axis of Left Joystick.
	 * @return Axis value
	 */
	public double getmechJoystickY() {
		return mechJoystick.getY();
	}
	/**
	 * Get the value of the Pivot Increase button.
	 * @return True if button is pressed
	 */
	public boolean isPivotIncreaseButtonPressed() {
		return mechJoystick.getRawButton(PIVOT_INCREASE_ID);
	}
	/**
	 * Get the value of the Pivot Decrease button.
	 * @return True if button is pressed
	 */
	public boolean isPivotDecreaseButtonPressed() {
		return mechJoystick.getRawButton(PIVOT_DECREASE_ID);
	}

	/**
	 * Get the value of the Extend button.
	 * @return True if button is pressed
	 */
	public boolean isExtendButtonPressed() {
		return mechJoystick.getRawButton(EXTEND_ID);
	}

	/**
	 * Get the value of the Retract button.
	 * @return True if button is pressed
	 */
	public boolean isRetractButtonPressed() {
		return mechJoystick.getRawButton(RETRACT_ID);
	}

	/**
	 * Get the value of the Shoot High button.
	 * @return True if button is pressed
	 */
	public boolean isShootHighButtonPressed() {
		return mechJoystick.getRawButton(AIM_HIGH_ID);
	}

	/**
	 * Get the value of the Shoot Mid button.
	 * @return True if button is pressed
	 */
	public boolean isShootMidButtonPressed() {
		return mechJoystick.getRawButton(AIM_MID_ID);
	}

	/* ------------------------ Right Joystick ------------------------ */
	/**
	 * Get X axis of Right Joystick.
	 * @return Axis value
	 */
	public double getdriveJoystickX() {
		return driveJoystick.getX();
	}
	/**
	 * Get Y axis of Right Joystick.
	 * @return Axis value
	 */
	public double getdriveJoystickY() {
		return driveJoystick.getY();
	}
	/**
	 * Get Button to pick up cube.
	 * @return Axis value
	 */
	public boolean getCubeButton() {
		return mechJoystick.getRawButton(1);
	}
	/**
	 * Get Button to pick up cone.
	 * @return Axis value
	 */
	public boolean getConeButton() {
		return mechJoystick.getRawButton(2);
	}
	/**
	 * Get Button to release.
	 * @return Axis value
	 */
	public boolean getOpenButton() {
		return mechJoystick.getRawButton(RELEASE_BUTTON);
	}
	/* ======================== Private methods ======================== */

}
