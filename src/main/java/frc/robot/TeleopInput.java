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
	 * Get the value of the shooter button.
	 * @return True if button is pressed
	 */
	public boolean isShooterButtonPressed() {
		return mechJoystick.getRawButton(1);
	}
	/**
	 * Get the value of the intake button.
	 * @return True if button is pressed
	 */
	public boolean isIntakeButtonPressed() {
		return mechJoystick.getRawButton(2);
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
<<<<<<< Updated upstream
	public boolean getCubeButton() {
		return mechJoystick.getRawButton(1);
=======
	public boolean isCubeButtonPressed() {
		return mechJoystick.getRawButton(CUBE_BUTTON);
>>>>>>> Stashed changes
	}
	/**
	 * Get Button to pick up cone.
	 * @return Axis value
	 */
<<<<<<< Updated upstream
	public boolean getConeButton() {
		return mechJoystick.getRawButton(2);
=======
	public boolean isConeButtonPressed() {
		return mechJoystick.getRawButton(CONE_BUTTON);
>>>>>>> Stashed changes
	}
	/**
	 * Get Button to release.
	 * @return Axis value
	 */
<<<<<<< Updated upstream
	public boolean getOpenButton() {
		return mechJoystick.getRawButton(RELEASE_BUTTON);
=======
	public boolean isOpenButtonPressed() {
		return mechJoystick.getRawButton(OPEN_BUTTON);
>>>>>>> Stashed changes
	}
	/* ======================== Private methods ======================== */

}
