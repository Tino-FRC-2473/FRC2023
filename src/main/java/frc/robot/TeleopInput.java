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
	private static final int STEERING_WHEEL_PORT = 3;

	private static final int PIVOT_INCREASE_BUTTON = 5;
	private static final int PIVOT_DECREASE_BUTTON = 3;
	private static final int EXTEND_BUTTON = 6;
	private static final int RETRACT_BUTTON = 4;
	private static final int AIM_HIGH_BUTTON = 7;
	private static final int AIM_MID_BUTTON = 9;
	private static final int BALANCE_BUTTON = 2;
	private static final int AIM_LOW_BUTTON = 11;
	private static final int SUBSTATION_PICKUP_BUTTON = 8;
	private static final int HOMING_BUTTON = 10;
	private static final int CV_ALIGN_BUTTON_LEFT_NODE = 4;
	private static final int CV_ALIGN_BUTTON_MIDDLE_NODE = 3;
	private static final int CV_ALIGN_BUTTON_RIGHT_NODE = 5;

	/* ======================== Private variables ======================== */
	// Input objects
	private Joystick mechJoystick;
	private Joystick driveJoystick;
	private Joystick steeringWheel;

	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		mechJoystick = new Joystick(MECH_JOYSTICK_PORT);
		driveJoystick = new Joystick(DRIVE_JOYSTICK_PORT);
		steeringWheel = new Joystick(STEERING_WHEEL_PORT);
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
	 * Get the value of the throttle.
	 * @return True if button is pressed
	 */
	public double getThrottle() {
		return mechJoystick.getThrottle();
	}

	/**
	 * Get the value of the Pivot Increase button.
	 * @return True if button is pressed
	 */
	public boolean isPivotIncreaseButtonPressed() {
		return mechJoystick.getRawButton(PIVOT_INCREASE_BUTTON);
	}

	/**
	 * Get the value of the Pivot Decrease button.
	 * @return True if button is pressed
	 */
	public boolean isPivotDecreaseButtonPressed() {
		return mechJoystick.getRawButton(PIVOT_DECREASE_BUTTON);
	}

	/**
	 * Get the value of the Extend button.
	 * @return True if button is pressed
	 */
	public boolean isExtendButtonPressed() {
		return mechJoystick.getRawButton(EXTEND_BUTTON);
	}

	/**
	 * Get the value of the Retract button.
	 * @return True if button is pressed
	 */
	public boolean isRetractButtonPressed() {
		return mechJoystick.getRawButton(RETRACT_BUTTON);
	}

	/**
	 * Get the value of the Shoot High button.
	 * @return True if button is pressed
	 */
	public boolean isShootHighButtonPressed() {
		return mechJoystick.getRawButton(AIM_HIGH_BUTTON);
	}

	/**
	 * Get the value of the Shoot Mid button.
	 * @return True if button is pressed
	 */
	public boolean isShootMidButtonPressed() {
		return mechJoystick.getRawButton(AIM_MID_BUTTON);
	}

	/**
	 * Get the value of the Shoot Low button.
	 * @return True if button is pressed
	 */
	public boolean isShootLowButtonPressed() {
		return mechJoystick.getRawButton(AIM_LOW_BUTTON);
	}

	/**
	 * Get the value of the Substation Pickup button.
	 * @return True if button is pressed
	 */
	public boolean isSubstationPickupButtonPressed() {
		return mechJoystick.getRawButton(SUBSTATION_PICKUP_BUTTON);
	}

	/**
	 * Get the boolean value of the Throttle button.
	 * @return True if button is pressed
	 */
	public boolean isThrottleForward() {
		return mechJoystick.getThrottle() <= 0 ? false : true;
	}

	/**
	 * Get the value of the Homing button.
	 * @return True if button is pressed
	 */
	public boolean isHomingButtonPressed() {
		return mechJoystick.getRawButton(HOMING_BUTTON);
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
	 * Get the value of release button.
	 * @return True if button pressed
	 */
	public boolean isReleaseButtonPressed() {
		return mechJoystick.getTrigger();
	}
	/**
	 * Get trigger button is pressed of left joystick.
	 * @return Axis value
	 */
	public boolean isDriveJoystickTriggerPressedRaw() {
		return driveJoystick.getTrigger();
	}

	/**
	 * Get if balance button (2) is pressed.
	 * @return true if button is pressed
	 */
	public boolean isDriveJoystickEngageButtonPressedRaw() {
		return driveJoystick.getRawButton(BALANCE_BUTTON);
	}

	/**
	 * Get if cv align button (4) is pressed.
	 * @return true if button is pressed
	 */
	public boolean isDriveJoystickCVAlignLeftButtonPressedRaw() {
		return driveJoystick.getRawButton(CV_ALIGN_BUTTON_LEFT_NODE);
	}

	/**
	 * Get if cv align button (3) is pressed.
	 * @return true if button is pressed
	 */
	public boolean isDriveJoystickCVAlignMiddleButtonPressedRaw() {
		return driveJoystick.getRawButton(CV_ALIGN_BUTTON_MIDDLE_NODE);
	}

	/**
	 * Get if cv align button (5) is pressed.
	 * @return true if button is pressed
	 */
	public boolean isDriveJoystickCVAlignRightButtonPressedRaw() {
		return driveJoystick.getRawButton(CV_ALIGN_BUTTON_RIGHT_NODE);
	}

	/* ------------------------ Steering Wheel ------------------------ */
	/**
	 * Get Angle of the steering Wheel from -1 to 1.
	 * @return Angle
	 */
	public double getSteerAngle() {
		return steeringWheel.getX();
	}

	/* ======================== Private methods ======================== */
}
