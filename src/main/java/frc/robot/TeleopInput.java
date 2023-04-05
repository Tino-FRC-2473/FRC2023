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

	// drive joystick
	private static final int BALANCE_BUTTON = 2;
	private static final int HOLD_BUTTON = 5;
	private static final int FINE_TUNING_BUTTON_DRIVE = 6;
	private static final int CV_ALIGN_BUTTON_LEFT_NODE = 4;
	private static final int CV_ALIGN_BUTTON_MIDDLE_NODE = 3;
	private static final int CV_ALIGN_BUTTON_TAG = 5;
	private static final int CV_ALIGN_BUTTON_CONE = 8;
	private static final int CV_ALIGN_BUTTON_CUBE = 9;
	private static final int CV_VISION_BUTTON = 7;

	// mech joystick

	private static final int PIVOT_INCREASE_BUTTON = 3;
	private static final int PIVOT_DECREASE_BUTTON = 5;
	private static final int FINE_TUNING_BUTTON = 12;
	private static final int HOMING_BUTTON = 10;
	private static final int DISABLE_UPDATE_BUTTON = 2;
	private static final int AIM_HIGH_BUTTON = 7;
	private static final int AIM_MID_BUTTON = 9;
	private static final int AIM_LOW_BUTTON = 11;
	private static final int SUBSTATION_PICKUP_BUTTON = 8;

	private static final int GROUND_MOUNT_BUTTON = 4;
	private static final int GROUND_MOUNT_BUTTON_SHOOT = 7;
	private static final int INTAKE_BUTTON = 6;

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
	 * Get the value of the Disable Intake button.
	 * @return True if the button is pressed
	 */
	public boolean isDisableUpdatedPressed() {
		return mechJoystick.getRawButtonPressed(DISABLE_UPDATE_BUTTON);
	}
	/**
	 * Get value of lower pivot button for ground mount.
	 * @return true if pressed
	 */
	public boolean isPivotButtonPressed() {
		return mechJoystick.getRawButton(GROUND_MOUNT_BUTTON);
	}
	/**
	 * Get value of lower pivot button for ground mount.
	 * @return true if pressed
	 */
	public boolean isGroundMountShootButtonPressed() {
		return mechJoystick.getRawButton(GROUND_MOUNT_BUTTON_SHOOT);
	}
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
	 * Get the value of the Shoot High button.
	 * @return True if button is pressed
	 */
	public boolean isGroundMountUpPressed() {
		return mechJoystick.getRawButton(AIM_HIGH_BUTTON);
	}

	/**
	 * Get the value of the Shoot Mid button.
	 * @return True if button is pressed
	 */
	public boolean isGroundMountMidPressed() {
		return mechJoystick.getRawButton(AIM_MID_BUTTON);
	}

	/**
	 * Get the value of the Shoot Low button.
	 * @return True if button is pressed
	 */
	public boolean isGroundMountLowPressed() {
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
		return mechJoystick.getThrottle() <= 0;
	}

	/**
	 * Get the value of the Homing button.
	 * @return True if button is pressed
	 */
	public boolean isHomingButtonPressed() {
		return mechJoystick.getRawButton(HOMING_BUTTON);
	}

	/**
	 * Get the value of the Fine tuning button.
	 * @return True if button is pressed
	 */
	public boolean isFineTuningButtonPressed() {
		return mechJoystick.getRawButtonPressed(FINE_TUNING_BUTTON);
	}

	/**
	 * Get the value of the Intake button.
	 * @return True if the button is pressed
	 */
	public boolean isIntakeButtonPressed() {
		return mechJoystick.getRawButtonPressed(INTAKE_BUTTON);
	}

	/**
	 * Get the value of the Disable Intake button.
	 * @return True if the button is pressed
	 */
	public boolean isToggleIntakeUpdatePressed() {
		return mechJoystick.getRawButtonPressed(DISABLE_UPDATE_BUTTON);
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
	 * Get if cv contour switch (8) is pressed.
	 * @return true if button is pressed
	 */
	public boolean isMechJoystickCVVisionButtonPressedRaw() {
		return driveJoystick.getRawButton(CV_VISION_BUTTON);
	}

	/**
	 * Get if cv align button (4) is pressed.
	 * @return true if button is pressed
	 */
	public boolean isDriveJoystickCVHighTapeButtonPressedRaw() {
		return driveJoystick.getRawButton(CV_ALIGN_BUTTON_LEFT_NODE);
	}

	/**
	 * Get if cv align button (3) is pressed.
	 * @return true if button is pressed
	 */
	public boolean isDriveJoystickCVLowTapeButtonPressedRaw() {
		return driveJoystick.getRawButton(CV_ALIGN_BUTTON_MIDDLE_NODE);
	}

	/**
	 * Get if cv align button (5) is pressed.
	 * @return true if button is pressed
	 */
	public boolean isDriveJoystickCVTagButtonPressedRaw() {
		return driveJoystick.getRawButton(CV_ALIGN_BUTTON_TAG);
	}

	/**
	 * Get if cv align button (8) is pressed.
	 * @return true if button is pressed
	 */
	public boolean isDriveJoystickCVConeButtonPressedRaw() {
		return driveJoystick.getRawButton(CV_ALIGN_BUTTON_CONE);
	}

	/**
	 * Get if cv align button (9) is pressed.
	 * @return true if button is pressed
	 */
	public boolean isDriveJoystickCVCubeButtonPressedRaw() {
		return driveJoystick.getRawButton(CV_ALIGN_BUTTON_CUBE);
	}


	/* ------------------------ Steering Wheel ------------------------ */
	/**
	 * Get Angle of the steering Wheel from -1 to 1.
	 * @return Angle
	 */
	public double getSteerAngle() {
		return steeringWheel.getX();
	}

	/**
	 * Get if hold button (5) is pressed.
	 * @return true if button is pressed
	 */
	public boolean isSteeringWheelHoldPressedRaw() {
		return steeringWheel.getRawButton(HOLD_BUTTON);
	}

	/**
	 * Get if fine tuning button (6) is pressed.
	 * @return true if button is pressed
	 */
	public boolean isSteeringWheelFineTuningPressedRaw() {
		return steeringWheel.getRawButton(FINE_TUNING_BUTTON_DRIVE);
	}

	/* ======================== Private methods ======================== */
}
