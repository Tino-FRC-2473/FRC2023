package frc.robot.drive;

import frc.robot.Constants;

public class DriveFunctions {

	/**
	 * Calculated the new motor powers by accelerating the current
	 * powers to the desire ones.
	 * @param targetPower the target powers for the motors to be at
	 * @param currentPower the current powers the motors are at
	 * @return the new motor powers
	 */
	public static DrivePower accelerate(DrivePower targetPower, DrivePower currentPower) {
		double dLeftPower = targetPower.getLeftPower() - currentPower.getLeftPower();
		double dRightPower = targetPower.getRightPower() - currentPower.getRightPower();
		double newLeftPower;
		double newRightPower;
		if (Math.abs(dLeftPower) > Constants.TELEOP_ACCELERATION_MIN) {
			newLeftPower = currentPower.getLeftPower()
				+ dLeftPower * Constants.TELEOP_ACCELERATION_CONSTANT;
		} else {
			newLeftPower = targetPower.getLeftPower();
		}
		if (Math.abs(dRightPower) > Constants.TELEOP_ACCELERATION_MIN) {
			newRightPower = currentPower.getRightPower()
				+ dRightPower * Constants.TELEOP_ACCELERATION_CONSTANT;
		} else {
			newRightPower = targetPower.getRightPower();
		}
		return new DrivePower(newLeftPower, newRightPower);
	}

	/**
	 * Calculated the new motor power by accelerating and decelerating the current
	 * powers to the desire ones.
	 * @param currentGyroAngle the current gyro angle
	 * @param total the desired angle
	 * @return the new power for turning
	 */
	public static double accelerateDecelerateTurn(double currentGyroAngle, double total) {

		double power = -Math.pow((Constants.ACCELERATION_CONSTANT_FOR_TURN
			* Math.pow(currentGyroAngle - total / 2.0, 2))
			/ (currentGyroAngle * currentGyroAngle), 2) + Constants.SPEED_CONSTANT_FOR_TURN;
		return power;
	}

	/**
	 * Calculates the adjusted power to set the motor given the joystick input.
	 * @param joystickInput the joystick input
	 * @return the adjusted motor power
	 */
	public static double calcForwardPower(double joystickInput) {
		return (1 - Math.cos(Math.PI * joystickInput / 2.0));
		// return 2 * joystickInput * joystickInput * joystickInput;
	}

	/**
	 * Calculates the adjusted steering power to set the motor
	 * given the steering wheel input.
	 * @param steeringInput the input from the steering wheel
	 * @return the adjusted steering power
	 */
	public static DrivePower calcSteeringPower(double steeringInput) {
		return new DrivePower((1 + steeringInput), (1 - steeringInput));
	}

	/**
	 * Determines if the robot should turn in place and calculates the
	 * resulting motor powers.
	 * @param joystickInput the input from the joystick
	 * @param steeringInput the input from the steering wheel
	 * @return the new motor powers
	 */
	public static DrivePower turnInPlace(double joystickInput, double steeringInput) {
		if (Math.abs(steeringInput) > Constants.TELEOP_MIN_TURN_POWER) {
			return new DrivePower(-steeringInput, -steeringInput);
		} else {
			return new DrivePower(0, 0);
		}
	}
}
