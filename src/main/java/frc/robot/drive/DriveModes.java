package frc.robot.drive;


public class DriveModes {

	/**
	* Calculate the left and right motor powers in an arcade drive setup.
	* @param joystickY joystick input
	* @param steerAngle steering wheel input from -1 to 1
	* @param currentLeftPower current power of the left motor
	* @param currentRightPower current power of the right motor
	* @param isDrivingForward true if the robot is currently in a
	* 							forward driving setup
	* @return return the powers to set the left and right motors
	*/
	public static DrivePower arcadeDrive(double joystickY, double steerAngle,
		double currentLeftPower, double currentRightPower, boolean isDrivingForward) {

		double adjustedInput = Functions.calcForwardPower(joystickY);
		DrivePower adjustedSteering = Functions.calcSteeringPower(steerAngle);

		if (joystickY < 0 && adjustedInput > 0) {
			adjustedInput *= -1;
		}

		double targetLeftPower = 0;
		double targetRightPower = 0;

		targetRightPower = -adjustedInput * adjustedSteering.getRightPower();
		targetLeftPower = adjustedInput * adjustedSteering.getLeftPower();

		//checks if the magnitude of the target powers is greater than 1
		if (Math.abs(targetLeftPower) > 1.0) {
			targetLeftPower /= Math.abs(targetLeftPower);
		}
		if (Math.abs(targetRightPower) > 1.0) {
			targetRightPower /= Math.abs(targetRightPower);
		}

		//reversible driving (currently set on buttons 5 and 6)
		if (!isDrivingForward) {
			targetLeftPower = -adjustedInput * adjustedSteering.getRightPower();
			targetRightPower = adjustedInput * adjustedSteering.getLeftPower();

			//checks if the magnitude of the target powers is greater than 1
			if (Math.abs(targetLeftPower) > 1.0) {
				targetLeftPower /= Math.abs(targetLeftPower);
			}
			if (Math.abs(targetRightPower) > 1.0) {
				targetRightPower /= Math.abs(targetRightPower);
			}
		}

		return new DrivePower(targetLeftPower, targetRightPower);
	}
}
