package frc.robot.drive;

public class DrivePower {

	private double leftPower;
	private double rightPower;

	/**
	 * Constructor to create the DrivePower object.
	 * @param leftPower set the left power
	 * @param rightPower set the right power
	 */
	public DrivePower(double leftPower, double rightPower) {
		this.leftPower = leftPower;
		this.rightPower = rightPower;
	}

	/**
	 * Multiplies the powers by a constant.
	 * @param scalar constant to multiply to the powers
	 * @return the new powers
	 */
	public DrivePower scale(double scalar) {
		leftPower *= scalar;
		rightPower *= scalar;
		return this;
	}

	/**
	 * Returns the left power.
	 * @return the left power
	 */
	public double getLeftPower() {
		return leftPower;
	}

	/**
	 * Returns the right power.
	 * @return the right power
	 */
	public double getRightPower() {
		return rightPower;
	}

	/**
	 * Sets the left power to the specified value.
	 * @param leftPower new power to which the left power will be set
	 */
	public void setLeftPower(double leftPower) {
		this.leftPower = leftPower;
	}

	/**
	 * Sets the right power to the specified value.
	 * @param rightPower new power to which the right power will be set
	 */
	public void setRightPower(double rightPower) {
		this.rightPower = rightPower;
	}
}
