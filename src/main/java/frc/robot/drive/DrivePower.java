package frc.robot.drive;

public class DrivePower {

	private double leftPower;
	private double rightPower;

	/**
	 * Constructor to create the DrivePower object.
	 * @param leftPower1 set the left power
	 * @param rightPower1 set the right power
	 */
	public DrivePower(double leftPower1, double rightPower1) {
		leftPower = leftPower1;
		rightPower = rightPower1;
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
	 * @param leftPower1 new power to which the left power will be set
	 */
	public void setLeftPower(double leftPower1) {
		this.leftPower = leftPower1;
	}

	/**
	 * Sets the right power to the specified value.
	 * @param rightPower1 new power to which the right power will be set
	 */
	public void setRightPower(double rightPower1) {
		this.rightPower = rightPower1;
	}
}
