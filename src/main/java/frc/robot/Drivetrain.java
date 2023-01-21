package frc.robot;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;


public class Drivetrain {
	// These represent our regular encoder objects, which we would
	// create to use on a real robot.
	private Encoder m_leftEncoder = new Encoder(0, 1);
	private Encoder m_rightEncoder = new Encoder(2, 3);

	// These are our EncoderSim objects, which we will only use in
	// simulation. However, you do not need to comment out these
	// declarations when you are deploying code to the roboRIO.
	private EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
	private EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);
	// Create our gyro object like we would on a real robot.
	private AnalogGyro m_gyro = new AnalogGyro(1);

	// Create the simulated gyro object, used for setting the gyro
	// angle. Like EncoderSim, this does not need to be commented out
	// when deploying code to the roboRIO.
	private AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);

	private DifferentialDrivetrainSim m_driveSim = DifferentialDrivetrainSim.createKitbotSim(
		KitbotMotor.kDualCIMPerSide, // 2 CIMs per side.
		KitbotGearing.k10p71,        // 10.71:1
		KitbotWheelSize.kSixInch,    // 6" diameter wheels.
		null                         // No measurement noise.
	);

	private PWMSparkMax m_leftMotor = new PWMSparkMax(0);
	private PWMSparkMax m_rightMotor = new PWMSparkMax(1);

	private Field2d m_field = new Field2d();
	PIDController pidControl = new PIDController(7, 0, 0);

	public Drivetrain() {
		SmartDashboard.putData("Field", m_field);
		m_leftEncoder.setDistancePerPulse(2 * Math.PI * 2 * 42);
		m_rightEncoder.setDistancePerPulse(2 * Math.PI * 2 * 42);
	}

	public void simulationPeriodic(double leftSpeed, double rightSpeed) {
		// Set the inputs to the system. Note that we need to convert
		// the [-1, 1] PWM signal to voltage by multiplying it by the
		// robot controller voltage.
		m_driveSim.setInputs(leftSpeed * RobotController.getInputVoltage(),
							rightSpeed * RobotController.getInputVoltage());

		//m_driveSim.setInputs(leftSpeed,
		//                    rightSpeed);
		// Advance the model by 20 ms. Note that if you are running this
		// subsystem in a separate thread or have changed the nominal timestep
		// of TimedRobot, this value needs to match it.
		m_driveSim.update(0.02);

		// Update all of our sensors.
		m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
		m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
		m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
		m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
		m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());
		m_field.setRobotPose(getCurPose());
		System.out.println(getCurPose());
		SmartDashboard.putData("Field", m_field);
	}

	public Pose2d getCurPose() {
		return m_driveSim.getPose();
	}
}