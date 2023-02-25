// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;
// Systems
// import frc.robot.systems.ArmFSM;
import frc.robot.systems.DriveFSMSystem;
// import frc.robot.systems.SpinningIntakeFSM;
// import frc.robot.systems.GroundMountFSM;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
	private TeleopInput input;

	// Systems
	// private ArmFSM armSystem;
	private DriveFSMSystem driveSystem;
	// private SpinningIntakeFSM spinningIntakeFSM;
	// private GroundMountFSM groundMountFSM;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");
		input = new TeleopInput();

		// Instantiate all systems here
		// if (!HardwareMap.isTestBoardArm() && !HardwareMap.isTestBoardGrabber()
		// 	&& !HardwareMap.isTestBoardGroundMount()) {
		driveSystem = new DriveFSMSystem();
			// armSystem = new ArmFSM();
			// spinningIntakeFSM = new SpinningIntakeFSM();
		// }
		// if (HardwareMap.isTestBoardArm()) {
		// 	armSystem = new ArmFSM();
		// }

		// if (HardwareMap.isTestBoardGrabber()) {
		// 	spinningIntakeFSM = new SpinningIntakeFSM();
		// }

		// if (HardwareMap.isTestBoardGroundMount()) {
		// 	groundMountFSM = new GroundMountFSM();
		// }
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
		// if (!HardwareMap.isTestBoardArm() && !HardwareMap.isTestBoardGrabber()
		// 	&& !HardwareMap.isTestBoardGroundMount()) {
		// 	// armSystem.reset();
		driveSystem.resetAutonomous();
			// spinningIntakeFSM.reset();
		//}
		// if (HardwareMap.isTestBoardArm()) {
		// 	armSystem.reset();
		// }
		// if (HardwareMap.isTestBoardGrabber()) {
		// 	spinningIntakeFSM.reset();
		// }
		// if (HardwareMap.isTestBoardGroundMount()) {
		// 	groundMountFSM.reset();
		// }
	}

	@Override
	public void autonomousPeriodic() {
		// if (!HardwareMap.isTestBoardArm() && !HardwareMap.isTestBoardGrabber()
		// 	&& !HardwareMap.isTestBoardGroundMount()) {
			// armSystem.update(null);
		driveSystem.update(null);
			// spinningIntakeFSM.update(null);
		//}
		// if (HardwareMap.isTestBoardArm()) {
		// 	armSystem.update(null);
		// }
		// if (HardwareMap.isTestBoardGrabber()) {
		// 	spinningIntakeFSM.update(null);
		// }
		// if (HardwareMap.isTestBoardGroundMount()) {
		// 	groundMountFSM.update(null);
		// }
	}

	@Override
	public void teleopInit() {
		// System.out.println("-------- Teleop Init --------");
		// if (!HardwareMap.isTestBoardArm() && !HardwareMap.isTestBoardGrabber()
		// 	&& !HardwareMap.isTestBoardGroundMount()) {
			// armSystem.reset();
		driveSystem.resetTeleop();
			// spinningIntakeFSM.reset();
		//}
		// if (HardwareMap.isTestBoardArm()) {
		// 	armSystem.reset();
		// }
		// if (HardwareMap.isTestBoardGrabber()) {
		// 	spinningIntakeFSM.reset();
		// }
		// if (HardwareMap.isTestBoardGroundMount()) {
		// 	groundMountFSM.reset();
		// }
	}

	@Override
	public void teleopPeriodic() {
		// if (!HardwareMap.isTestBoardArm() && !HardwareMap.isTestBoardGrabber()
		// 	&& !HardwareMap.isTestBoardGroundMount()) {
			// armSystem.update(input);
		driveSystem.update(input);
			// spinningIntakeFSM.update(input);
		//}
		// if (HardwareMap.isTestBoardArm()) {
		// 	armSystem.update(input);
		// }
		// if (HardwareMap.isTestBoardGrabber()) {
		// 	spinningIntakeFSM.update(input);
		// }
		// if (HardwareMap.isTestBoardGroundMount()) {
		// 	groundMountFSM.update(input);
		// }
	}

	@Override
	public void disabledInit() {
		System.out.println("-------- Disabled Init --------");
	}

	@Override
	public void disabledPeriodic() {

	}

	@Override
	public void testInit() {
		System.out.println("-------- Test Init --------");
	}

	@Override
	public void testPeriodic() {

	}

	/* Simulation mode handlers, only used for simulation testing  */
	@Override
	public void simulationInit() {
		System.out.println("-------- Simulation Init --------");
	}

	@Override
	public void simulationPeriodic() { }

	// Do not use robotPeriodic. Use mode specific periodic methods instead.
	@Override
	public void robotPeriodic() { }
}
