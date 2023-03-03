// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;
// Systems
import frc.robot.systems.ArmFSM;
import frc.robot.systems.DriveFSMSystem;
import frc.robot.systems.SpinningIntakeFSM;
import frc.robot.systems.GroundMountFSM;

import frc.robot.systems.ArmFSM.ArmFSMState;
import frc.robot.systems.DriveFSMSystem.FSMState;
import frc.robot.systems.SpinningIntakeFSM.SpinningIntakeFSMState;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
	private TeleopInput input;

	// Systems
	private ArmFSM armSystem;
	private DriveFSMSystem driveSystem;
	private SpinningIntakeFSM spinningIntakeFSM;
	private GroundMountFSM groundMountFSM;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");
		input = new TeleopInput();

		driveSystem = new DriveFSMSystem();
		armSystem = new ArmFSM();
		spinningIntakeFSM = new SpinningIntakeFSM();

		// // Instantiate all systems here
		// if (!HardwareMap.isTestBoardArm() && !HardwareMap.isTestBoardGrabber()
		// 	&& !HardwareMap.isTestBoardGroundMount() && !HardwareMap.isTestBoardArmGrabber()) {
		// 	driveSystem = new DriveFSMSystem();
		// 	armSystem = new ArmFSM();
		// 	spinningIntakeFSM = new SpinningIntakeFSM();
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
		// if (HardwareMap.isTestBoardArmGrabber()) {
		// 	armSystem = new ArmFSM();
		// 	spinningIntakeFSM = new SpinningIntakeFSM();
		// }
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");

		armSystem.reset();
		driveSystem.resetAutonomous();
		spinningIntakeFSM.reset();
		// if (!HardwareMap.isTestBoardArm() && !HardwareMap.isTestBoardGrabber()
		// 	&& !HardwareMap.isTestBoardGroundMount() && !HardwareMap.isTestBoardArmGrabber()) {
		// 	armSystem.reset();
		// 	driveSystem.resetAutonomous();
		// 	spinningIntakeFSM.reset();
		// }
		// if (HardwareMap.isTestBoardArm()) {
		// 	armSystem.reset();
		// }
		// if (HardwareMap.isTestBoardGrabber()) {
		// 	spinningIntakeFSM.reset();
		// }
		// if (HardwareMap.isTestBoardGroundMount()) {
		// 	groundMountFSM.reset();
		// }
		// if (HardwareMap.isTestBoardArmGrabber()) {
		// 	armSystem.reset();
		// 	spinningIntakeFSM.reset();
		// }
	}

	@Override
	public void autonomousPeriodic() {

		// move the arm to lower state to push in cube
		if (driveSystem.getCurrentState() == (FSMState.P1N1)
			|| driveSystem.getCurrentState() == (FSMState.P2N1)) {
			armSystem.updateAuto(ArmFSMState.SHOOT_LOW_FORWARD);
		// shoot out the cube, then set the arm to idle state and stop the spinning intake
		} else if (driveSystem.getCurrentState() == (FSMState.P1N2)
			|| driveSystem.getCurrentState() == (FSMState.P2N2)) {
			spinningIntakeFSM.updateAutonomous(SpinningIntakeFSMState.RELEASE);
			armSystem.updateAuto(ArmFSMState.IDLE);
			spinningIntakeFSM.updateAutonomous(SpinningIntakeFSMState.IDLE_STOP);
		}

		// move the arm to shoot to the high node backwards
		if (driveSystem.getCurrentState() == (FSMState.P3N1)) {
			armSystem.updateAuto(ArmFSMState.SHOOT_HIGH_BACKWARD);
		// shoot the cube, then make the arm go to the lower state to pick up
		// another game element
		} else if (driveSystem.getCurrentState() == (FSMState.P3N2)) {
			spinningIntakeFSM.updateAutonomous(SpinningIntakeFSMState.RELEASE);
			armSystem.updateAuto(ArmFSMState.SHOOT_LOW_FORWARD);
		// set the motors to intake another game element, move arm to shoot in mid
		// node backwards
		} else if (driveSystem.getCurrentState() == (FSMState.P3N3)) {
			spinningIntakeFSM.updateAutonomous(SpinningIntakeFSMState.START_STATE);
			armSystem.updateAuto(ArmFSMState.SHOOT_MID_BACKWARD);
		// shoot out the game element, then set arm to idle and stop motors
		} else if (driveSystem.getCurrentState() == (FSMState.P3N4)) {
			spinningIntakeFSM.updateAutonomous(SpinningIntakeFSMState.RELEASE);
			armSystem.updateAuto(ArmFSMState.IDLE);
			spinningIntakeFSM.updateAutonomous(SpinningIntakeFSMState.IDLE_STOP);
		}
		// if (!HardwareMap.isTestBoardArm() && !HardwareMap.isTestBoardGrabber()
		// 	&& !HardwareMap.isTestBoardGroundMount() && !HardwareMap.isTestBoardArmGrabber()) {
		// 	armSystem.update(null);
		// 	driveSystem.update(null);
		// 	spinningIntakeFSM.update(null);
		// }
		// if (HardwareMap.isTestBoardArm()) {
		// 	armSystem.update(null);
		// }
		// if (HardwareMap.isTestBoardGrabber()) {
		// 	spinningIntakeFSM.update(null);
		// }
		// if (HardwareMap.isTestBoardGroundMount()) {
		// 	groundMountFSM.update(null);
		// }
		// if (HardwareMap.isTestBoardArmGrabber()) {
		// 	armSystem.update(null);
		// 	spinningIntakeFSM.update(null);
		// }
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");

		armSystem.reset();
		driveSystem.resetTeleop();
		spinningIntakeFSM.reset();
		// if (!HardwareMap.isTestBoardArm() && !HardwareMap.isTestBoardGrabber()
		// 	&& !HardwareMap.isTestBoardGroundMount() && !HardwareMap.isTestBoardArmGrabber()) {
		// 	armSystem.reset();
		// 	driveSystem.resetTeleop();
		// 	spinningIntakeFSM.reset();
		// }
		// if (HardwareMap.isTestBoardArm()) {
		// 	armSystem.reset();
		// }
		// if (HardwareMap.isTestBoardGrabber()) {
		// 	spinningIntakeFSM.reset();
		// }
		// if (HardwareMap.isTestBoardGroundMount()) {
		// 	groundMountFSM.reset();
		// }
		// if (HardwareMap.isTestBoardArmGrabber()) {
		// 	armSystem.reset();
		// 	spinningIntakeFSM.reset();
		// }
	}


	@Override
	public void teleopPeriodic() {

		armSystem.update(input);
		driveSystem.update(input);
		spinningIntakeFSM.update(input);
		// if (!HardwareMap.isTestBoardArm() && !HardwareMap.isTestBoardGrabber()
		// 	&& !HardwareMap.isTestBoardGroundMount() && !HardwareMap.isTestBoardArmGrabber()) {
		// 	armSystem.update(input);
		// 	driveSystem.update(input);
		// 	spinningIntakeFSM.update(input);
		// }
		// if (HardwareMap.isTestBoardArm()) {
		// 	armSystem.update(input);
		// }
		// if (HardwareMap.isTestBoardGrabber()) {
		// 	spinningIntakeFSM.update(input);
		// }
		// if (HardwareMap.isTestBoardGroundMount()) {
		// 	groundMountFSM.update(input);
		// }
		// if (HardwareMap.isTestBoardArmGrabber()) {
		// 	armSystem.update(input);
		// 	spinningIntakeFSM.update(input);
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
