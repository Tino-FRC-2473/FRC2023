// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.cameraserver.CameraServer;
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

	// autonomus
	private static boolean finishedDeposit = false;
	private static int node = -1; // -1 is none, 0 is low, 1, mid, 2 is high

	// /**
	//  * This function that returns whether or not the robot has finished
	//  * 	depositing the object in autonomus.
	//  * @return whether or not the robot has finished depositing the object in autonomus
	//  */
	// public static boolean getFinishedDeposit() {
	// 	return finishedDeposit;
	// }

	// /**
	//  * This function that resets finishedDeposit to false when a new point begins.
	//  */
	// public static void resetFinishedDeposit() {
	// 	finishedDeposit = false;
	// }

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");
		input = new TeleopInput();
		CameraServer.startAutomaticCapture();
		driveSystem = new DriveFSMSystem();
		armSystem = new ArmFSM();
		spinningIntakeFSM = new SpinningIntakeFSM();
		System.gc();
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");

		armSystem.reset();
		driveSystem.resetAutonomous();
		spinningIntakeFSM.reset();
	}

	@Override
	public void autonomousPeriodic() {

		armSystem.update(null);
		driveSystem.update(null);
		spinningIntakeFSM.update(null);

		if (driveSystem.getCurrentState() == (FSMState.P1N1)
			|| driveSystem.getCurrentState() == (FSMState.P2N1)
			|| driveSystem.getCurrentState() == (FSMState.P3N1)
			|| driveSystem.getCurrentState() == (FSMState.P3N1)) {


			if (node == 2) {
				if (armSystem.updateAuto(ArmFSMState.SHOOT_HIGH_FORWARD)) {
					finishedDeposit =
						spinningIntakeFSM.updateAutonomous(SpinningIntakeFSMState.RELEASE);
				}
			}
			if (node == 1) {
				if (armSystem.updateAuto(ArmFSMState.SHOOT_MID_FORWARD)) {
					finishedDeposit =
						spinningIntakeFSM.updateAutonomous(SpinningIntakeFSMState.RELEASE);
				}
			}
			if (node == 0) {
				// WRITE CODE TO LEAVE ARM IN HOMING POSITION
				if (armSystem.updateAuto(ArmFSMState.SHOOT_LOW_FORWARD)) {
					finishedDeposit =
						spinningIntakeFSM.updateAutonomous(SpinningIntakeFSMState.RELEASE);
				}
			}
			if (node == -1) {
				// WRITE CODE TO LEAVE ARM IN HOMING POSITION
				armSystem.updateAuto(ArmFSMState.HOMING_STATE);
			}
		} else if (driveSystem.getCurrentState() == (FSMState.P1N2)
			|| driveSystem.getCurrentState() == (FSMState.P2N2)
			|| driveSystem.getCurrentState() == (FSMState.P3N2)) {
			spinningIntakeFSM.updateAutonomous(SpinningIntakeFSMState.IDLE_STOP);
			armSystem.updateAuto(ArmFSMState.AUTONOMOUS_RETRACT);
		}

		// // move the arm to lower state to push in cube
		// if (driveSystem.getCurrentState() == (FSMState.P1N1)
		// 	|| driveSystem.getCurrentState() == (FSMState.P2N1)) {
		// 	boolean done = armSystem.updateAuto(ArmFSMState.SHOOT_HIGH_FORWARD);
		// 	boolean released = false;
		// 	if (done) {
		// 		released = spinningIntakeFSM.updateAutonomous(SpinningIntakeFSMState.RELEASE);
		// 	}
		// 	Constants.finishedDeposit = released;
		// // shoot out the cube, then set the arm to idle state and stop the spinning intake
		// } else if (driveSystem.getCurrentState() == (FSMState.P1N2)
		// 	|| driveSystem.getCurrentState() == (FSMState.P2N2)) {
		// 	spinningIntakeFSM.updateAutonomous(SpinningIntakeFSMState.IDLE_STOP);
		// 	armSystem.updateAuto(ArmFSMState.AUTONOMOUS_RETRACT);
		// }

		// // move the arm to shoot to the high node backwards
		// if (driveSystem.getCurrentState() == (FSMState.P3N1)) {
		// 	armSystem.updateAuto(ArmFSMState.SHOOT_HIGH_BACKWARD);
		// // shoot the cube, then make the arm go to the lower state to pick up
		// // another game element
		// } else if (driveSystem.getCurrentState() == (FSMState.P3N2)) {
		// 	spinningIntakeFSM.updateAutonomous(SpinningIntakeFSMState.RELEASE);
		// 	armSystem.updateAuto(ArmFSMState.SHOOT_LOW_FORWARD);
		// // set the motors to intake another game element, move arm to shoot in mid
		// // node backwards
		// } else if (driveSystem.getCurrentState() == (FSMState.P3N3)) {
		// 	spinningIntakeFSM.updateAutonomous(SpinningIntakeFSMState.START_STATE);
		// 	armSystem.updateAuto(ArmFSMState.SHOOT_MID_BACKWARD);
		// // shoot out the game element, then set arm to idle and stop motors
		// } else if (driveSystem.getCurrentState() == (FSMState.P3N4)) {
		// 	spinningIntakeFSM.updateAutonomous(SpinningIntakeFSMState.RELEASE);
		// 	armSystem.updateAuto(ArmFSMState.IDLE);
		// 	spinningIntakeFSM.updateAutonomous(SpinningIntakeFSMState.IDLE_STOP);
		// }
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");

		System.gc();
		armSystem.reset();
		driveSystem.resetTeleop();
		spinningIntakeFSM.reset();
	}


	@Override
	public void teleopPeriodic() {

		armSystem.update(input);
		driveSystem.update(input);
		spinningIntakeFSM.update(input);
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
