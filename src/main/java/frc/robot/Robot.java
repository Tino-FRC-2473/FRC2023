// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;
// Systems
import frc.robot.systems.DriveFSMSystem;
import frc.robot.systems.SpinningIntakeFSM;
import frc.robot.systems.GroundMountFSM;

import frc.robot.systems.DriveFSMSystem.FSMState;
import frc.robot.systems.SpinningIntakeFSM.SpinningIntakeFSMState;
import frc.robot.systems.GroundMountFSM.GroundMountFSMState;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.util.datalog.StringLogEntry;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
	private TeleopInput input;

	// Systems
	private DriveFSMSystem driveSystem;
	private SpinningIntakeFSM spinningIntakeFSM;
	private GroundMountFSM groundMountFSM;
	private boolean isDriveEnabled = true;
	private boolean isIntakeEnabled = true;
	private static StringLogEntry myStringLog;
	private boolean resetLogs = true;

	// autonomus
	private static boolean finishedDeposit = false;
	private static int node = -1; // -1 is none, 0 is low, 1, mid, 2 is high
	private AutoPathChooser autoPathChooser;

	/**
	 * get string log.
	 * @return string log
	 */
	public static StringLogEntry getStringLog() {
		return myStringLog;
	}

	/**
	 * This function that returns whether or not the robot has finished
	 * 	depositing the object in autonomus.
	 * @return whether or not the robot has finished depositing the object in autonomus
	 */
	public static boolean getFinishedDeposit() {
		return finishedDeposit;
	}

	/**
	 * This function that resets finishedDeposit to false when a new point begins.
	 */
	public static void resetFinishedDeposit() {
		finishedDeposit = false;
	}

	/**
	 * This function that sets node to high if 2 is entered, mid if 1 is entered,
	 * 	low if 0 is entered, and none if 0 is entered.
	 * @param level the node at which the robot will deposit an object in autonomus
	 */
	public static void setNode(int level) {
		node = level;
	}

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");
		DataLogManager.start();
		DriverStation.startDataLog(DataLogManager.getLog(), false);
		myStringLog = new StringLogEntry(DataLogManager.getLog(), "/my/string");
		input = new TeleopInput();
		autoPathChooser = new AutoPathChooser();
		driveSystem = new DriveFSMSystem();
		groundMountFSM = new GroundMountFSM();
		spinningIntakeFSM = new SpinningIntakeFSM();
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
		groundMountFSM.reset();
		driveSystem.resetAutonomous();
		spinningIntakeFSM.reset();
	}

	@Override
	public void autonomousPeriodic() {
		groundMountFSM.update(null);
		driveSystem.update(null);
		spinningIntakeFSM.update(null);

		if (driveSystem.getCurrentState() == (FSMState.P1N1)
			|| driveSystem.getCurrentState() == (FSMState.P2N1)
			|| driveSystem.getCurrentState() == (FSMState.P3N1)
			|| driveSystem.getCurrentState() == (FSMState.P4N1)) {

			if (node == 1) {
				if (groundMountFSM.updateAutonomous(GroundMountFSMState.AUTONOMOUS_MID)) {
					finishedDeposit =
							spinningIntakeFSM.updateAutonomous(SpinningIntakeFSMState.RELEASE);
				}
			}
			if (node == 0) {
				if (groundMountFSM.updateAutonomous(GroundMountFSMState.AUTONOMOUS_MID)) {
					finishedDeposit =
							spinningIntakeFSM.updateAutonomous(SpinningIntakeFSMState.RELEASE);
				}
			}
			if (node == -1) {
				finishedDeposit = true;
			}

		} else if (driveSystem.getCurrentState() == (FSMState.P5N1)
			|| driveSystem.getCurrentState() == (FSMState.P6N1)
			|| driveSystem.getCurrentState() == (FSMState.P7N1)) {

			finishedDeposit = true;

		} else if (finishedDeposit && (driveSystem.getCurrentState() == (FSMState.P1N2)
			|| driveSystem.getCurrentState() == (FSMState.P1N3)
			|| driveSystem.getCurrentState() == (FSMState.P2N2)
			|| driveSystem.getCurrentState() == (FSMState.P3N2)
			|| driveSystem.getCurrentState() == (FSMState.P5N2)
			|| driveSystem.getCurrentState() == (FSMState.P5N3)
			|| driveSystem.getCurrentState() == (FSMState.P6N2)
			|| driveSystem.getCurrentState() == (FSMState.P7N2)
			|| driveSystem.getCurrentState() == FSMState.IDLE)) {

			spinningIntakeFSM.updateAutonomous(SpinningIntakeFSMState.IDLE_STOP);
			groundMountFSM.updateAutonomous(GroundMountFSMState.AUTONOMOUS_UP);
		}
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");
		groundMountFSM.reset();
		driveSystem.resetTeleop();
		spinningIntakeFSM.reset();
	}

	@Override
	public void teleopPeriodic() {
		groundMountFSM.update(input);
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
