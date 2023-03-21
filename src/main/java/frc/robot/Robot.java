// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// Systems
import frc.robot.systems.ArmFSM;
import frc.robot.systems.DriveFSMSystem;
import frc.robot.systems.SpinningIntakeFSM;
import frc.robot.systems.GroundMountFSM;

import frc.robot.systems.ArmFSM.ArmFSMState;
import frc.robot.systems.DriveFSMSystem.FSMState;
import frc.robot.systems.SpinningIntakeFSM.SpinningIntakeFSMState;
import frc.robot.systems.GroundMountFSM.GroundMountFSMState;

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
	private static SendableChooser<FSMState> autoPathChooser;
	private static SendableChooser<Integer> nodeChooser;
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
	 * This function returns the chooser object for the auto path for the robot.
	 * @return The chooser object which contains information on which auto path was selected.
	 */
	public static SendableChooser<FSMState> getAutoChooser() {
		return autoPathChooser;
	}

	/**
	 * This function returns the chooser object for the node for the auto path.
	 * @return The chooser object which contains information on the node for the auto path.
	 */
	public static SendableChooser<Integer> getNodeChooser() {
		return nodeChooser;
	}
	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");
		input = new TeleopInput();
		SmartDashboard.putBoolean("Enable Drive", true);
		SmartDashboard.putBoolean("Enable Arm", true);
		SmartDashboard.putBoolean("Enable Intake", true);
		SmartDashboard.putBoolean("Enable Ground Mount", true);
		autoPathChooser = new SendableChooser<>();
		autoPathChooser.setDefaultOption("Path 1", FSMState.P1N1);
		autoPathChooser.addOption("Path 2", FSMState.P2N1);
		autoPathChooser.addOption("Path 3", FSMState.P3N1);
		autoPathChooser.addOption("Path 4", FSMState.P4N1);
		autoPathChooser.addOption("Path 5", FSMState.P5N1);
		autoPathChooser.addOption("Path 6", FSMState.P6N1);
		autoPathChooser.addOption("Path 7", FSMState.P7N1);
		SmartDashboard.putData("Auto Path", autoPathChooser);

		nodeChooser = new SendableChooser<>();
		nodeChooser.setDefaultOption("Low", 0);
		nodeChooser.addOption("Mid", 1);
		nodeChooser.addOption("High", 2);
		nodeChooser.addOption("None", -1);
		SmartDashboard.putData("Node", nodeChooser);
		driveSystem = new DriveFSMSystem();
		if (HardwareMap.isRobotGroundMount()) {
			groundMountFSM = new GroundMountFSM();
		} else {
			armSystem = new ArmFSM();
		}
		spinningIntakeFSM = new SpinningIntakeFSM();
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
		System.gc();
		if (HardwareMap.isRobotGroundMount()) {
			groundMountFSM .reset();
		} else {
			armSystem.reset();
		}
		driveSystem.resetAutonomous();
		spinningIntakeFSM.reset();
	}

	@Override
	public void autonomousPeriodic() {

		if (HardwareMap.isRobotGroundMount()) {
			groundMountFSM.update(null);
		} else {
			armSystem.update(null);
		}
		driveSystem.update(null);
		spinningIntakeFSM.update(null);

		System.out.println(finishedDeposit);

		if (driveSystem.getCurrentState() == (FSMState.P1N1)
			|| driveSystem.getCurrentState() == (FSMState.P2N1)
			|| driveSystem.getCurrentState() == (FSMState.P3N1)
			|| driveSystem.getCurrentState() == (FSMState.P4N1)) {

			if (HardwareMap.isRobotGroundMount()) {
				if (groundMountFSM.updateAutonomous(GroundMountFSMState.AUTONOMOUS_DOWN)) {
					finishedDeposit =
							spinningIntakeFSM.updateAutonomous(SpinningIntakeFSMState.RELEASE);
				}
			} else {
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
					if (armSystem.updateAuto(ArmFSMState.SHOOT_LOW_FORWARD)) {
						finishedDeposit =
							spinningIntakeFSM.updateAutonomous(SpinningIntakeFSMState.RELEASE);
					}
				}
				if (node == -1) {
					finishedDeposit = true;
				}
			}
		} else if (driveSystem.getCurrentState() == (FSMState.P5N1)
			|| driveSystem.getCurrentState() == (FSMState.P6N1)
			|| driveSystem.getCurrentState() == (FSMState.P7N1)) {

			if (HardwareMap.isRobotGroundMount()) {
				finishedDeposit = true;
			} else {
				if (node == 2) {
					if (armSystem.updateAuto(ArmFSMState.SHOOT_HIGH_BACKWARD)) {
						finishedDeposit =
							spinningIntakeFSM.updateAutonomous(SpinningIntakeFSMState.RELEASE);
					}
				}
				if (node == 1) {
					if (armSystem.updateAuto(ArmFSMState.SHOOT_MID_BACKWARD)) {
						finishedDeposit =
							spinningIntakeFSM.updateAutonomous(SpinningIntakeFSMState.RELEASE);
					}
				}
				if (node == -1) {
					finishedDeposit = true;
				}
			}
		} else if (driveSystem.getCurrentState() == (FSMState.P1N2)
			|| driveSystem.getCurrentState() == (FSMState.P1N3)
			|| driveSystem.getCurrentState() == (FSMState.P2N2)
			|| driveSystem.getCurrentState() == (FSMState.P3N2)
			|| driveSystem.getCurrentState() == (FSMState.P5N2)
			|| driveSystem.getCurrentState() == (FSMState.P5N3)
			|| driveSystem.getCurrentState() == (FSMState.P6N2)
			|| driveSystem.getCurrentState() == (FSMState.P7N2)) {

			spinningIntakeFSM.updateAutonomous(SpinningIntakeFSMState.IDLE_STOP);
			if (HardwareMap.isRobotGroundMount()) {
				groundMountFSM.updateAutonomous(GroundMountFSMState.AUTONOMOUS_UP);
			} else {
				if (armSystem.updateAuto(ArmFSMState.AUTONOMOUS_RETRACT)) {
					armSystem.updateAuto(ArmFSMState.MOVING_TO_START_STATE);
				}
			}
		}
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");

		if (HardwareMap.isRobotGroundMount()) {
			groundMountFSM.reset();
		} else {
			armSystem.reset();
		}
		driveSystem.resetTeleop();
		spinningIntakeFSM.reset();
	}


	@Override
	public void teleopPeriodic() {
		System.out.println("Loop start: " + Timer.getFPGATimestamp());

		if (HardwareMap.isRobotGroundMount()) {
			groundMountFSM.update(input);
		} else {
			armSystem.update(input);
		}
		driveSystem.update(input);
		spinningIntakeFSM.update(input);

		System.out.println("Loop end: " + Timer.getFPGATimestamp());
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
