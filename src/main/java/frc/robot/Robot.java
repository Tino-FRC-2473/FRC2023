// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;
// Systems
import frc.robot.systems.FSMSystem;

/**
 * The VM is configured to automatically run this class, and to call the
 *  functions corresponding to each mode, as described in the TimedRobot
 *  documentation.
 */
public class Robot extends TimedRobot {
    /** TeleopInput object to provide driver inputs. */
    private TeleopInput input;

    // Systems
    /** FSMSystem object manages execution flow of different states. */
    private FSMSystem fsmSystem;

    /**
     * This function is run when the robot is first started up and should
     *  be used for any initialization code.
     */
    @Override
    public void robotInit() {
        System.out.println("robotInit");
        input = new TeleopInput();

        // Instantiate all systems here
        fsmSystem = new FSMSystem();
    }

    /**
     * Initialization for autonomous.
     */
    @Override
    public void autonomousInit() {
        System.out.println("-------- Autonomous Init --------");
        fsmSystem.reset();
    }

	public void autonomousPeriodic() {
		PhotonCameraWrapper loc = new PhotonCameraWrapper();
		loc.update();
		//fsmSystem.update(null);
	}

	/**
	 * Initialization for teleop.
	 */
	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");
		fsmSystem.reset();
	}

	public void teleopPeriodic() {
		// LimeLight thing = new LimeLight();
		// thing.update();
		PhotonCameraWrapper loc = new PhotonCameraWrapper();
		loc.update();
	}

	/**
	 * Disabled initialization.
	 */
	@Override
	public void disabledInit() {
		System.out.println("-------- Disabled Init --------");
	}

	public void disabledPeriodic() {

	}

	/**
	 * Test initialization.
	 */
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
