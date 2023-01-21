// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

 import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
 import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.TimedRobot;
 import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
 
 public class Robot extends TimedRobot { 
	 Drivetrain dtSim = new Drivetrain();
	 AutoController autoControl = new AutoController();
	 DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.5);
	 @Override
	 public void robotInit() {
		 // Flush NetworkTables every loop. This ensures that robot pose and other values
		 // are sent during every iteration.
		 setNetworkTablesFlushEnabled(true);
	 }
 
	 @Override
	 public void autonomousInit() {
		 //resetOdometery();
		 autoControl.startPath();
	 }

	 @Override
	 public void autonomousPeriodic() {
		 ChassisSpeeds adjustedSpeeds = autoControl.getCurMotorCmds(dtSim.getCurPose());
		 DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(adjustedSpeeds);

		 //CONVERT above to a diff units, maybe voltage?
		 dtSim.simulationPeriodic(wheelSpeeds.leftMetersPerSecond,wheelSpeeds.rightMetersPerSecond);
	 }
 
	 @Override
	 public void teleopPeriodic() {
		//dt.drive(opInf.getFwdRevSpdCmd(), opInf.getRotateSpdCmd());
	 }
 
	 @Override
	 public void robotPeriodic() {
		//pt.setEstimatedPose(dt.getCtrlsPoseEstimate());
		//pt.update();
	 }
 
	 @Override
	 public void disabledPeriodic() {
		//dt.drive(0, 0);
	 }
 
	 @Override
	 public void simulationPeriodic() {
		//if (opInf.getSimKickCmd()) {
		//     dtSim.applyKick();
		//}
		//dtSim.update();
		//pt.setActualPose(dtSim.getCurPose());
	 }
 
	 private void resetOdometery() {
		//Pose2d startPose = autoCtrl.getInitialPose();
		 //dtSim.resetPose(startPose);
		 //dt.resetOdometry(startPose);
	 }
 }