/*
* MIT License
*
* Copyright (c) 2022 PhotonVision
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
	AutoController autoCtrl = new AutoController();
	Drivetrain dt = new Drivetrain();



	@Override
	public void robotInit() {
		// Flush NetworkTables every loop. This ensures that robot pose and other values
		// are sent during every iteration.
		setNetworkTablesFlushEnabled(true);
	}

	@Override
	public void autonomousInit() {
		resetOdometery();
		autoCtrl.startPath();
	}

	@Override
	public void autonomousPeriodic() {
		ChassisSpeeds speeds = autoCtrl.getCurMotorCmds(dt.getCtrlsPoseEstimate());
		dt.drive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void robotPeriodic() {
	}

	@Override
	public void disabledPeriodic() {
		dt.drive(0, 0);
	}

	@Override
	public void simulationPeriodic() {
	}

	private void resetOdometery() {
		Pose2d startPose = autoCtrl.getInitialPose();
		dt.resetOdometry(startPose);
	}
}