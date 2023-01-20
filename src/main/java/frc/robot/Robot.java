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
<<<<<<< Updated upstream
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
//import frc.robot.AutoController;
//import frc.robot.Drivetrain;
import frc.robot.DrivetrainSim;
import frc.robot.PoseTelemetry;
=======
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
>>>>>>> Stashed changes


<<<<<<< Updated upstream
public class Robot extends TimedRobot {
<<<<<<< Updated upstream
    //AutoController autoCtrl = new AutoController();
    //Drivetrain dt = new Drivetrain();
    //OperatorInterface opInf = new OperatorInterface();
=======
	/**
	 * Runs autonomous periodically.
	 */
	public void autonomousPeriodic() {
		loc.update();
		//fsmSystem.update(null);
	}
>>>>>>> Stashed changes

    DrivetrainSim dtSim = new DrivetrainSim();
    DrivetrainPoseEstimator dtpe = new DrivetrainPoseEstimator(0, 0);
    PoseTelemetry pt = new PoseTelemetry();

    @Override
    public void robotInit() {
        // Flush NetworkTables every loop. This ensures that robot pose and other values
        // are sent during every iteration.
    }
=======

	/** Reports our expected, desired, and actual poses to dashboards */
	public class PoseTelemetry {
		Field2d field = new Field2d();

		Pose2d estPose = new Pose2d();

		public PoseTelemetry() {
			SmartDashboard.putData("Field", field);
			update();
		}

		public void update() {
			field.getObject("Robot").setPose(estPose);
		}

		public void setEstimatedPose(Pose2d in) {
			estPose = in;
		}
	}
	//AutoController autoCtrl = new AutoController();
	DrivetrainSim dtSim = new DrivetrainSim();
	PoseTelemetry pt = new PoseTelemetry();
>>>>>>> Stashed changes

    @Override
    public void autonomousInit() {
        //resetOdometery();
        //autoCtrl.startPath();
        setNetworkTablesFlushEnabled(true);
    }

<<<<<<< Updated upstream
    @Override
    public void autonomousPeriodic() {
        //ChassisSpeeds speeds = autoCtrl.getCurMotorCmds(dt.getCtrlsPoseEstimate());
        //dt.drive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
        //pt.setDesiredPose(autoCtrl.getCurPose2d());
        pt.setEstimatedPose(dtpe.getPoseEst());
        pt.update();
    }

    @Override
    public void teleopPeriodic() {
        //dt.drive(opInf.getFwdRevSpdCmd(), opInf.getRotateSpdCmd());
    }
=======
	@Override
	public void autonomousInit() {
		//resetOdometery();
		//autoCtrl.startPath();
	}

	@Override
	public void autonomousPeriodic() {
		dtSim.update();
		pt.setEstimatedPose(dtSim.getCurPose());
		pt.update();
		//ChassisSpeeds speeds = autoCtrl.getCurMotorCmds(dt.getCtrlsPoseEstimate());
		//dt.drive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
	}
>>>>>>> Stashed changes

    @Override
    public void robotPeriodic() {
        
    }

<<<<<<< Updated upstream
    @Override
    public void disabledPeriodic() {
        //dt.drive(0, 0);
    }
=======
	@Override
	public void disabledPeriodic() {
		//dt.drive(0, 0);
	}
>>>>>>> Stashed changes

    @Override
    public void simulationPeriodic() {
        //if (opInf.getSimKickCmd()) {
        //    dtSim.applyKick();
        //}
        //dtSim.update();
        //pt.setActualPose(dtSim.getCurPose());
    }

<<<<<<< Updated upstream
    private void resetOdometery() {
        //Pose2d startPose = autoCtrl.getInitialPose();
        //dtSim.resetPose(startPose);
        //dt.resetOdometry(startPose);
    }
}
=======
	private void resetOdometery() {
		//Pose2d startPose = autoCtrl.getInitialPose();
		//dt.resetOdometry(startPose);
	}
}
>>>>>>> Stashed changes
