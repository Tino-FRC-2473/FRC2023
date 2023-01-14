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
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
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

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class contains constants for drive train, field, vision,
 * and AprilTags.
 */
public class Constants {
    /**
     * static class for drive train constants.
     */
    static class DriveTrainConstants {
        /** maximum speed power for robot (meters per second). */
        static final double KMAXSPEED = 3.0;
        /** maximum angular speed (1 rotation per second). */
        static final double K_MAX_ANGULAR_SPEED = 2 * Math.PI;
        /** width of track (meters). */
        static final double KTRACKWIDTH = 0.381 * 2;
        /** wheel radius (meters). */
        static final double KWHEELRADIUS = 0.0508;
        /** encoder resolution. */
        static final int K_ENCODER_RESOLUTION = 4096;
        /** distance per pulse. */
        static final double DISTANCEPERPULSE = 2 * Math.PI
            * KWHEELRADIUS / (double) K_ENCODER_RESOLUTION;
    }

    /**
     * static class for field constants.
     */
    static class FieldConstants {
        /** length of field (meters). */
        static final double LENGTH = Units.feetToMeters(54);
        /** width of field (meters). */
        static final double WIDTH = Units.feetToMeters(27);
    }

    /**
     * static class for vision constants.
     */
    static class VisionConstants {
        /** position/angle of camera relative to the center of the robot. */
        static final Transform3d ROBOT_TO_CAM =
                new Transform3d(
                        new Translation3d(0.0, 0.0, 0.5),
                        new Rotation3d(
                                0, 0,
                                0));
        /** name of camera. */
        static final String CAMERA_NAME = "gloworm";
    }

    /**
     * static class for AprilTag constants.
     */
    static class AprilTagConstants {
        /** x coordinate of AprilTag 1. */
        static final double X1 = Units.inchesToMeters(610.77);
        /** y coordinate of AprilTag 1. */
        static final double Y1 = Units.inchesToMeters(42.19);
        /** z coordinate of AprilTag 1. */
        static final double Z1 = Units.inchesToMeters(18.22);
        /** angle of AprilTag 1. */
        static final double ROT1 = 180.0;

        /** x coordinate of AprilTag 2. */
        static final double X2 = Units.inchesToMeters(610.77);
        /** y coordinate of AprilTag 2. */
        static final double Y2 = Units.inchesToMeters(108.19);
        /** z coordinate of AprilTag 2. */
        static final double Z2 = Units.inchesToMeters(18.22);
        /** angle of AprilTag 2. */
        static final double ROT2 = 180.0;
    }
}
