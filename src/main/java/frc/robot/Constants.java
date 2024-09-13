// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.util.Units;
import frc.robot.utilities.SwerveModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
        /**
         * The left-to-right distance between the drivetrain wheels
         *
         * Should be measured from center to center.
         */
        public static final double DRIVETRAIN_TRACK_WIDTH = Units.inchesToMeters(24.75);
        /**
         * The front-to-back distance between the drivetrain wheels
         *
         * Should be measured from center to center.
         */
        public static final double DRIVETRAIN_TRACK_LENGTH = Units.inchesToMeters(24.75);

        /**
         * Drive base radius in meters. Distance from robot center to furthest module.
         */
        public static final double DRIVETRAIN_RADIUS = Math.hypot(DRIVETRAIN_TRACK_WIDTH / 2.0,
                        DRIVETRAIN_TRACK_LENGTH / 2.0);

        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        /**
         * The maximum velocity of the robot in meters per second.
         * <p>
         * This is a measure of how fast the robot should be able to drive in a straight
         * line.
         * 
         * Uses Mk4i for gear ratio as it is slowed than the Mk4n
         * The formula for calculating the theoretical maximum velocity is:
         * <Motor Free Speed RPM> / 60 * <Gear Ratio> * <Wheel Circumference Meters>
         */
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 4800.0 / 60.0
                        * SwerveModuleType.Mk4iL2.driveGearRatio
                        * WHEEL_CIRCUMFERENCE;
        /**
         * The maximum angular velocity of the robot in radians per second.
         * <p>
         * This is a measure of how fast the robot can rotate in place.
         */
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
                        / DRIVETRAIN_RADIUS;

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 1;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 2;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 1;

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 3;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 4;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 3;

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 7;

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 5;

        public static final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
                        // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0),
                        // Rotation PID constants
                        new PIDConstants(5.0, 0.0, 0.0),
                        // Max module speed, in m/s
                        MAX_VELOCITY_METERS_PER_SECOND,
                        // Drive base radius in meters. Distance from robot center to furthest module.
                        DRIVETRAIN_RADIUS,
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
        );

        // Larger numbers -> less sensitive to small changes in input
        // 1.0 = Linear
        // 2.0 = Squared
        // Should be a float greater than 1.0
        public static final double CONTROLLER_SENSITIVITY = 2.0;
        public static final double CONTROLLER_DEAD_BAND = 0.08;

        public static boolean logging = false;
}
