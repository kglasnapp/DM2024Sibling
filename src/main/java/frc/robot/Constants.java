// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

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

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 7; // FIXed Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 8; // FIXed Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 7; // FIXed Set front left steer encoder ID

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 5; // FIXed Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 6; // FIXed Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 5; // FIXed Set front right steer encoder ID

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 1; // FIXed Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 2; // FIXed Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 1; // FIXed Set back left steer encoder ID

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 3; // FIXed Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 4; // FIXed Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 3; // FIXed Set back right steer encoder ID

    // Larger numbers -> less sensitive to small changes in input
    // 1.0 = Linear
    // 2.0 = Squared
    // Should be a float greater than 1.0
    public static final double CONTROLLER_SENSITIVITY = 2.0;
    public static final double CONTROLLER_DEAD_BAND = 0.08;

    public static boolean logging = false;

    // FIXME: None of these are referenced
    public static String robotType = "Swerve";
    public static final int kTimeoutMs = 20;
    public static final boolean isMini = false;
}
