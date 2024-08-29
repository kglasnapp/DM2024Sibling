// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Transform3d;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    // TODO Change the next tqo .762 to .62865 -- note .628 is 24.7 inches
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = .762; // FIXed Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = .762; // FIXed Measure and set wheelbase

    // public static final int DRIVETRAIN_PIGEON_ID = 0; // FIXed Set Pigeon ID

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 7; // FIXed Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 8; // FIXed Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 7; // FIXed Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(47.988); //48.340); // FIXed Measure and set front left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 5; // FIXed Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 6; // FIXed Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 5; // FIXed Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(307.617); //305.859); // FIXed Measure and set front right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 1; // FIXed Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 2; // FIXed Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 1; // FIXed Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(158.3 + 30); //345.146); //161.543); // FIXed Measure and set back left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 3; // FIXed Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 4; // FIXed Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 3; // FIXed Set back right steer encoder ID





    // public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1; // FIXed Set front left module drive motor ID
    // public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2; // FIXed Set front left module steer motor ID
    // public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 1; // FIXed Set front left steer encoder ID
    // public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(47.988); //48.340); // FIXed Measure and set front left steer offset

    // public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3; // FIXed Set front right drive motor ID
    // public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4; // FIXed Set front right steer motor ID
    // public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 3; // FIXed Set front right steer encoder ID
    // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(307.617); //305.859); // FIXed Measure and set front right steer offset

    // public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5; // FIXed Set back left drive motor ID
    // public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6; // FIXed Set back left steer motor ID
    // public static final int BACK_LEFT_MODULE_STEER_ENCODER = 5; // FIXed Set back left steer encoder ID
    // public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(158.3 + 30); //345.146); //161.543); // FIXed Measure and set back left steer offset

    // public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7; // FIXed Set back right drive motor ID
    // public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8; // FIXed Set back right steer motor ID
    // public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 7; // FIXed Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(271.3 + 20); //281.865); //275.977); // FIXed Measure and set back right steer offset
    public static boolean logging = false;
    public static String robotType = "Swerve";
    public static final int kTimeoutMs = 20;
    public static final boolean isMini = false;
    public static class VisionConstants {

    
        /**
         * Physical location of the camera on the robot, relative to the center of the robot.
         */
        public static final Transform3d CAMERA_TO_ROBOT[] =
            { };
            // new Transform3d(new Translation3d(-0.36, -0.045, 0.0), new Rotation3d(0.0,0.0,0.0));
      }
}
