// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

/**
 * Defines the different types of swerve drive modules that we use and their
 * gear ratios
 *
 * @see <a href=
 *      "https://www.swervedrivespecialties.com/products/mk4i-swerve-module">Mk4i
 *      Spec</a>
 * @see <a href=
 *      "https://www.swervedrivespecialties.com/products/mk4n-swerve-module">Mk4n
 *      Spec</a>
 */
public enum SwerveModuleType {
    Mk4iL2(1.0 / 6.75, 7.0 / 150.0),
    Mk4nL2(1.0 / 5.9, 1.0 / 18.75);

    public final double driveGearRatio;
    public final double angleGearRatio;

    SwerveModuleType(double driveGearRatio, double angleGearRatio) {
        this.driveGearRatio = driveGearRatio;
        this.angleGearRatio = angleGearRatio;
    }
}
