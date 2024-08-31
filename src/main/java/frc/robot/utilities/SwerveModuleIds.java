package frc.robot.utilities;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleIds {
  public final int driveMotorID;
  public final int angleMotorID;
  public final int cancoderID;

  /**
   * Swerve Module Constants to be used when creating swerve modules.
   *
   * @param driveMotorID
   * @param angleMotorID
   * @param canCoderID
   * @param angleOffset
   */
  public SwerveModuleIds(
      int driveMotorID, int angleMotorID, int canCoderID) {
    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;
    this.cancoderID = canCoderID;
  }
}
