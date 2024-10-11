// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.utilities.Util.logf;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PoseSubsystem;
import frc.robot.subsystems.TiltSubsystem;

public class AimTiltToSpeaker extends Command {
  private static final double ANGLE_TABLE[][] = {
      { 0.91, 53.0 },
      { 1.39, 49.2 },
      { 2.62, 36.0 },
      { 2.83, 35.0 },
      { 4.14, 29.0 },
  };

  private final TiltSubsystem tiltSybsystem;
  private final PoseSubsystem poseSubsystem;
  private final boolean continuous;
  private Pose2d speakerPose;
  private double targetAngle;
  private double distance;

  /** Creates a new AimTiltToSpeaker. */
  public AimTiltToSpeaker(TiltSubsystem tiltSybsystem, PoseSubsystem poseSubsystem, boolean continuous) {
    this.tiltSybsystem = tiltSybsystem;
    this.poseSubsystem = poseSubsystem;
    this.continuous = continuous;
    addRequirements(tiltSybsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Alliance alliance = DriverStation.getAlliance().get();
    speakerPose = alliance == Alliance.Blue ? RobotContainer.BLUE_SPEAKER : RobotContainer.RED_SPEAKER;
    if (!poseSubsystem.hasGoodPose()) {
      cancel();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distance = distance(poseSubsystem.get(), speakerPose);
    targetAngle = lookupAngle(distance);

    tiltSybsystem.setAngle(targetAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    logf("Aim Tilt finised: distance: %.2f, tilt: %.2f\n", distance, targetAngle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!continuous) {
      boolean isNearTarget = Math.abs(tiltSybsystem.getTiltAngle() - targetAngle) < 0.2;
      return isNearTarget;
    } else {
      return false;
    }
  }

  private static double lookupAngle(double distance) {
    double[] min = ANGLE_TABLE[0];
    double[] max = ANGLE_TABLE[ANGLE_TABLE.length - 1];

    // Clamp small distances to first angle
    if (distance <= min[0]) {
      return min[1];
    }

    for (int i = 1; i < ANGLE_TABLE.length; i++) {
      if (distance > ANGLE_TABLE[i][0]) {
        continue;
      }

      double[] a = ANGLE_TABLE[i - 1];
      double[] b = ANGLE_TABLE[i];

      double alpha = (distance - a[0]) / (b[0] - a[0]);
      double angle = b[1] * alpha + a[1] * (1 - alpha);

      return angle;
    }

    // Clamp large distances to last angle
    return max[1];
  }

  private static double distance(Pose2d pose1, Pose2d pose2) {
    double dx = pose1.getX() - pose2.getX();
    double dy = pose1.getY() - pose2.getY();

    return Math.sqrt(dx * dx + dy * dy);
  }
}
