package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ExtenderCommand;
import frc.robot.commands.GrabberCommand;
import frc.robot.commands.RobotOrientedDriveCommand;
import frc.robot.commands.RobotOrientedDriveDeacceleratedCommand;
import frc.robot.commands.RotateCommand;
import frc.robot.commands.ShoulderCommand;
import frc.robot.commands.StraightPathCommand;
import frc.robot.commands.ZeroExtenderCommand;
import frc.robot.commands.ZeroShoulderCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.PoseEstimatorAggregator;

public class AutonomousCommandFactory {

  public static Command getAutonomousAcceleratedAndLeftOutCommand(DrivetrainSubsystem m_drivetrainSubsystem,
      ArmSubsystem m_armSubsystem,
      GrabberSubsystem grabberSubsystem) {
    return getAutonomousSimpleCommand(m_drivetrainSubsystem, m_armSubsystem, grabberSubsystem)
        .andThen(new RobotOrientedDriveDeacceleratedCommand(m_drivetrainSubsystem, 0, -0.04, 0, 500.0))
        .andThen(new RobotOrientedDriveDeacceleratedCommand(m_drivetrainSubsystem, -0.04, 0, 0, 4000))
        .andThen(new RobotOrientedDriveDeacceleratedCommand(m_drivetrainSubsystem, 0, 0, 0, 500));
  }

  public static Command getAutonomousSimpleAndLeftOutCommand(DrivetrainSubsystem m_drivetrainSubsystem,
      ArmSubsystem m_armSubsystem,
      GrabberSubsystem grabberSubsystem) {
    return getAutonomousSimpleCommand(m_drivetrainSubsystem, m_armSubsystem, grabberSubsystem)
        .andThen(new RobotOrientedDriveCommand(m_drivetrainSubsystem, 0, -0.02, 0, 500.0))
        .andThen(new RobotOrientedDriveCommand(m_drivetrainSubsystem, -0.02, 0, 0, 4000))
        .andThen(new RobotOrientedDriveCommand(m_drivetrainSubsystem, 0, 0, 0, 500));
  }

  public static Command getAutonomousSimpleAndRightDeacceleratedOutCommand(DrivetrainSubsystem m_drivetrainSubsystem,
      ArmSubsystem m_armSubsystem,
      GrabberSubsystem grabberSubsystem) {
    return getAutonomousSimpleCommand(m_drivetrainSubsystem, m_armSubsystem, grabberSubsystem)
        .andThen(new RobotOrientedDriveDeacceleratedCommand(m_drivetrainSubsystem, 0, 0.04, 0, 500))
        .andThen(new RobotOrientedDriveDeacceleratedCommand(m_drivetrainSubsystem, -0.04, 0, 0, 4000))
        .andThen(new RobotOrientedDriveDeacceleratedCommand(m_drivetrainSubsystem, 0, 0, 0, 500));
  }

  public static Command getAutonomousSimpleAndRightOutCommand(DrivetrainSubsystem m_drivetrainSubsystem,
      ArmSubsystem m_armSubsystem,
      GrabberSubsystem grabberSubsystem) {
    return getAutonomousSimpleCommand(m_drivetrainSubsystem, m_armSubsystem, grabberSubsystem)
        .andThen(new RobotOrientedDriveCommand(m_drivetrainSubsystem, 0, 0.02, 0, 500))
        .andThen(new RobotOrientedDriveCommand(m_drivetrainSubsystem, -0.02, 0, 0, 4000))
        .andThen(new RobotOrientedDriveCommand(m_drivetrainSubsystem, 0, 0, 0, 500));
  }

  public static Command getAutonomousSimpleCommand(DrivetrainSubsystem m_drivetrainSubsystem,
      ArmSubsystem m_armSubsystem, GrabberSubsystem grabberSubsystem) {
    return new GrabberCommand(grabberSubsystem, false)
        .andThen(new ZeroExtenderCommand(m_armSubsystem))
        .andThen(new ShoulderCommand(m_armSubsystem, 4000000))
        .andThen(new ExtenderCommand(m_armSubsystem, 400000000))
        .andThen(new WaitCommand(1))
        .andThen(new GrabberCommand(grabberSubsystem, true))
        .andThen(new WaitCommand(1))
        .andThen(new ZeroExtenderCommand(m_armSubsystem))
        .andThen(new ZeroShoulderCommand(m_armSubsystem));
  }

  public static Command getAutonomousSimpleLowCommand(DrivetrainSubsystem m_drivetrainSubsystem,
  ArmSubsystem m_armSubsystem, GrabberSubsystem grabberSubsystem) {
return new GrabberCommand(grabberSubsystem, false)
    .andThen(new ZeroExtenderCommand(m_armSubsystem))
    .andThen(new ShoulderCommand(m_armSubsystem, 67000))
    //.andThen(new ExtenderCommand(m_armSubsystem, 400000000))
    .andThen(new WaitCommand(0.25))
    .andThen(new GrabberCommand(grabberSubsystem, true))
    .andThen(new WaitCommand(0.25))
    .andThen(new ZeroExtenderCommand(m_armSubsystem))
    .andThen(new ZeroShoulderCommand(m_armSubsystem));
}

  public static Command getSetPositionAndBalanceCommand(DrivetrainSubsystem m_drivetrainSubsystem,
      PoseEstimatorAggregator poseEstimator) {
    return new CommandBase() {
      @Override
      public void initialize() {
        DefaultDriveCommand.autonomous = true;
        SwerveModule.powerRatio = 1.5;
      }

      @Override
      public boolean isFinished() {
        return true;
      }

    }.andThen(
        new RobotOrientedDriveCommand(m_drivetrainSubsystem, -0.01, 0, 0, 250))
        // comment out drive tpo location then drive straight back instead
        // new StraightPathCommand(m_drivetrainSubsystem, poseEstimator,
        // new Pose2d(2.0, 2.4, new Rotation2d(Math.toRadians(177)))))
        .andThen(new CommandBase() {
          @Override
          public void initialize() {
            DefaultDriveCommand.autonomous = false;
            SwerveModule.powerRatio = SwerveModule.TURBO;
          }

          @Override
          public boolean isFinished() {
            return true;
          }

        })
        .andThen(new BalanceCommand(m_drivetrainSubsystem));
  }

  public static Command getOverAndBalanceCommand(DrivetrainSubsystem m_drivetrainSubsystem,
      PoseEstimatorAggregator poseEstimator) {
    return new CommandBase() {
      @Override
      public void initialize() {
        DefaultDriveCommand.autonomous = true;
        SwerveModule.powerRatio = 1.5;
      }

      @Override
      public boolean isFinished() {
        return true;
      }

    }.andThen(
      new RobotOrientedDriveCommand(m_drivetrainSubsystem, -0.02, 0, 0, 4750))
    .andThen(new RotateCommand(m_drivetrainSubsystem))   
    .andThen(new RobotOrientedDriveCommand(m_drivetrainSubsystem, -0.02, 0, 0, 250)) 
    .andThen(
        new RobotOrientedDriveCommand(m_drivetrainSubsystem, -0.01, 0, 0, 250))
        // comment out drive tpo location then drive straight back instead
        // new StraightPathCommand(m_drivetrainSubsystem, poseEstimator,
        // new Pose2d(2.0, 2.4, new Rotation2d(Math.toRadians(177)))))
        .andThen(new CommandBase() {
          @Override
          public void initialize() {
            DefaultDriveCommand.autonomous = false;
            SwerveModule.powerRatio = SwerveModule.TURBO;
          }

          @Override
          public boolean isFinished() {
            return true;
          }

        })
        .andThen(new BalanceCommand(m_drivetrainSubsystem));
  }

  public static Command pickupCubeCommand(DrivetrainSubsystem m_drivetrainSubsystem,
      ArmSubsystem m_armSubsystem, GrabberSubsystem grabberSubsystem,
      PoseEstimatorAggregator poseEstimator) {
    return new CommandBase() {
      @Override
      public void initialize() {
        DefaultDriveCommand.autonomous = true;
        SwerveModule.powerRatio = SwerveModule.NORMAL;
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    }.andThen(new StraightPathCommand(m_drivetrainSubsystem, poseEstimator,
        new Pose2d(2, 6.28, new Rotation2d(Math.toRadians(180)))))
        .andThen(new StraightPathCommand(m_drivetrainSubsystem, poseEstimator,
            new Pose2d(6.84, 6.28, new Rotation2d(Math.toRadians(0)))))
        .andThen(new ShoulderCommand(m_armSubsystem, 32400))
        .andThen(new ExtenderCommand(m_armSubsystem, 3500));

  }
}
