package frc.robot.commands;

import static frc.robot.Util.logf;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;

public class PathFollowCommand extends Command {
  DrivetrainSubsystem drivetrainSubsystem;
  Trajectory trajectory;
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRATINTS = new TrapezoidProfile.Constraints(Math.toRadians(180), Math.toRadians(180));
  private final ProfiledPIDController xController = new ProfiledPIDController(1, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(1, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(1, 0, 0, OMEGA_CONSTRATINTS);
  private final Supplier<Pose2d> poseProvider;
  Trajectory.State lastState;
  Trajectory.State lastGoal;

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public PathFollowCommand(DrivetrainSubsystem drivetrainSubsystem, Supplier<Pose2d> poseProvider) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        1.0, // kMaxSpeedMetersPerSecond,
        1.0) // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DrivetrainSubsystem.m_kinematics);
    this.poseProvider = poseProvider;
    // An example trajectory to follow. All units in meters.
    trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0.5, 0.5, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1.0, 1.0),new Translation2d(1.0, 1.0), new Translation2d(0.5, 1.5),
        new Translation2d(2,2)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(0.5, 2.5, new Rotation2d(0)),
        // Pass config
        config);
    xController.setTolerance(0.01);
    yController.setTolerance(0.01);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drivetrainSubsystem);
    List<Trajectory.State> states = trajectory.getStates();
    lastState = states.get(states.size() - 1);
    logf("Path Follow states:%d\n", states.size());
  }

  public PathFollowCommand(Trajectory trajectory, DrivetrainSubsystem drivetrainSubsystem, Supplier<Pose2d> poseProvider) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.trajectory = trajectory;
    this.poseProvider = poseProvider;
   
    xController.setTolerance(0.01);
    yController.setTolerance(0.01 );
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drivetrainSubsystem);
    List<Trajectory.State> states = trajectory.getStates();
    lastState = states.get(states.size() - 1);
    logf("Path Follow states:%d\n", states.size());
  }

  double initialTime = RobotController.getFPGATime();

  public void initialize() {

    initialTime = RobotController.getFPGATime();
    var robotPose = poseProvider.get();
    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
    logf("Init Path Follow pose:<%.2f,%.2f,%.2f>\n", robotPose.getX(), robotPose.getY(),
        robotPose.getRotation().getDegrees());
  }

  @Override
  public void execute() {
    // logf("executing path follow command\n");
    double currentTime = RobotController.getFPGATime();
    Trajectory.State goal = trajectory.sample((currentTime - initialTime) / 1000000);

    lastGoal = goal;

    xController.setGoal(goal.poseMeters.getX());
    yController.setGoal(goal.poseMeters.getY());
    omegaController.setGoal(goal.poseMeters.getRotation().getRadians());
    var robotPose = poseProvider.get();

    if (Robot.count % 10 == 8) {
      SmartDashboard.putNumber("goal X", goal.poseMeters.getX());
      SmartDashboard.putNumber("goal Y", goal.poseMeters.getY());
      SmartDashboard.putNumber("goal A", goal.poseMeters.getRotation().getDegrees());
    }

    if (Robot.count % 20 == 3) {
      logf("Path time:%.3f goal:<%.2f,%.2f,%.2f> robot pose:<%.2f,%.2f,%.2f>\n", (currentTime - initialTime) / 1000000,
          goal.poseMeters.getX(), goal.poseMeters.getX(), goal.poseMeters.getRotation().getDegrees(),
          robotPose.getX(), robotPose.getY(), robotPose.getRotation().getDegrees());
    }
    var xSpeed = xController.calculate(robotPose.getX());
    if (xController.atGoal()) {
      xSpeed = 0;
    }

    var ySpeed = yController.calculate(robotPose.getY());
    if (yController.atGoal()) {
      ySpeed = 0;
    }

    var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());
    if (omegaController.atGoal()) {
      omegaSpeed = 0;
    }

    drivetrainSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()));
  }

  @Override
  public boolean isFinished() {
    // double currentTime = RobotController.getFPGATime();
    // var robotPose = poseProvider.get();
    // logf("Path Follow Complete time:%3f robot pose:<%.2f,%.2f,%.2f>\n", (currentTime - initialTime) / 1000000,
    //     robotPose.getX(), robotPose.getY(), robotPose.getRotation().getDegrees());
    return xController.atGoal() && yController.atGoal() && omegaController.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }
}
