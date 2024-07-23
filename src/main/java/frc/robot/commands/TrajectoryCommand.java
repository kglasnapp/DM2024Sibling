package frc.robot.commands;

import static frc.robot.Util.logf;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.function.Supplier;

//import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Util;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TrajectoryCommand extends Command {
    DrivetrainSubsystem drivetrainSubsystem;

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(7, 2.5);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(7, 2.5);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRATINTS = new TrapezoidProfile.Constraints(
            Math.toRadians(180), Math.toRadians(180));
    private final ProfiledPIDController xController = new ProfiledPIDController(0.1, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(0.1, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(0.015, 0, 0, OMEGA_CONSTRATINTS);
    private final Supplier<Pose2d> poseProvider;
    Pose2d initialPose;
    Supplier<Pose2d> destinationProvider;
    Trajectory trajectory;
    long initialTime;
    Pose2d destination = new Pose2d(4.49, 5.08, Rotation2d.fromDegrees(0));

    public TrajectoryCommand(String filename, DrivetrainSubsystem drivetrainSubsystem, Supplier<Pose2d> poseProvider) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.poseProvider = poseProvider;
        trajectory = generateTrajectory(filename);
        init();
        List<Trajectory.State> states = trajectory.getStates();
        destination = states.get(states.size()-1).poseMeters;
        //HolonomicDriveController h = null;
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }

    public static Trajectory generateTrajectory(String filename) {
        Trajectory trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + filename, ex.getStackTrace());
        }
        return trajectory;

        // var interiorWaypoints = new ArrayList<Translation2d>();
        // interiorWaypoints.add(new Translation2d(2.8, 5.08));
        // // interiorWaypoints.add(new Translation2d(Units.feetToMeters(21.04), Units.feetToMeters(18.23)));
        // Pose2d startPose = new Pose2d(1.75,4.36,new Rotation2d(Math.toRadians(180)));
        // TrajectoryConfig config = new TrajectoryConfig(7, 3.5);
        // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        //     startPose,
        //     interiorWaypoints,
        //     destination,
        //     config);

        // return trajectory;
    }

    void init() {
        xController.setTolerance(0.005);
        yController.setTolerance(0.005);
        omegaController.setTolerance(Units.degreesToRadians(1));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrainSubsystem);
    }

    public void initialize() {
        initialPose = poseProvider.get();
        logf("Init Trajectory Follow pose:<%.2f,%.2f,%.2f> yaw:%.2f\n", initialPose.getX(), initialPose.getY(),
                initialPose.getRotation().getDegrees(), drivetrainSubsystem.m_navx.getYaw());
        initialTime = RobotController.getFPGATime();

        omegaController.reset(initialPose.getRotation().getRadians());
        xController.reset(initialPose.getX());
        yController.reset(initialPose.getY());

    }

    @Override
    public void execute() {

        var robotPose = poseProvider.get();

        // if (Robot.count % 10 == 8) {
        //     SmartDashboard.putNumber("goal X", goalX);
        //     SmartDashboard.putNumber("goal Y", goalY);
        //     SmartDashboard.putNumber("goal A", goalAngle);
        // }

        Trajectory.State goal = trajectory
                .sample(((double) (RobotController.getFPGATime() - initialTime)) / 1000000.0);
        if (Robot.count % 10 == 3) {
            // logf("Trajectory Path time:%.3f goal:<%.2f,%.2f,%.2f> robot pose:<%.2f,%.2f,%.2f>\n",
            //         (RobotController.getFPGATime() - initialTime) / 1000000.0,
            //         goal.poseMeters.getX(), goal.poseMeters.getY(),
            //         goal.poseMeters.getRotation().getDegrees(),
            //         robotPose.getX(), robotPose.getY(), robotPose.getRotation().getDegrees());
            logf("Trajectory Path time:%.3f goal:<%.2f,%.2f,%.2f> robot pose:<%.2f,%.2f,%.2f>\n",
                    (RobotController.getFPGATime() - initialTime) / 1000000.0,
                    destination.getX(), destination.getY(),
                    destination.getRotation().getDegrees(),
                    robotPose.getX(), robotPose.getY(), robotPose.getRotation().getDegrees());
        }

        xController.setGoal(goal.poseMeters.getX());
        yController.setGoal(goal.poseMeters.getY());
        omegaController.setGoal(goal.poseMeters.getRotation().getRadians());
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
        var robotPose = poseProvider.get();
        if (Math.abs(robotPose.getX() - destination.getX()) < 0.03 &&
                Math.abs(robotPose.getY() - destination.getY()) < 0.03 &&
                Math.abs((Util.normalizeAngle(robotPose.getRotation().getDegrees() -
                        destination.getRotation().getDegrees()))) < 2) {
            logf("Trajectory Follow Complete time:%3f robot pose:<%.2f,%.2f,%.2f, %b, %b, %b> yaw:%.2f\n",
                    (RobotController.getFPGATime() - initialTime) / 1000000.0,
                    robotPose.getX(), robotPose.getY(), robotPose.getRotation().getDegrees(), true, true,
                    true, drivetrainSubsystem.m_navx.getYaw());
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            logf("*********** Finished straight path command with  interrupted:%b\n",
                    interrupted);
        }
        drivetrainSubsystem.stop();
    }
}
