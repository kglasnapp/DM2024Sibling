package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Util;
import frc.robot.subsystems.DrivetrainSubsystem;
import static frc.robot.Util.logf;

public class StraightPathCommand extends Command {
    DrivetrainSubsystem drivetrainSubsystem;

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(10, 7);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(10, 7);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRATINTS = new TrapezoidProfile.Constraints(
            Math.toRadians(360), Math.toRadians(520));
    private final ProfiledPIDController xController = new ProfiledPIDController(0.1, 0, 0.001, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(0.1, 0, 0.001, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(0.0515, 0, 0, OMEGA_CONSTRATINTS);
    private final Supplier<Pose2d> poseProvider;
    Pose2d initialPose;
    Supplier<Pose2d> destinationProvider;
    Pose2d destination;
    long initialTime;

    public StraightPathCommand(DrivetrainSubsystem drivetrainSubsystem, Supplier<Pose2d> poseProvider,
            Supplier<Pose2d> destination) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.destinationProvider = destination;
        this.poseProvider = poseProvider;
        init();
    }

    public StraightPathCommand(DrivetrainSubsystem drivetrainSubsystem, Supplier<Pose2d> poseProvider,
            Pose2d destination) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.destination = destination;
        this.poseProvider = poseProvider;
        init();
    }

    void init() {
        xController.setTolerance(0.005);
        yController.setTolerance(0.005);
        omegaController.setTolerance(Units.degreesToRadians(1));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drivetrainSubsystem);
    }

    public void initialize() {
        initialTime = RobotController.getFPGATime();
        if (poseProvider != null) {
            initialPose = poseProvider.get();
        } else {
            initialPose = RobotContainer.instance.poseSubsystem.get();
        }
        omegaController.reset(initialPose.getRotation().getRadians());
        xController.reset(initialPose.getX());
        yController.reset(initialPose.getY());
        logf("Init Path Follow pose:<%.2f,%.2f,%.2f> yaw:%.2f\n", initialPose.getX(), initialPose.getY(),
                initialPose.getRotation().getDegrees(), drivetrainSubsystem.m_navx.getYaw());
    }

    double getIntermediateGoal(double endPosition, double initialPosition, double targetTime, double currentTime) {
        double velocity = (endPosition - initialPosition) / targetTime;
        if (targetTime < currentTime) {
            return endPosition;
        }
        return initialPosition + velocity * currentTime;
    }

    @Override
    public void execute() {
        // logf("executing path follow command\n");
        // double currentTime = RobotController.getFPGATime() - initialTime;
        // currentTime/= 1000;
        if (destinationProvider != null) {
            destination = destinationProvider.get();
        }

        // double goalX = getIntermediateGoal(destination.getX(), initialPose.getX(),
        // 1000, currentTime);
        // double goalY = getIntermediateGoal(destination.getY(), initialPose.getY(),
        // 1000, currentTime);
        // double goalAngle =
        // getIntermediateGoal(destination.getRotation().getDegrees(),
        // initialPose.getRotation().getDegrees(), 2, currentTime);
        xController.setGoal(destination.getX());
        yController.setGoal(destination.getY());
        omegaController.setGoal(destination.getRotation().getRadians());
        var robotPose = poseProvider.get();

        // if (Robot.count % 10 == 8) {
        // SmartDashboard.putNumber("goal X", goalX);
        // SmartDashboard.putNumber("goal Y", goalY);
        // SmartDashboard.putNumber("goal A", goalAngle);
        // }

        if (Robot.count % 5 == 3) {
            logf("Path goal:<%.2f,%.2f,%.2f> robot pose:<%.2f,%.2f,%.2f>\n",
                    destination.getX(), destination.getY(),
                    destination.getRotation().getDegrees(),
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
        if (destinationProvider != null) {
            destination = destinationProvider.get();
        }
        var robotPose = poseProvider.get();
        boolean atGoalX = Math.abs(robotPose.getX() - destination.getX()) < 0.05;
        boolean atGoalY = Math.abs(robotPose.getY() - destination.getY()) < 0.05;
        boolean atGoalO = Math.abs((Util.normalizeAngle(robotPose.getRotation().getDegrees() -
                destination.getRotation().getDegrees()))) < 1;
        boolean finished = atGoalX && atGoalY && atGoalO;
        if (finished) {
            logf("Path Follow Complete time:%3f robot pose:<%.2f,%.2f,%.2f, %b, %b, %b> yaw:%.2f\n",
                    (RobotController.getFPGATime() - initialTime) / 1000000.0,
                    robotPose.getX(), robotPose.getY(), robotPose.getRotation().getDegrees(), atGoalX, atGoalY,
                    atGoalO, drivetrainSubsystem.m_navx.getYaw());
        }
        return finished;

    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            logf("*********** Finished straight path command with destination:%s interrupted:%b\n", destination,
                    interrupted);
        }
        drivetrainSubsystem.stop();
    }
}
