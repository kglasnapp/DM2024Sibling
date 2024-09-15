package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseSubsystem;
import frc.robot.utilities.Util;

import static frc.robot.utilities.Util.logf;

import java.util.function.DoubleSupplier;

public class RotateCommand extends Command {
    private final DoubleSupplier rotationTargetSupplier;
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final PoseSubsystem poseSubsystem;
    private final boolean relative;
    private final boolean continuous;
    private long initialTime = 0;
    private double offset = 0;
    private double error = 0;

    public RotateCommand(DrivetrainSubsystem drivetrainSubsystem, PoseSubsystem poseSubsystem) {
        this(drivetrainSubsystem, poseSubsystem, 180, true);
    }

    public RotateCommand(DrivetrainSubsystem drivetrainSubsystem, PoseSubsystem poseSubsystem, double degrees) {
        this(drivetrainSubsystem, poseSubsystem, degrees, true);
    }

    public RotateCommand(DrivetrainSubsystem drivetrainSubsystem, PoseSubsystem poseSubsystem, double degrees,
            boolean relative) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.poseSubsystem = poseSubsystem;
        this.rotationTargetSupplier = () -> degrees;
        this.relative = relative;
        this.continuous = false;
        addRequirements(drivetrainSubsystem);
    }

    public RotateCommand(DrivetrainSubsystem drivetrainSubsystem, PoseSubsystem poseSubsystem,
            DoubleSupplier rotationErrorSupplier,
            boolean relative, boolean continuous) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.poseSubsystem = poseSubsystem;
        this.rotationTargetSupplier = rotationErrorSupplier;
        this.relative = relative;
        this.continuous = continuous;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        if (relative) {
            offset = poseSubsystem.get().getRotation().getDegrees();
        }
        initialTime = RobotController.getFPGATime();
        logf("Start Rotate Command\n");
    }

    @Override
    public void execute() {
        double yaw = poseSubsystem.get().getRotation().getDegrees();
        double goal = rotationTargetSupplier.getAsDouble() + offset;

        error = shortestPathBetweenAngles(yaw, goal);
        double omegaSpeed = error * 2.5;

        logf("In rotate command goal = %.2f yaw = %.2f and omegaSpeed = %.2f\n", Util.normalizeAngle(goal),
                Util.normalizeAngle(yaw), omegaSpeed);
        drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, Math.toRadians(omegaSpeed)));
    }

    @Override
    public boolean isFinished() {
        if (!continuous) {
            return Math.abs(error) < 0.5;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        logf("Rotate Command Complete time:%.2f\n", (RobotController.getFPGATime() - initialTime) / 1000000.0);
        drivetrainSubsystem.stop();
    }

    private static double shortestPathBetweenAngles(double from, double to) {
        double result = Util.normalizeAngle(Util.normalizeAngle(to) - Util.normalizeAngle(from));
        return result;
    }
}
