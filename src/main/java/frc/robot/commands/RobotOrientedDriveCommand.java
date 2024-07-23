package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import static frc.robot.utilities.Util.logf;

public class RobotOrientedDriveCommand extends Command {

    DrivetrainSubsystem drivetrainSubsystem;
    double xSpeed;
    double ySpeed;
    double angleSpeed;
    double duration;
    double initTime = 0;
    boolean isFinished = false;

    public RobotOrientedDriveCommand(DrivetrainSubsystem drivetrainSubsystem, double xSpeed, double ySpeed,
            double angleSpeed,
            double duration) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.angleSpeed = angleSpeed;
        this.duration = duration;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        initTime = RobotController.getFPGATime() / 1000;
        logf("Robot Drive Speed X: %.4f y:%.4f angle:%.4f\n", xSpeed, ySpeed, angleSpeed);
        drivetrainSubsystem.drive(new ChassisSpeeds(xSpeed, ySpeed, angleSpeed));
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return RobotController.getFPGATime() / 1000 > initTime + duration;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
    }

}
