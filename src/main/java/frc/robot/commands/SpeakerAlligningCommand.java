package frc.robot.commands;

import static frc.robot.utilities.Util.logf;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseSubsystem;

public class SpeakerAlligningCommand extends Command {
    PoseSubsystem poseEstimatorSubsystem;
    DrivetrainSubsystem drivetrainSubsystem;
    RotateCommand rotateCommand;

    public SpeakerAlligningCommand(
            PoseSubsystem poseEstimatorSubsystem,
            DrivetrainSubsystem drivetrainSubsystem) {
        this.poseEstimatorSubsystem = poseEstimatorSubsystem;
        this.drivetrainSubsystem = drivetrainSubsystem;
    }

    @Override
    public void initialize() {
        Alliance alliance = DriverStation.getAlliance().get();
        Pose2d speakerPose = alliance == Alliance.Blue ? RobotContainer.BLUE_SPEAKER : RobotContainer.RED_SPEAKER;
        Pose2d pose2d = poseEstimatorSubsystem.get();
        // speakerPose = new Pose2d(speakerPose.getX(), speakerPose.getY(),
        // speakerPose.getRotation());
        double targetAngle = Math
                .toDegrees(Math.atan2(pose2d.getY() - speakerPose.getY() + 0.05, pose2d.getX() - speakerPose.getX()));
        double angleToRotate = 180 - pose2d.getRotation().getDegrees() + targetAngle;
        angleToRotate = frc.robot.utilities.Util.normalizeAngle(angleToRotate);
        logf("Robot aiming to target needs to rotate %.2f degress\n", angleToRotate);
        rotateCommand = new RotateCommand(drivetrainSubsystem, angleToRotate);
        rotateCommand.initialize();
    }

    @Override
    public void execute() {
        rotateCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        rotateCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return rotateCommand.isFinished();
    }

}
