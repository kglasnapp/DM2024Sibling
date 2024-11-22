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
    private final PoseSubsystem poseEstimatorSubsystem;
    private final DrivetrainSubsystem drivetrainSubsystem;
    private RotateCommand rotateCommand;

    public SpeakerAlligningCommand(
            PoseSubsystem poseEstimatorSubsystem,
            DrivetrainSubsystem drivetrainSubsystem) {
        this.poseEstimatorSubsystem = poseEstimatorSubsystem;
        this.drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        Alliance alliance = DriverStation.getAlliance().get();
        Pose2d speakerPose = alliance == Alliance.Blue ? RobotContainer.BLUE_SPEAKER : RobotContainer.RED_SPEAKER;

        logf("Robot aiming to speaker target\n");
        rotateCommand = new RotateCommand(drivetrainSubsystem, poseEstimatorSubsystem, () -> {
            Pose2d robotPose = poseEstimatorSubsystem.get();
            return 180 + Math.toDegrees(
                    Math.atan2(robotPose.getY() - speakerPose.getY(),
                            robotPose.getX() - speakerPose.getX()));
        }, false, false);
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
