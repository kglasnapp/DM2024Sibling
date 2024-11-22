package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PoseSubsystem;
import static frc.robot.Util.logf;

public class ResetOdometryWithCameraCommand extends Command {

    PoseSubsystem poseSubsystem;

    public ResetOdometryWithCameraCommand(PoseSubsystem poseSubsystem) {
        this.poseSubsystem = poseSubsystem;
    }

    @Override
    public void initialize() {
        logf("*************************Starting the reset odometry\n");
        poseSubsystem.assumeNextVisionPose();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
