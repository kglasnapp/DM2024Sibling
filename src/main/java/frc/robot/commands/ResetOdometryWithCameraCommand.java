package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLightPoseSubsystem;
import static frc.robot.Util.logf;

public class ResetOdometryWithCameraCommand extends Command {

    LimeLightPoseSubsystem limeLightPoseSubsystem;

    public ResetOdometryWithCameraCommand(LimeLightPoseSubsystem lightPoseSubsystem) {
        this.limeLightPoseSubsystem = lightPoseSubsystem;
    }

    @Override
    public void initialize() {
        logf("*************************Starting the reset odometry\n");
        String pipeLine = "botpose_wpiblue";//(Robot.alliance == Alliance.Red) ? "botpose_wpired" : "botpose_wpiblue";
        double llPose[] = NetworkTableInstance.getDefault().getTable(limeLightPoseSubsystem.cameraId).getEntry(pipeLine)
                .getDoubleArray(new double[6]);
        double cameraAngle = Math.toRadians(llPose[5]);
        Pose2d visionPose = new Pose2d(llPose[0], llPose[1], new Rotation2d(cameraAngle));
        limeLightPoseSubsystem.setCurrentPose(visionPose);
        // limeLightPoseSubsystem.setCurrentPose(new Pose2d(1.89, 0.5, new Rotation2d(Math.toRadians(180))));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
