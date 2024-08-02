package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.DrivetrainSubsystem;

public class StraightToAmpCommand extends StraightPathCommand {
    
    public final Pose2d blueAmp = new Pose2d(1.84,7.45, new Rotation2d(Math.toRadians(90)));
    public final Pose2d redAmp = new Pose2d(14.7,7.45, new Rotation2d(Math.toRadians(90)));

    public StraightToAmpCommand(DrivetrainSubsystem drivetrainSubsystem, Supplier<Pose2d> poseProvider) {
        super(drivetrainSubsystem, poseProvider, new Pose2d());
    }
    @Override
    public void initialize() {
        Alliance alliance = DriverStation.getAlliance().get();
        if (alliance == Alliance.Blue) {
            destination = blueAmp;
        } else {
            destination = redAmp;
        }
        super.initialize();
    }

}
