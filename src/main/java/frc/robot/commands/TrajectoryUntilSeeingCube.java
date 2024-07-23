package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

import static frc.robot.Util.logf;

public class TrajectoryUntilSeeingCube extends TrajectoryCommand {

    public TrajectoryUntilSeeingCube(String filename, DrivetrainSubsystem drivetrainSubsystem,
            Supplier<Pose2d> poseProvider) {
        super(filename, drivetrainSubsystem, poseProvider);
    }

    double initTime;

    @Override
    public void initialize() {
        super.initialize();
        initTime = RobotController.getFPGATime();
    }

    @Override
    public boolean isFinished() {
        if (super.isFinished() || (RobotController.getFPGATime() - initTime > 2500000 && 
        "cube".equals(RobotContainer.coralSubsystem.type) && 
        RobotContainer.coralSubsystem.percent > 0.6)) {
            logf("TrajectoryUntilSeeingCube Follow Complete time:%3f is a cube %.2f percent\n",
                    (RobotController.getFPGATime() - initTime) / 1000000.0, RobotContainer.coralSubsystem.percent);
            return true;
        }
        return false;
    }
    
}
