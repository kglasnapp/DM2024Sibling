package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.utilities.Util;
import static frc.robot.utilities.Util.logf;

public class RotateCommand extends Command {
    double initialTime = 0;
    double targetRotationAngle = 180;
    double goal = 0;
    double initialAngle = 0;    
    DrivetrainSubsystem drivetrainSubsystem;

    public RotateCommand(DrivetrainSubsystem drivetrainSubsystem) {
        this(drivetrainSubsystem, 180);
    }

    public RotateCommand(DrivetrainSubsystem drivetrainSubsystem, double targetRotationAngleDegrees) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.targetRotationAngle = targetRotationAngleDegrees; //normalizeAngle(targetRotationAngleDegrees);        
    }

    @Override
    public void initialize() {
        addRequirements(drivetrainSubsystem);
        initialAngle = normalizeAngle(drivetrainSubsystem.getGyroscopeRotation().getDegrees());
        goal = normalizeAngle(initialAngle + targetRotationAngle);        
        logf("Start Rotate Command initial angle: %.2f targetRotation = %.2f shortestPath = %.2f\n", 
            initialAngle, targetRotationAngle, shortestPathBetweenAngles(initialAngle, goal));
        initialTime = RobotController.getFPGATime();
    }

    
  public static void main(String arg[]) {
    System.out.println(shortestPathBetweenAngles(179,-178));
    System.out.println(shortestPathBetweenAngles(-179,178));
    System.out.println(shortestPathBetweenAngles(179,178));
    System.out.println(shortestPathBetweenAngles(90,45));
  }

  public static double shortestPathBetweenAngles(double sourceAngle, double targetAngle) {
    double result = targetAngle - sourceAngle;
    result += (result>180) ? -360 : (result<-180) ? 360 : 0;
    return result;
  }

    @Override
    public void execute() {
        double normilizedYaw = normalizeAngle(drivetrainSubsystem.getGyroscopeRotation().getDegrees());
        double shortestPathAngle = shortestPathBetweenAngles(normilizedYaw, goal);
        double omegaSpeed = shortestPathAngle / 64;
        logf("In rotate command goal = %.2f yaw = %.2f and omegaSpeed = %.2f\n", goal, normilizedYaw, omegaSpeed);
        drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, Math.toRadians(omegaSpeed)));
    }

    double normalizeAngle(double angle) {
        return Util.normalizeAngle(angle);
    }

    @Override
    public boolean isFinished() {
        double currentAngle = normalizeAngle(drivetrainSubsystem.getGyroscopeRotation().getDegrees());
        return Math.abs(normalizeAngle(goal - currentAngle)) < 1;
    }

    @Override
    public void end(boolean interrupted) {
        logf("Rotate Command Complete time:%.2f\n", (RobotController.getFPGATime() - initialTime) / 1000000);
        drivetrainSubsystem.stop();
    }
}
