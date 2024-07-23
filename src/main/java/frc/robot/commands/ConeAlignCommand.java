package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.utilities.PiecePickerPoseProvider;
import frc.robot.utilities.PiecePickerPoseProvider.PieceEstimatedPose;
import static frc.robot.utilities.Util.logf;

public class ConeAlignCommand extends Command {

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 0.5);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 0.5);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRATINTS = new TrapezoidProfile.Constraints(1, 0.5);

    private PiecePickerPoseProvider pickerPoseProvider;

    private final DrivetrainSubsystem drivetrainSubsystem;
    private final Supplier<Pose2d> poseProvider;

    private final ProfiledPIDController xController = new ProfiledPIDController(1, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(1, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRATINTS);

    private PieceEstimatedPose lastTarget;

    public ConeAlignCommand(
            PiecePickerPoseProvider pickerPoseProvider,
            DrivetrainSubsystem drivetrainSubsystem,
            Supplier<Pose2d> poseProvider) {
        this.pickerPoseProvider = pickerPoseProvider;
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.poseProvider = poseProvider;

        xController.setTolerance(0.20);
        yController.setTolerance(0.20);
        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        lastTarget = null;
        var robotPose = poseProvider.get();
        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }

    public static Pose2d transformConeToField(double coneXToTheRobot, double coneYToTheRobot, Pose2d robotPose2d, double coneAngleInField) {
        double robotAngle = robotPose2d.getRotation().getRadians();
        double goalPoseX = robotPose2d.getX() + Math.sin(robotAngle)*coneXToTheRobot + Math.cos(robotAngle)*coneYToTheRobot;
        double goalPoseY = robotPose2d.getY() - Math.cos(robotAngle)*coneXToTheRobot + Math.sin(robotAngle)*coneYToTheRobot;
        return new Pose2d(goalPoseX, goalPoseY, new Rotation2d(coneAngleInField));
    }

    public static void main(String arg[]) throws Exception {
        Pose2d robotPose[] = {
            new Pose2d(0,0,new Rotation2d(Math.toRadians(0))),
            new Pose2d(0,0,new Rotation2d(Math.toRadians(45))),
            new Pose2d(0,0,new Rotation2d(Math.toRadians(-45))),
            new Pose2d(0,0,new Rotation2d(Math.toRadians(180)))
        };
        for (Pose2d pose2d : robotPose) {
            System.out.println(transformConeToField(5,5, pose2d,0));
        }
        for (Pose2d pose2d : robotPose) {
            System.out.println(transformConeToField(5,10, pose2d,0));
        }
    }

    @Override
    public boolean isFinished() {
        if (lastTarget == null) {
            return true;
        }
        double coneX = lastTarget.getPose().getX();
        double coneY = lastTarget.getPose().getY();
        double coneAngle = lastTarget.getPose().getRotation().getDegrees() + 90;
        boolean atGoalY = Math.abs(coneX - 340) < 40;
        boolean atGoalX = Math.abs(270-coneY) < 40;
        boolean atGoalA = Math.abs(coneAngle) < 10;
        return atGoalA && atGoalX && atGoalY;
    }

    @Override
    public void execute() {
        var robotPose2d = poseProvider.get();
        var robotPose = new Pose3d(
                robotPose2d.getX(),
                robotPose2d.getY(),
                0.0,
                new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));
        boolean atGoalX = false;
        boolean atGoalY = true;
        boolean atGoalA = true;                
        if (pickerPoseProvider.hasResult()) {
            PieceEstimatedPose target = pickerPoseProvider.getResult();
            if (target.getTimestamp() < RobotController.getFPGATime()/1000 - 100) {
                drivetrainSubsystem.stop();
                return;
            }
            lastTarget = target;
            double coneX = target.getPose().getX();
            double coneY = target.getPose().getY();
            double coneAngle = target.getPose().getRotation().getDegrees() + 90;
            atGoalY = Math.abs(coneX - 340) < 40;
            atGoalX = Math.abs(270-coneY) < 40;
            if (Math.abs(coneAngle) > 50) {
                coneAngle = 0;
            }
            atGoalA = Math.abs(Math.abs(coneAngle)) < 10;
            
            //double robotAngle = robotPose.getRotation().getAngle();
            double coneXToTheRobot = (((coneX-340)*0.16) / 100);
            double coneYToTheRobot = (((270-coneY)*0.7) / 100);
            double goalPoseAngle = Math.toRadians(robotPose2d.getRotation().getDegrees() - coneAngle);

            Pose2d coneInFieldPose = transformConeToField(coneXToTheRobot, coneYToTheRobot, robotPose2d, goalPoseAngle);

            // double goalPoseX = robotPose2d.getX() - (Math.cos(robotAngle)*coneYToTheRobot - Math.sin(robotAngle)*coneXToTheRobot);
            // double goalPoseY = robotPose2d.getY() - (Math.sin(robotAngle)* coneYToTheRobot + Math.cos(robotAngle)*coneXToTheRobot);
            
            
            //double goalPoseY = robotPose2d.getY() - (Math.sin(robotAngle)* coneXToTheRobot + Math.cos(robotAngle)*coneYToTheRobot);
            //double goalPoseX = robotPose2d.getX() - (Math.cos(robotAngle)*coneXToTheRobot - Math.sin(robotAngle)*coneYToTheRobot);
            if(Robot.count % 20 == 5) {
               logf("cone X:%.2f Y:%.2f goal X:%.2f Y:%.2f atGoalX:%.2f %b Y:%.2f %b A:%.2f %b\n",coneXToTheRobot, coneYToTheRobot, coneInFieldPose.getX(), coneInFieldPose.getY(), coneX, atGoalX, coneY, atGoalY, coneAngle, atGoalA);
            }
            // Drive
            yController.setGoal(coneInFieldPose.getY());
            xController.setGoal(coneInFieldPose.getX());
            omegaController.setGoal(coneInFieldPose.getRotation().getRadians());
        }
        double millSecs = RobotController.getFPGATime() / 1000;
        if (lastTarget == null || lastTarget.getTimestamp() <= millSecs - 2000) {
            // No target has been visible
            drivetrainSubsystem.stop();
        } else {
            // Drive to the target
            var xSpeed = xController.calculate(robotPose.getX());
            if (atGoalX) {
                xSpeed = 0;
            }

            var ySpeed = yController.calculate(robotPose.getY());
            if (atGoalY) {
                ySpeed = 0;
            }

            var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
            if (atGoalA) {
                omegaSpeed = 0;
            }
            // omegaSpeed = 0;
            drivetrainSubsystem.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose2d.getRotation()));
        }
    }

    double cleanAngle(double angle) {
        angle = Math.abs(angle);
        while (angle > 90) {
            angle-=90;
        }
        return angle;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stop();
    }

}
