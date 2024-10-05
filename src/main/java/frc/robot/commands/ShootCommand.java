package frc.robot.commands;
// Copyright (c) FIRST and other WPILib contributors.

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.PoseSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import static frc.robot.Util.logf;

public class ShootCommand extends Command {
    IndexerSubsystem indexer;
    ShooterSubsystem shooter;
    PoseSubsystem poseSubsystem;
    long startTime;
    int waitCount = 0;
    State lastState = State.IDLE;
    double angle;
    double speedPercentage = 0;
    int MAX_SPEED = 6500;
    boolean finished = false;

    public ShootCommand(ShooterSubsystem shooter, IndexerSubsystem indexer,
            PoseSubsystem poseSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.indexer = indexer;
        this.shooter = shooter;
        this.poseSubsystem = poseSubsystem;
        addRequirements(shooter);
        addRequirements(indexer);
        // addRequirements(null);
    }

    public ShootCommand(ShooterSubsystem shooter, IndexerSubsystem indexer,
            PoseSubsystem poseSubsystem, double speedPercentage) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.indexer = indexer;
        this.shooter = shooter;
        this.poseSubsystem = poseSubsystem;
        this.speedPercentage = speedPercentage;
        //addRequirements(shooter);
        addRequirements(indexer);
        addRequirements(shooter);
    }

    public static enum State {
        IDLE,
        WAIT_SHOOT_SPEED,
        WAIT1,
        WAIT_NOTE_OUT,
        WAIT2,
        FINISHED
    };

    State state = State.IDLE;

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logf("Initializing the shooters\n");
        state = State.IDLE;
        startTime = RobotController.getFPGATime();
        boolean note = indexer.isNotePresent();
        finished = false;
        logf("The note is present: %b, speed: %.2f\n", note, speedPercentage);
        if (note) {
            shooter.setAllShooterPower(speedPercentage);
            state = State.WAIT_SHOOT_SPEED;
        } else {
            state = State.FINISHED;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (state != lastState) {
            long elapsedTime = RobotController.getFPGATime() - startTime;
            //logf("ShootCommand new state:%s elapsed:%.2f\n", state, elapsedTime / 1000000.0);
        }
        if (state == State.WAIT_SHOOT_SPEED) {
            if (shooter.isShooterAtSpeed(MAX_SPEED * (speedPercentage - .02))) {
                state = State.WAIT1;
                waitCount = 25;
            }
        }
        if (state == State.WAIT1) {
            waitCount--;
            if (waitCount < 0) {
                state = State.WAIT_NOTE_OUT;
                indexer.setSpeed(IndexerSubsystem.SHOOT_SPEED);
            }
        }
        if (state == State.WAIT_NOTE_OUT) {
            boolean note = indexer.isNotePresent();
            logf("Shoot Command Note Out: %b\n", note);
            if (!note) {

                state = State.WAIT2;
                waitCount = 25;
            }
        }
        if (state == State.WAIT2) {
            waitCount--;
            if (waitCount < 0) {
                logf("Shoot Command finished\n");
                indexer.stop();
                shooter.stop();
                finished = true;
            }

        }
        if (state == State.FINISHED) {
            finished = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        logf("shoot command end\n");
        shooter.stop();
        indexer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return finished;
    }

    double angleTable[][] = {
            { 1.31, -8.05 },
            { 1.71, -13.39 },
            { 2.06, -15.50 },
            { 2.37, -21.0 },
            { 2.67, -23 },
            { 2.99, -24 },
            { 3.25, -25.1 },
            { 3.70, -26 }
    };

    public double calculateTiltAngle() {
        Pose2d pose = poseSubsystem.get();
        Alliance alliance = DriverStation.getAlliance().get();
        Pose2d speakerPose = alliance == Alliance.Blue ? RobotContainer.BLUE_SPEAKER : RobotContainer.RED_SPEAKER;
        double distance = distance(speakerPose, pose);
        logf("Distance to the target: %s\n", distance);

        if (distance <= 1.31) {
            return 0;
        }
        for (int i = 1; i < angleTable.length; ++i) {
            if (distance <= angleTable[i][0]) {
                double m = (angleTable[i][1] - angleTable[i - 1][1]) / (angleTable[i][0] - angleTable[i - 1][0]);
                double b = angleTable[i][1] - (m * angleTable[i][0]);
                return m * distance + b;
            }
        }
        double m = (angleTable[angleTable.length - 1][1] - angleTable[angleTable.length - 2][1])
                / (angleTable[angleTable.length - 1][0] - angleTable[angleTable.length - 2][0]);
        double b = angleTable[angleTable.length - 1][1] - (m * angleTable[angleTable.length - 1][0]);
        angle = m * distance + b;
        // double dsqr = distance * distance;
        // double angle = 4.4263 * dsqr * distance - 23.6759 * dsqr + 30.1972 * distance
        // - 12.9287;
        // // the robot physically cannot move more than 30 degress. We are adding a
        // // software limit in here.
        // if (distance > 3.488) {
        // angle = -3.284 * distance - 10.116;
        // }
        // // angle = -8 * distance + 7.63;

        // // angle = -(60 - angle);
        if (angle < -30) {
            angle = -30;
        }
        if (angle > 0) {
            angle = 0;
        }

        return angle;
    }

    public static double distance(Pose2d pose1, Pose2d pose2) {
        double dx = pose1.getX() - pose2.getX();
        double dy = pose1.getY() - pose2.getY();

        return Math.sqrt(dx * dx + dy * dy);
    }
}