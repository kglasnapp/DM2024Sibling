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
    private static final double SPEED_TABLE[][] = {
            { 0.91, 0.7 },
            // { 1.39, 49.2 },
            { 2.62, 0.9 },
            // { 2.83, 35.0 },
            // { 4.14, 29.0 },
    };

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
    boolean calculateSpeed = true;

    public ShootCommand(ShooterSubsystem shooter, IndexerSubsystem indexer,
            PoseSubsystem poseSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.indexer = indexer;
        this.shooter = shooter;
        this.poseSubsystem = poseSubsystem;
        this.calculateSpeed = true;
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
        this.calculateSpeed = false;
        // addRequirements(shooter);
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
        if (calculateSpeed) {
            Alliance alliance = DriverStation.getAlliance().get();
            Pose2d speakerPose = alliance == Alliance.Blue ? RobotContainer.BLUE_SPEAKER : RobotContainer.RED_SPEAKER;
            if (!poseSubsystem.hasGoodPose()) {
                cancel();
            }
            speedPercentage = lookupSpeed(distance(poseSubsystem.get(), speakerPose));
        }
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
            // logf("ShootCommand new state:%s elapsed:%.2f\n", state, elapsedTime /
            // 1000000.0);
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

    private static double lookupSpeed(double distance) {
        double[] min = SPEED_TABLE[0];
        double[] max = SPEED_TABLE[SPEED_TABLE.length - 1];

        // Clamp small distances to first angle
        if (distance <= min[0]) {
            return min[1];
        }

        for (int i = 1; i < SPEED_TABLE.length; i++) {
            if (distance > SPEED_TABLE[i][0]) {
                continue;
            }

            double[] a = SPEED_TABLE[i - 1];
            double[] b = SPEED_TABLE[i];

            double alpha = (distance - a[0]) / (b[0] - a[0]);
            double angle = b[1] * alpha + a[1] * (1 - alpha);

            return angle;
        }

        // Clamp large distances to last angle
        return max[1];
    }

    private static double distance(Pose2d pose1, Pose2d pose2) {
        double dx = pose1.getX() - pose2.getX();
        double dy = pose1.getY() - pose2.getY();

        return Math.sqrt(dx * dx + dy * dy);
    }
}