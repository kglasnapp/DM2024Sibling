package frc.robot.commands;
// Copyright (c) FIRST and other WPILib contributors.

import static frc.robot.Util.logf;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpShotCommand extends Command {
    IndexerSubsystem indexer;
    ShooterSubsystem shooter;
    long startTime;
    int waitCount = 0;
    STATE lastState = STATE.IDLE;
    double angle;
    double SPEED_PERCENTAGE = .1;
    int MAX_SPEED = 4500;
    boolean finished = false;

    public AmpShotCommand(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        this.indexer = indexer;
        this.shooter = shooter;
        addRequirements(shooter);
    }

    public static enum STATE {
        IDLE,
        WAIT_SHOOT_SPEED,
        WAIT_NOTE_OUT,
        WAIT,
        FINISHED
    };

    STATE state = STATE.IDLE;

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        finished = true;
        boolean note = indexer.isNotePresent();
        logf("Init the amp command note:%b\n", note);
        state = STATE.IDLE;
        startTime = RobotController.getFPGATime();
        if (note) {
            shooter.setAllShooterPower(SPEED_PERCENTAGE);
            state = STATE.WAIT_SHOOT_SPEED;
        } else {
            state = STATE.FINISHED;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        finished = false;
        if (state != lastState) {
            long elapsedTime = RobotController.getFPGATime() - startTime;
            logf("ShootCommand new state:%s elapsed:%.2f\n", state, elapsedTime / 1000000.0);
        }
        if (state == STATE.WAIT_SHOOT_SPEED) {
            if (shooter.isShooterAtSpeed(MAX_SPEED * SPEED_PERCENTAGE)) {
                state = STATE.WAIT_NOTE_OUT;
                indexer.setSpeed(.4);
            }
        }
        if (state == STATE.WAIT_NOTE_OUT) {
            boolean note = indexer.isNotePresent();
            logf("Shoot Command Note Out: %b\n", note);
            if (!note) {
                state = STATE.WAIT;
                waitCount = 50;
            }
        }
        if (state == STATE.WAIT) {
            waitCount--;
            if (waitCount < 0) {
                logf("Shoot Command finished\n");
                indexer.stop();
                shooter.stop();
                finished = true;
            }
        }
        if (state == STATE.FINISHED) {
            finished = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        indexer.stop();
        shooter.stop();
        logf("Amp command end interupt:%b\n", interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return finished;
    }
}