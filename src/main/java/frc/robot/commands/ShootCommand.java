package frc.robot.commands;
// Copyright (c) FIRST and other WPILib contributors.

import edu.wpi.first.wpilibj.RobotController;

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import static frc.robot.Util.logf;

public class ShootCommand extends Command {
    IndexerSubsystem indexer;
    ShooterSubsystem shooter;
    long startTime;
    int waitCount = 0;
    STATE lastState = STATE.IDLE;

    public ShootCommand(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.indexer = indexer;
        this.shooter = shooter;
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
        logf("Initializing the shooters\n");
        state = STATE.IDLE;
        startTime = RobotController.getFPGATime();
        boolean note = indexer.isNotePresent();
        logf("The note is present: %b\n", note);
        if (note) {
            shooter.setAllShooterSpeed(.95);
            state = STATE.WAIT_SHOOT_SPEED;
        } else {
            state = STATE.FINISHED;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (state != lastState) {
            long elapsedTime = RobotController.getFPGATime() - startTime;
            logf("ShootCommand new state:%s elasped:%.2f\n", state, elapsedTime / 1000000.0);
        }
        if (state == STATE.WAIT_SHOOT_SPEED) {
            if (shooter.isShooterAtSpeed(5500)) {
                state = STATE.WAIT_NOTE_OUT;
                indexer.setSpeed(.5);
            }
        }
        if (state == STATE.WAIT_NOTE_OUT) {
            boolean note = indexer.isNotePresent();
            logf("Shoot Command Note Out: %b\n", note);
            if (!note) {
                
                state = STATE.WAIT;
                waitCount = 5;
            }
        }
        if (state == STATE.WAIT) {
            waitCount--;
            if (waitCount < 0) {
                logf("Shoot Command finished\n");
                indexer.setSpeed(0);
                shooter.setAllShooterSpeed(0);
                return true;
            }

        }
        if (state == STATE.FINISHED) {
            return true;
        }
        return false;
    }
}