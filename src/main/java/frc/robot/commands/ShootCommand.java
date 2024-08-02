package frc.robot.commands;
// Copyright (c) FIRST and other WPILib contributors.

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

    public ShootCommand(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.indexer = indexer;
        this.shooter = shooter;
    }

    public static enum State {
        IDLE,
        WAIT_SHOOT_SPEED,
        WAIT_NOTE_OUT,
        WAIT,
    };

    State state = State.IDLE;

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        state = State.IDLE;
        boolean note = indexer.isNotePresent();
        if (note) {
            shooter.setAllShooterSpeed(.95);
            state = State.WAIT_SHOOT_SPEED;
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
        if (state == State.WAIT_SHOOT_SPEED) {
            if (shooter.isShooterAtSpeed(5500)) {
                state = State.WAIT_NOTE_OUT;
                indexer.setSpeed(.5);
            }
        }
        if (state == State.WAIT_NOTE_OUT) {
            boolean note = indexer.isNotePresent();
            if (!note) {
                logf("Shoot Command Note Out\n");
                state = State.WAIT;
                waitCount = 5;
            }
        }
        if (state == State.WAIT) {
            waitCount--;
            if (waitCount < 0) {
                logf("Shoot Command finished\n");
                indexer.setSpeed(0);
                shooter.setAllShooterSpeed(0);
                return true;
            }

        }

        return false;
    }
}