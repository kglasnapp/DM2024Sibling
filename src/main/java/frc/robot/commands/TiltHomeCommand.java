package frc.robot.commands;
// Copyright (c) FIRST and other WPILib contributors.

import static frc.robot.utilities.Util.logd;

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TiltSubsystem;
import frc.robot.subsystems.TiltSubsystem.State;

public class TiltHomeCommand extends Command {
    TiltSubsystem tiltSubsystem;
    int waitCount = 0;
    boolean finished = false;
    boolean homing = false;

    public TiltHomeCommand(TiltSubsystem tiltSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.tiltSubsystem = tiltSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
         homing = false;
        logd("Initializing the tilt homing\n", true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!homing) {
            tiltSubsystem.homeTilt();
            homing = true;
            waitCount = 150;
        }
        if (homing && tiltSubsystem.state == State.HOMED) {
            finished = true;
        }
        if (waitCount <= 0) {
            finished = true;
            tiltSubsystem.state = State.STOP;
        }
        waitCount--;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return finished;
    }
}