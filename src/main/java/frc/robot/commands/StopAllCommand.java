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
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PoseSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import static frc.robot.Util.logf;

public class StopAllCommand extends Command {
    IndexerSubsystem indexer;
    ShooterSubsystem shooter;
    IntakeSubsystem intake;

    public StopAllCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, IntakeSubsystem intake) {
        this.indexer = indexer;
        this.shooter = shooter;
        this.intake = intake;
        addRequirements(intake);
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

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
        shooter.stop();
        indexer.stop();
        intake.stop();
        logf("stop all finished");
        return true;
    }
}