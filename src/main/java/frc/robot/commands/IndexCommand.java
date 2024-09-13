package frc.robot.commands;

import static frc.robot.utilities.Util.logf;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IndexCommand extends Command {
    IndexerSubsystem indexerSubsystem;
    long startTime;

    public IndexCommand(IndexerSubsystem indexerSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        //addRequirements(intakeSubsystem);
        addRequirements(indexerSubsystem);
    }

    @Override
    public void initialize() {
        startTime = RobotController.getFPGATime();
        indexerSubsystem.setSpeed(IndexerSubsystem.INTAKE_SPEED);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        boolean note = indexerSubsystem.isNotePresent();
        return note;
    }

}
