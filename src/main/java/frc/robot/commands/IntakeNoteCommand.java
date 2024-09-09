package frc.robot.commands;

import static frc.robot.utilities.Util.logf;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeNoteCommand extends Command {
    IntakeSubsystem intakeSubsystem;
    IndexerSubsystem indexerSubsystem;
    long startTime;
    boolean finished;
    boolean note;

    public IntakeNoteCommand(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        startTime = RobotController.getFPGATime();
        intakeSubsystem.intakeIn();
        indexerSubsystem.setSpeed(.3);
        logf("Start Intake\n");
        finished = false;
        note = false;
    }

    @Override
    public void execute() {
        note = indexerSubsystem.isNotePresent();
        if (note) {
            finished = true;
        }  
    }

    @Override
    public void end(boolean interrupted) {
        long elapsedTime = RobotController.getFPGATime() - startTime;
        logf("Intake complete saw a note at %.1f seconds\n", elapsedTime / 1000000.0);
        indexerSubsystem.stop();
        intakeSubsystem.stop();
        finished = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

}
