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

    public IntakeNoteCommand(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        //addRequirements(intakeSubsystem);
        // addRequirements(indexerSubsystem);
    }

    @Override
    public void initialize() {
        startTime = RobotController.getFPGATime();
        intakeSubsystem.intakeIn();
        indexerSubsystem.setSpeed(.3);
        logf("Start Intake\n");
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        boolean note = indexerSubsystem.isNotePresent();
        if (note) {
            long elapsedTime = RobotController.getFPGATime() - startTime;
            logf("Intake complete saw a note at %.1f seconds\n", elapsedTime / 1000000.0);
            indexerSubsystem.setSpeed(0);
            intakeSubsystem.intakeStop();

        }
        return note;
    }

}
