package frc.robot.commands;

import static frc.robot.utilities.Util.logf;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    IntakeSubsystem intakeSubsystem;
    IndexerSubsystem indexerSubsystem;
    long startTime;
    int count = 0;

    public IntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        // addRequirements(intakeSubsystem);
        // addRequirements(indexerSubsystem);
    }

    @Override
    public void initialize() {
        startTime = RobotController.getFPGATime();
        intakeSubsystem.intakeIn();
        logf("Start Intake\n");
    }

    @Override
    public void execute() {
        count++;
    }

    @Override
    public void end(boolean interrupted) {
        long elapsedTime = RobotController.getFPGATime() - startTime;
        logf("Intake complete saw a note at %.1f seconds\n", elapsedTime / 1000000.0);
        intakeSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        //boolean note = indexerSubsystem.isNotePresent();
        return count == 250;
    }

}
