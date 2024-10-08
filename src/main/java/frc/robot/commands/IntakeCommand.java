package frc.robot.commands;

import static frc.robot.utilities.Util.logf;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;

public class IntakeCommand extends Command {
    IntakeSubsystem intakeSubsystem;
    IndexerSubsystem indexerSubsystem;
    long startTime;
    int count;
    LedSubsystem leds;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, LedSubsystem leds) {
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.leds = leds;
        addRequirements(intakeSubsystem, indexerSubsystem);
    }

    @Override
    public void initialize() {
        startTime = RobotController.getFPGATime();
        intakeSubsystem.intakeIn();
        logf("Start Intake2\n");
        count = 0;

    }

    @Override
    public void execute() {
        count++;
        double current = intakeSubsystem.getMotorCurrent();
        if (current > .01) {
            logf("Intake current:%.2f count:%d\n", current, count);
        }
        leds.setLedsToWhite(current > 12);
    }

    @Override
    public void end(boolean interrupted) {
        long elapsedTime = RobotController.getFPGATime() - startTime;
        logf("Intake complete saw a note at %.1f seconds\n", elapsedTime / 1000000.0);
        intakeSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        boolean note = indexerSubsystem.isNotePresent();
        logf("Intake finished:%b count:%d\n", count > 500 || note, count);
        // TODO Keith what does this do?
        return count > 500 || note; // After 10 Seconds
    }

}
