package frc.robot.commands;

import static frc.robot.utilities.Util.logf;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;

public class IntakeNoteCommand extends Command {
    IntakeSubsystem intakeSubsystem;
    IndexerSubsystem indexerSubsystem;
    long startTime;
    boolean finished;
    boolean note;
    LedSubsystem leds;

    public IntakeNoteCommand(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, LedSubsystem leds) {
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.leds = leds;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        startTime = RobotController.getFPGATime();
        intakeSubsystem.intakeIn();
        indexerSubsystem.setSpeed(IndexerSubsystem.INTAKE_SPEED);
        logf("Start Intake1\n");
        finished = false;
        note = false;
    }

    @Override
    public void execute() {
        note = indexerSubsystem.isNotePresent();
        double current = intakeSubsystem.getMotorCurrent();
        if (current > 0.01) {
            // logf("Intake note current:%.2f \n", current);
        }
        leds.setLedsToWhite(current > 12);
        if (note) {
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        long elapsedTime = RobotController.getFPGATime() - startTime;
        logf("Intake complete after %.1f seconds\n", elapsedTime / 1000000.0);
        indexerSubsystem.stop();
        intakeSubsystem.stop();
        finished = true;
    }

    @Override
    public boolean isFinished() {
        if (finished) {
            leds.setLedsToWhite(false);
        }
        return finished;
    }

}
