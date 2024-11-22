package frc.robot.commands;

import static frc.robot.utilities.Util.logf;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeNoteCommand extends Command {
    IntakeSubsystem intakeSubsystem;
    IndexerSubsystem indexerSubsystem;
    long startTime;
    boolean finished;
    boolean note;
    int count = 0;
    int timer = 0;
    boolean startTimer = false;

    public AutoIntakeNoteCommand(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        startTime = RobotController.getFPGATime();
        intakeSubsystem.intakeIn();
        indexerSubsystem.setSpeed(IndexerSubsystem.INTAKE_SPEED);
        logf("Start Intake3\n");
        count = 0;
        timer = 0;
        startTimer = false;
        finished = false;
        note = false;
    }

    @Override
    public void execute() {
        if (count % 5 == 0 && intakeSubsystem.getMotorCurrent() != 0.0) {
            logf("Intake Current: %.2f\n", intakeSubsystem.getMotorCurrent());
         }
        note = indexerSubsystem.isNotePresent();
        if (note) {
            logf("note indexed\n");
            finished = true;
        }
        if (count > 20 && intakeSubsystem.getMotorCurrent() > 13) {
            logf("Note Grabbed\n");
            startTimer = true;
        }
        if (startTimer) {
            //logf("start timer\n");
            timer++;
        }
        if (timer > 225) {
            logf("timer expired\n");
            finished = true;
        }
        if (count > 225) {
            logf("timeout expired\n");
            finished = true;
        }
         count ++;
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
        return finished;
    }

}
