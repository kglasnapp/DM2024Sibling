package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

import static frc.robot.utilities.Util.logf;

public class IntakeCommand extends Command {
    IntakeSubsystem intakeSubsystem;
    State state;
    double timeOut;
    double startTime;
    //private final static double THRESHOLD = 2;
    public enum State {
        IN, OUT, OFF
    }

    // This command sets the state of the intake to the value of the state
    // Turn off intake when timeout or over current hits
    public IntakeCommand(IntakeSubsystem intakeSubsystem, State state, double timeOut) {
        this.intakeSubsystem = intakeSubsystem;
        this.state = state;
        this.timeOut = timeOut;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        startTime = RobotController.getFPGATime() / 1000;
        if (state == State.OUT) {
            intakeSubsystem.intakeOut();
        }
        if (state == State.IN) {
            intakeSubsystem.intakeIn();
        }
        
        logf("Intake %s for %.2f milli seconds\n", state, timeOut);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
       return true;
    }


}
