package frc.robot.commands;


import static frc.robot.Util.logf;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class LoadNoteForAutonomousCommand extends Command {
    IntakeSubsystem intakeSubsystem;
    GrabberSubsystem grabberSubsystem;
    
    double startTime;
    boolean finished = false;
    State state;

    /**
     * Number of ticks we allow as a threshold before we accept that the shooter
     * reach
     * the desired angle for the intake
     */
    public final static double INTAKE_ANGLE_POS_THRESHOLD = 1;

    
    /**
     * Multiplier of the distance to calculate the shooter power
     */
    public final static double POWER_RATIO = 1;


    public static enum State {
        CHECK_INTAKE_ANGLE,
        TIMER_ON
    };

    public LoadNoteForAutonomousCommand(IntakeSubsystem intakeSubsystem,
            GrabberSubsystem grabberSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.grabberSubsystem = grabberSubsystem;
        
        addRequirements(intakeSubsystem, grabberSubsystem);
    }

    @Override
    public void initialize() {
        lastState = null;
        finished = false;
        state = State.CHECK_INTAKE_ANGLE;        
        intakeSubsystem.intakeIn();
        startTime = RobotController.getFPGATime() / 1000;
    }

    State lastState = null;

    @Override
    public void execute() {
        if (Robot.count % 10 == 0) {
            SmartDashboard.putString("loadSts", state.toString());
        }
        if (state != lastState) {
            lastState = state;
            logf("Load Note State changed to:%s\n", state);
        }
        switch (state) {
            case CHECK_INTAKE_ANGLE:                
                if (intakeSubsystem.reverseLimit.isPressed()) {
                    startTime = RobotController.getFPGATime() / 1000;
                    state = State.TIMER_ON;
                }
                break;
            case TIMER_ON:
                if ((RobotController.getFPGATime() / 1000) - startTime > 50) {                                
                    finished = true;
                }                
                break;
        }

    }

    @Override
    public void end(boolean interrupted) {
        grabberSubsystem.grabberOff();
    }

    @Override
    public boolean isFinished() {
        if (finished) {
            logf("LoadNoteCommand finished\n");
        }
        return finished;
    }
}
