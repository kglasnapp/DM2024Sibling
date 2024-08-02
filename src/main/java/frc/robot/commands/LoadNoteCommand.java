package frc.robot.commands;

import static frc.robot.Util.logf;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystemOld;

public class LoadNoteCommand extends Command {
    IntakeSubsystem intakeSubsystem;
    GrabberSubsystem grabberSubsystem;
    ShooterSubsystemOld shooterSubsystem;
    double startTime;
    boolean finished = false;
    State state = State.CHECK_INTAKE_ANGLE_TO_FEED;

    /**
     * Number of ticks we allow as a threshold before we accept that the shooter
     * reach
     * the desired angle for the intake
     */
    public final static double INTAKE_ANGLE_POS_THRESHOLD = 1;

    /**
     * 
     * Number of DEGRESS we allow as a threshold before we accept that the shooter
     * reach
     * the desired angle for the shooter
     */
    public final static double SHOOTER_ANGLE_DEGREES_THRESHOLD = 1;

    /**
     * Multiplier of the distance to calculate the shooter angle
     */
    public final static double ANGLE_RATIO = 1;

    /**
     * Multiplier of the distance to calculate the shooter power
     */
    public final static double POWER_RATIO = 1;

    public final static double MIN_ANGLE_TO_FEED = -3;

    public static enum State {
        CHECK_INTAKE_ANGLE_TO_FEED,
        WAIT_TO_FEED,
        FEED_NOTE,
        CHECK_INDEXED_COMPLETE,
        INDEX_STEP,
        CHECK_INTAKE_ANGLE,
        END
    };

    public LoadNoteCommand(IntakeSubsystem intakeSubsystem,
            GrabberSubsystem grabberSubsystem,
            ShooterSubsystemOld shooterSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.grabberSubsystem = grabberSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(intakeSubsystem, grabberSubsystem, shooterSubsystem);
    }

    @Override
    public void initialize() {
        lastState = null;
        finished = false;
        state = State.CHECK_INTAKE_ANGLE_TO_FEED;
        shooterSubsystem.setTiltAngle(MIN_ANGLE_TO_FEED);
        intakeSubsystem.intakeIn();
        grabberSubsystem.ignoreSensorToStop = true;
        grabberSubsystem.grabberIn();        
        startTime = RobotController.getFPGATime() / 1000;
    }

    State lastState = null;
    double targetPositionForIndexingWithNeo = 0;

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
            case CHECK_INTAKE_ANGLE_TO_FEED:
                if (Robot.count % 2 == 0) {
                   logf("Shooter tilt angle = %.2f\n ", shooterSubsystem.getTiltAngle());
                }
                if (Math.abs(shooterSubsystem.getTiltAngle() - MIN_ANGLE_TO_FEED) < 3) {
                    state = State.CHECK_INTAKE_ANGLE;
                    startTime = RobotController.getFPGATime() / 1000;
                }
                break;
            case CHECK_INTAKE_ANGLE:           
                // if (Math.abs(intakeSubsystem.getPosition()) < 3) { 
                //     state = State.WAIT_TO_FEED;            
                //     startTime = RobotController.getFPGATime() / 1000;
                //     if (grabberSubsystem.useNeo) {
                //         grabberSubsystem.grabberOff();
                //     }
                // }
                break;
            case WAIT_TO_FEED:
                if (startTime + 50 < RobotController.getFPGATime() / 1000) {
                    if (grabberSubsystem.useNeo) {
                        logf("Executing indexing with NEO\n");                          
                        targetPositionForIndexingWithNeo = grabberSubsystem.indexNoteInShooter();
                        state = State.CHECK_INDEXED_COMPLETE;                        
                    } else {
                        grabberSubsystem.grabberOut();
                        startTime = RobotController.getFPGATime() / 1000;
                        state = State.FEED_NOTE;
                    }                    
                }
                break;
            case INDEX_STEP:
                targetPositionForIndexingWithNeo = grabberSubsystem.indexNoteInShooterSlow();
                state = State.CHECK_INDEXED_COMPLETE;
            case CHECK_INDEXED_COMPLETE:
                if (shooterSubsystem.noteIsIndexedSensor.getVoltage() < 2.9) {
                    state = State.FEED_NOTE;
                } else {
                    state = State.INDEX_STEP;
                }
            case FEED_NOTE:
                if (grabberSubsystem.useNeo) {
                    if (Math.abs(grabberSubsystem.grabberMotorNeo.getEncoder().getPosition() - 
                        targetPositionForIndexingWithNeo) < 0.2) {
                        state = State.END;
                    }
                } else {
                    if ((RobotController.getFPGATime() / 1000) - startTime > 50) {
                        state = State.END;
                        grabberSubsystem.grabberOff();                        
                    }
                }
                break;
            case END:               
                finished = true;
                break;
        }

    }

    @Override
    public void end(boolean interrupted) {
        // shooterSubsystem.setShooterPower(0);
        //intakeSubsystem.intakeIn();
        
            grabberSubsystem.grabberOff();
            grabberSubsystem.ignoreSensorToStop = false;
        
        
    }

    @Override
    public boolean isFinished() {
        if (finished) {
            logf("LoadNoteCommand finished\n");
        }
        return finished;
    }
}
