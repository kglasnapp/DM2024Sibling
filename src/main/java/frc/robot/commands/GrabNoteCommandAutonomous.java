package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class GrabNoteCommandAutonomous extends Command {
    
    RobotContainer robotContainer;
    IntakeNoteCommand intakeCommand;
    GrabberInCommand grabberInCommand;
    DriveToObjectCommand driveToObjectCommand;
    boolean useSensorIfNoteInGrabber = false;

    public GrabNoteCommandAutonomous(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        //intakeCommand = new IntakeCommand(robotContainer.intakeSubsystem, IntakeCommand.State.OUT, 1000);
        grabberInCommand = new GrabberInCommand(robotContainer.grabberSubsystem);
        driveToObjectCommand = new DriveToObjectCommand(robotContainer.drivetrainSubsystem, "note",4000); 
    }

    public GrabNoteCommandAutonomous(RobotContainer robotContainer, boolean useSensorIfNoteInGrabber) {
        this(robotContainer);
        this.useSensorIfNoteInGrabber = useSensorIfNoteInGrabber;
    }

  
    @Override
    public void initialize() {
        intakeCommand.initialize();
        grabberInCommand.initialize();
        driveToObjectCommand.initialize();
        intakeCommandFinished = false;
        grabberCommandFinished = false;
        driveToObjectCommandFinished = false;
    }

    boolean intakeCommandFinished = false;
    boolean grabberCommandFinished = false;
    boolean driveToObjectCommandFinished = false;


    @Override
    public void execute() {
        if (!intakeCommandFinished && !(intakeCommandFinished = intakeCommand.isFinished())) {            
            intakeCommand.execute();
        } 
        if (!grabberCommandFinished && !(grabberCommandFinished = grabberInCommand.isFinished())) {            
            grabberInCommand.execute();
        }
        if (useSensorIfNoteInGrabber) {
            boolean noteInGrabber = grabberInCommand.grabberSubsystem.seeNote();
            if (noteInGrabber) {
                System.out.println("Note in Grabber!!!!");
                driveToObjectCommand.end(true);
                driveToObjectCommandFinished = true;
            }
        }
        if (!driveToObjectCommandFinished && !(driveToObjectCommandFinished = driveToObjectCommand.isFinished())) {            
            driveToObjectCommand.execute();
        }
    }  

    @Override
    public boolean isFinished() {
       return grabberInCommand.grabberSubsystem.getIntakePower() == 0;
    }

    @Override
    public void end(boolean interrupted) {
        intakeCommand.end(interrupted);
        grabberInCommand.end(interrupted);
        driveToObjectCommand.end(interrupted);
    }
}
