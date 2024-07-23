package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GrabberSubsystem;

public class GrabberInCommand extends Command {

    GrabberSubsystem grabberSubsystem;
    
    public GrabberInCommand(GrabberSubsystem grabberSubsystem) {
        this.grabberSubsystem = grabberSubsystem;
        // addRequirements(grabberSubsystem);
    }


    @Override
    public void initialize() {
        grabberSubsystem.grabberIn();        
    }

    // @Override
    // public void end(boolean interrupted) {
    //     grabberSubsystem.grabberOff();
    // }

    @Override
    public boolean isFinished() {
       return true;
    }
}
