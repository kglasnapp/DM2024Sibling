package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GrabberSubsystem;

public class GrabberOutCommand extends Command {

    GrabberSubsystem grabberSubsystem;
    double power = 1;

    public GrabberOutCommand(GrabberSubsystem grabberSubsystem) {
        this.grabberSubsystem = grabberSubsystem;
        addRequirements(grabberSubsystem);
    }

    public GrabberOutCommand(GrabberSubsystem grabberSubsystem, double power) {
        this.grabberSubsystem = grabberSubsystem;
        this.power = power;
        addRequirements(grabberSubsystem);
    }


    @Override
    public void initialize() {
        grabberSubsystem.grabberOut(power);        
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
