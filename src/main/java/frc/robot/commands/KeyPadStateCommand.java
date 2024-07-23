package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utilities.KeyPadPositionSupplier;

/**
 * This command is used to set the state of the keypad.
 * Tells the robot to which set of targets we want it to go after.
 */
public class KeyPadStateCommand extends Command {
    boolean complete = false;
    int state = 0;
    public KeyPadStateCommand(int state) {
        this.state = state;
    }

    @Override
    public void execute() { 
        KeyPadPositionSupplier.state = state;
        complete = true;
    }

    @Override
    public boolean isFinished() {
        return complete;
    }
    
}
