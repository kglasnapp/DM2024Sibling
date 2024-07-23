package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utilities.*;

/**
 * An example command. You can replace me with your own command.
 */
public class DisplayLogCommand extends Command {
  private String s;
  boolean includeTimestamp;

  public DisplayLogCommand(String s) {
    // Use requires() here to declare subsystem dependencies
    this.s = s;
  }

  public DisplayLogCommand(String s, boolean includeTimestamp) {
    // Use requires() here to declare subsystem dependencies
    this.s = s;
    this.includeTimestamp = includeTimestamp;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    if (includeTimestamp) {
      Util.logf("%s - %.3f\n", s, RobotController.getFPGATime()/1000.0);
    } else {
      Util.logf("%s\n", s);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }
 
}