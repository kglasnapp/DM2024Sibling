package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utilities.SwerveModule;

public class ChangeNormalModeCommand extends Command{
    @Override
          public void initialize() {
            SwerveModule.setPowerRatio(SwerveModule.NORMAL);
          }
    
          @Override
          public boolean isFinished() {
            return true;
          }
}
