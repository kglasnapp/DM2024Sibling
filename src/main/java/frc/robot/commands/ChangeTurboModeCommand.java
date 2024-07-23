package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utilities.SwerveModule;

public class ChangeTurboModeCommand extends Command{
    @Override
          public void initialize() {
            SwerveModule.setPowerRatio(SwerveModule.TURBO);
          }
    
          @Override
          public boolean isFinished() {
            return true;
          }
}
