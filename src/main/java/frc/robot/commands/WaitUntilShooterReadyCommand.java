package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class WaitUntilShooterReadyCommand extends Command {
    
    ShooterSubsystem shooterSubsystem;
    
    public WaitUntilShooterReadyCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }   

    @Override
    public void initialize() {
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.getShootVelocity() > 3500 &&
                    shooterSubsystem.getShootVelocity2() > 3500;
    }
}
