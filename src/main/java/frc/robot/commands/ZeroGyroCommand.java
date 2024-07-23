package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ZeroGyroCommand extends Command {
    DrivetrainSubsystem m_drivetrainSubsystem;
    BalanceCommand balanceCommand;
    double currentOrientation;

    public ZeroGyroCommand(DrivetrainSubsystem drivetrainSubsystem, BalanceCommand balanceCommand, double currentOrientation) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.currentOrientation = currentOrientation;
        this.balanceCommand = balanceCommand;
    }

    @Override
    public void execute() {
        m_drivetrainSubsystem.zeroGyroscope(currentOrientation);
        balanceCommand.zeroGyroscope();
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
