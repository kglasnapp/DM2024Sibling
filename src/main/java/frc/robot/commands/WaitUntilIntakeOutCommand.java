package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class WaitUntilIntakeOutCommand extends Command {
    
    IntakeSubsystem intakeSubsystem;

    public WaitUntilIntakeOutCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.getPosition() >= IntakeSubsystem.OUT_POSITION - 5;
    }
}
