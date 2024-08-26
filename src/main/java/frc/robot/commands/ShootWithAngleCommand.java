package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootWithAngleCommand extends Command {
    ShooterSubsystem shooterSubsystem;

    boolean finished = false;
    double startTime = 0;
    enum State {
        START_MOTORS,
        TRIGGER,
        END
    };
    State state;

    public ShootWithAngleCommand(ShooterSubsystem shooterSubsystem) {
            this.shooterSubsystem = shooterSubsystem;
        }

    @Override
    public void initialize() {
        state = State.START_MOTORS;
        finished = false;
        shooterSubsystem.setTiltAngle(0);        
        shooterSubsystem.setShooterPower(0.95);
        startTime = RobotController.getFPGATime() / 1000;
    }

    @Override
    public void execute() {
        switch (state) {
            case START_MOTORS:
                if (RobotController.getFPGATime() / 1000 > startTime + 1000) {
                    startTime = RobotController.getFPGATime() / 1000;
                    state = State.TRIGGER;
                }        
                break;
            case TRIGGER:
                if (RobotController.getFPGATime() / 1000 > startTime + 3000) {
                    state = State.END;
                }
                break;
            case END:
                shooterSubsystem.setShooterPower(0);                
                break;
        }
        
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

}
