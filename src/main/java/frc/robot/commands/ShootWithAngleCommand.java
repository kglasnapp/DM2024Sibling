package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootWithAngleCommand extends Command {
    ShooterSubsystem shooterSubsystem;
    GrabberSubsystem grabberSubsystem;
    boolean finished = false;
    double startTime = 0;
    enum State {
        START_MOTORS,
        TRIGGER,
        END
    };
    State state;

    public ShootWithAngleCommand(ShooterSubsystem shooterSubsystem,
        GrabberSubsystem grabberSubsystem) {
            this.shooterSubsystem = shooterSubsystem;
            this.grabberSubsystem = grabberSubsystem;
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
                    grabberSubsystem.grabberOut();
                    startTime = RobotController.getFPGATime() / 1000;
                    state = State.TRIGGER;
                }        
                break;
            case TRIGGER:
                if (RobotController.getFPGATime() / 1000 > startTime + 3000) {
                    grabberSubsystem.grabberOff();
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
