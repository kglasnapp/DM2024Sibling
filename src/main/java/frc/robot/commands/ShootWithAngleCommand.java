package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TiltSubsystem;

public class ShootWithAngleCommand extends Command {
    ShooterSubsystem shooterSubsystem;
    TiltSubsystem tiltSubsystem;

    boolean finished = false;
    double startTime = 0;

    enum State {
        START_MOTORS,
        TRIGGER,
        END
    };

    State state;

    public ShootWithAngleCommand(ShooterSubsystem shooterSubsystem, TiltSubsystem tiltSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.tiltSubsystem = tiltSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        state = State.START_MOTORS;
        finished = false;
        tiltSubsystem.setTiltAngle(0);
        shooterSubsystem.setAllShooterPower(0.95);
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
                shooterSubsystem.stop();
                break;
        }

    }

    @Override
    public boolean isFinished() {
        return finished;
    }

}
