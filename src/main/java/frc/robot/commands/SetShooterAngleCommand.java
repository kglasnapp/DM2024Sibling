package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import static frc.robot.utilities.Util.logf;

public class SetShooterAngleCommand extends Command {

    ShooterSubsystem shooterSubsystem;
    double angle;
    boolean servoOut;

    public SetShooterAngleCommand(ShooterSubsystem shooterSubsystem, boolean servoOut, double angle) {
        this.angle = angle;
        this.shooterSubsystem = shooterSubsystem;
        this.servoOut = servoOut;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (servoOut) {shooterSubsystem.servo.setAngle(ShooterSubsystem.FLAPPER_EXPANDED_ANGLE); 
        logf("#######################set servo 130 extend  Keith\n");}
        else {shooterSubsystem.servo.setAngle(ShooterSubsystem.FLAPPER_RETRACTED_ANGLE);
        logf("#######################set servo 30 retrack Keith\n");}
        shooterSubsystem.setTiltAngle(angle);
        logf("Set Shooter Angle to %.2f, servoOut: %b\n", angle, servoOut);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}