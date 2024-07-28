package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystemOld;
import static frc.robot.utilities.Util.logf;

public class SetShooterAngleCommand extends Command {

    ShooterSubsystemOld shooterSubsystem;
    double angle;
    boolean servoOut;

    public SetShooterAngleCommand(ShooterSubsystemOld shooterSubsystem, boolean servoOut, double angle) {
        this.angle = angle;
        this.shooterSubsystem = shooterSubsystem;
        this.servoOut = servoOut;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (servoOut) {shooterSubsystem.servo.setAngle(ShooterSubsystemOld.FLAPPER_EXPANDED_ANGLE); 
        logf("#######################set servo 130 extend  Keith\n");}
        else {shooterSubsystem.servo.setAngle(ShooterSubsystemOld.FLAPPER_RETRACTED_ANGLE);
        logf("#######################set servo 30 retrack Keith\n");}
        shooterSubsystem.setTiltAngle(angle);
        logf("Set Shooter Angle to %.2f, servoOut: %b\n", angle, servoOut);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}