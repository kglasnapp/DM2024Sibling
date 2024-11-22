package frc.robot.commands;

import static frc.robot.utilities.Util.logf;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.utilities.SwerveModule;

public class DefaultDriveCommand extends Command {

    /**
     * If the robot is running any automation, this property is set to true.
     * In that case, we don't change the Motor Mode (TURBO or NORMAL) when the
     * user presses the controller. We leave it in the state that the autonomous
     * command left it.
     */
    public static boolean autonomous;

    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final BooleanSupplier precisionActivator;
    private final BooleanSupplier robotOrientedActivator;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier,
            BooleanSupplier precisionActivator, BooleanSupplier robotOrientedActivator) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.precisionActivator = precisionActivator;
        this.robotOrientedActivator = robotOrientedActivator;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // Currently implemented with separate commands
        if (precisionActivator.getAsBoolean()) {
            SwerveModule.setPowerRatio(SwerveModule.PRECISION);
        } else {
            SwerveModule.setPowerRatio(SwerveModule.TURBO);
        }

        if (Robot.count % 20 == 0) {
            if (m_translationXSupplier.getAsDouble() != 0 &&
                    m_translationYSupplier.getAsDouble() != 0) {
                logf("Robot Oriented Speed X: %.2f y:%.2f angle:%.2f\n", m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(), m_rotationSupplier.getAsDouble());
            }
        }

        // this allows for Robot Oriented driving
        if (robotOrientedActivator.getAsBoolean()) {
            m_drivetrainSubsystem.drive(
                    new ChassisSpeeds(
                            m_translationXSupplier.getAsDouble(),
                            m_translationYSupplier.getAsDouble(),
                            m_rotationSupplier.getAsDouble()));

        } else {
            m_drivetrainSubsystem.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            m_translationXSupplier.getAsDouble(),
                            m_translationYSupplier.getAsDouble(),
                            m_rotationSupplier.getAsDouble(),
                            m_drivetrainSubsystem.getGyroscopeRotation()));
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        SwerveModule.setPowerRatio(SwerveModule.DEFAULT);
    }
}
