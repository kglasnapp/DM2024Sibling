package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ShooterSubsystem  extends SubsystemBase {

    public static int INDEXER_MOTOR_ID = 11;
    private double lastSpeed;

    private TalonFX indexerMotor = new TalonFX(INDEXER_MOTOR_ID);
    private final double CURRENT_LIMIT = 60;

    public ShooterSubsystem() {
        setConfig(indexerMotor);
    }

    private void setConfig(TalonFX talon) {
        TalonFXConfiguration configuration = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits = currentLimits
                .withStatorCurrentLimit(CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyTimeThreshold(100);
        talon.getConfigurator().apply(configuration.withCurrentLimits(currentLimits));
        talon.setNeutralMode(NeutralModeValue.Brake);
        configuration.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
        configuration.HardwareLimitSwitch.ForwardLimitEnable = true;
        configuration.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        configuration.HardwareLimitSwitch.ReverseLimitEnable = true;
    }

    @Override
    public void periodic() {
        if (RobotContainer.driveController.getHID().getAButtonPressed()) {
            indexerMotor.set(0.75);
        } 
        if (RobotContainer.driveController.getHID().getBButtonReleased()) {
            indexerMotor.set(0);
        }
        if (RobotContainer.driveController.getHID().getBButtonPressed()) {
            indexerMotor.set(-0.75);
        } 
        if (RobotContainer.driveController.getHID().getAButtonReleased()) {
            indexerMotor.set(0);
        }
    }
}
