package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class IndexerSubsystem extends SubsystemBase {
    public static double SHOOT_SPEED = 0.9;
    public static double INTAKE_SPEED = 0.7;

    public static int INDEXER_MOTOR_ID = 11;

    private TalonFX indexerMotor = new TalonFX(INDEXER_MOTOR_ID);
    private final double CURRENT_LIMIT = 60;
    private final LedSubsystem leds;

    private DigitalInput notePresent = new DigitalInput(2);

    public IndexerSubsystem(LedSubsystem leds) {
        setConfig(indexerMotor);
        this.leds = leds;
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

    public boolean isNotePresent() {
        return !notePresent.get();
    }

    public void setSpeed(double speed) {
        indexerMotor.set(speed);
    }

    public void stop() {
        indexerMotor.set(0);
    }

    @Override
    public void periodic() {
        if (Robot.count % 10 == 4) {
            SmartDashboard.putBoolean("Note", isNotePresent());
        }

        //if (Robot.count % 5 == 0) {
            leds.setNoteState(isNotePresent());
        //}
    }
}
