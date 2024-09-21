package frc.robot.subsystems;

import static frc.robot.Util.logf;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class IntakeSubsystem extends SubsystemBase {
    private IntakeMotor motor;

    private final static int OVER_CURRENT = 30;
    private int INTAKE_MOTOR_ID = 21;

    public IntakeSubsystem() {
        this.motor = new IntakeMotor(INTAKE_MOTOR_ID, false);
    }

    public void intakeIn() {
        motor.setSpeed(.9);
    }

    public void stop() {
        motor.setSpeed(0);
    }

    class IntakeMotor {
        CANSparkFlex motor;
        SparkPIDController pidController;
        RelativeEncoder encoder;
        int id;

        IntakeMotor(int id, boolean inverted) {
            this.id = id;
            motor = new CANSparkFlex(id, MotorType.kBrushless);
            motor.restoreFactoryDefaults();
            motor.setInverted(inverted);
            setBrakeMode(false);
            motor.setSmartCurrentLimit((int) OVER_CURRENT);
            encoder = motor.getEncoder();
            setSpeed(0);
        }

        double getVelocity() {
            return motor.getEncoder().getVelocity();
        }

        public double getMotorCurrent() {
            return motor.getOutputCurrent();
        }

        double getPosition() {
            return motor.getEncoder().getPosition();
        }

        void setSpeed(double power) {
            motor.set(power);
        }

        void setBrakeMode(boolean mode) {
            motor.setIdleMode(mode ? IdleMode.kBrake : IdleMode.kCoast);
            logf("For Intake id:%d Brake mode:%s\n", id, motor.getIdleMode());
        }
    }

    int lastPOV = -1;

    @Override
    public void periodic() {
        if (Robot.count % 5 == 0) {
            SmartDashboard.putNumber("Intake Speed", motor.getVelocity());
        }
    }

    public double getMotorCurrent() {
       return motor.getMotorCurrent();
    }

}
