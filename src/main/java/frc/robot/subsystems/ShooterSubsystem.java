package frc.robot.subsystems;

import static frc.robot.Util.logf;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

/**
 * REV Smart Motion Guide
 * 
 * The SPARK MAX includes a control mode, REV Smart Motion which is used to
 * control the position of the motor, and includes a max velocity and max
 * acceleration parameter to ensure the motor moves in a smooth and predictable
 * way. This is done by generating a motion profile on the fly in SPARK MAX and
 * controlling the velocity of the motor to follow this profile.
 * 
 * Since REV Smart Motion uses the velocity to track a profile, there are only
 * two steps required to configure this mode:
 * 1) Tune a velocity PID loop for the mechanism
 * 2) Configure the smart motion parameters
 * 
 * Tuning the Velocity PID Loop
 * 
 * The most important part of tuning any closed loop control such as the
 * velocity
 * PID, is to graph the inputs and outputs to understand exactly what is
 * happening.
 * For tuning the Velocity PID loop, at a minimum we recommend graphing:
 *
 * 1) The velocity of the mechanism (‘Process variable’)
 * 2) The commanded velocity value (‘Setpoint’)
 * 3) The applied output
 *
 * This example will use ShuffleBoard to graph the above parameters. Make sure
 * to
 * load the shuffleboard.json file in the root of this directory to get the full
 * effect of the GUI layout.
 */
public class ShooterSubsystem extends SubsystemBase {

    private ShootMotor lowerMotor;
    private ShootMotor upperMotor;
    private PID_MAX pid = new PID_MAX();
    private final static int OVER_CURRENT = 30;
    private int SHOOTER_MOTOR_ID1 = 9;
    private int SHOOTER_MOTOR_ID3 = 10;

    public ShooterSubsystem() {
        this.upperMotor = new ShootMotor(SHOOTER_MOTOR_ID1, false);
        this.lowerMotor = new ShootMotor(SHOOTER_MOTOR_ID3, true);

    }

    class ShootMotor {
        CANSparkFlex motor;
        SparkPIDController pidController;
        RelativeEncoder encoder;
        int id;

        ShootMotor(int id, boolean inverted) {
            this.id = id;
            motor = new CANSparkFlex(id, MotorType.kBrushless);
            motor.restoreFactoryDefaults();
            motor.setInverted(inverted);
            setBrakeMode(false);

            motor.setSmartCurrentLimit((int) OVER_CURRENT);
            encoder = motor.getEncoder();

            pidController = motor.getPIDController();
            pid.PIDCoefficientsShoot(pidController);
            pid.PIDToMax();

            setShooterVelocity(0);
        }

        public void setShooterVelocity(double value) {
            if (value >= 0) {
                logf("Set shooter velocity:%.3f\n", value);
            }
            pidController.setReference(value * 6500, CANSparkBase.ControlType.kSmartVelocity);

        }

        double getVelocity() {
            return motor.getEncoder().getVelocity();
        }

        double getPosition() {
            return motor.getEncoder().getPosition();
        }

        void setSpeed(double power) {
            motor.set(power);
        }

        void setBrakeMode(boolean mode) {
            motor.setIdleMode(mode ? IdleMode.kBrake : IdleMode.kCoast);
            logf("For id:%d Brake mode:%s\n", id, motor.getIdleMode());
        }
    }

    public void setAllShooterPower(double power) {
        logf("speedPercentage: %.2f", power);
        upperMotor.setSpeed(power);
        lowerMotor.setSpeed(power);
        // upperMotor.setShooterVelocity(power);
        // lowerMotor.setShooterVelocity(power);
    }

    public void stop() {
        upperMotor.setSpeed(0);
        lowerMotor.setSpeed(0);
    }

    public boolean isShooterAtSpeed(double desired) {
        double upSpeed = Math.abs(upperMotor.getVelocity());
        double lowSpeed = Math.abs(lowerMotor.getVelocity());
       // logf("Shooter speed up: %.1f low:%.1f desired:%.2f\n", upSpeed, lowSpeed, desired);
       if (upSpeed > desired && lowSpeed > desired) {
            logf("Shooter speed up: %.1f low:%.1f\n", upSpeed, lowSpeed);
            return true;
        }
        if (Robot.count % 2 == 0) {
            // logf("Shooter speed up: %.1f low:%.1f\n", upSpeed, lowSpeed);
        }
        return false;

    }



    // int lastPOV = -1;

    @Override
    public void periodic() {
        // if (robot.count % 5 == 0) {
        SmartDashboard.putNumber("Motor Up", upperMotor.getVelocity());
        SmartDashboard.putNumber("Motor Down", lowerMotor.getVelocity());
        SmartDashboard.putNumber("Motor Diff", upperMotor.getVelocity() - lowerMotor.getVelocity());
        SmartDashboard.putNumber("Motor Pos", lowerMotor.getPosition());
        // }

        // int pov = RobotContainer.operatorController.getHID().getPOV();
        // double value = 0;
        // if ((lastPOV != pov) && (pov >= 0)) {
        // if (pov == 0) {
        // value = 0.0;
        // }
        // if (pov == 90) {
        // value = 0.1;
        // }
        // if (pov == 180) {
        // value = 0.6;
        // }
        // if (pov == 270) {
        // value = 0.9;
        // }
        // upperMotor.setSpeed(value);
        // lowerMotor.setSpeed(value);
        // lastPOV = pov;
        // }
    }
}