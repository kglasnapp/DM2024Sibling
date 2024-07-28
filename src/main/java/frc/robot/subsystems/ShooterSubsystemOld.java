package frc.robot.subsystems;

import static frc.robot.Util.logf;
import static frc.robot.Util.round2;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.utilities.RunningAverage;

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
public class ShooterSubsystemOld extends SubsystemBase {

    public final static double FLAPPER_RETRACTED_ANGLE = 30;
    public final static double FLAPPER_EXPANDED_ANGLE = 130;
    private CANSparkFlex shooterLowerMotor;
    private CANSparkFlex shooterHigherMotor;
    public Servo servo;
    private CANSparkMax tiltMotor;
    private SparkLimitSwitch tiltReverseLimit;
    private SparkLimitSwitch tiltForwardLimit;
    private SparkPIDController pidControllerLowerMotor;
    private SparkPIDController pidControllerHigherMotor;
    private SparkPIDController pidControllerTiltMotor;
    private PID_MAX pid = new PID_MAX();
    private final static int OVER_CURRENT = 50;
    private int SHOOTER_MOTOR_ID1 = 21;
    private int SHOOTER_MOTOR_ID2 = 23;
    private int TILT_SHOOTER_MOTOR_ID = 24;
    private RelativeEncoder shootLowerEncoder;
    private RelativeEncoder shootHigherEncoder;
    private RelativeEncoder tiltEncoder;
    private final static double SHOOT_SPEAKER_SPEED = 1;
    private final static double SHOOT_AMP_SPEED = 2;

    public final static double DEGREES_PER_REV = 360.0; 
    public final static double GEAR_RATIO = (1.0/36.0) * ((0.55+0.438)/(11.517+11.613)) * 12/15 ;// 0.04266;

    private double desiredSpeed = 0;

    public AnalogInput noteIsIndexedSensor = new AnalogInput(0);

    public enum State {
        GO_HOME,HOMING, HOMED
    };

    public final double HOMING_SPEED = -0.4;

    public State state = State.GO_HOME;

    RunningAverage avgCurrent = new RunningAverage(5);

    public ShooterSubsystemOld(RobotContainer robotContainer) {
        servo = new Servo(1);
        shooterLowerMotor = new CANSparkFlex(SHOOTER_MOTOR_ID1, MotorType.kBrushless);
        shooterHigherMotor = new CANSparkFlex(SHOOTER_MOTOR_ID2, MotorType.kBrushless);
        tiltMotor = new CANSparkMax(TILT_SHOOTER_MOTOR_ID, MotorType.kBrushless);

        shooterLowerMotor.restoreFactoryDefaults();
        shooterHigherMotor.restoreFactoryDefaults();
        tiltMotor.restoreFactoryDefaults();
        setBrakeMode(false);
        tiltMotor.setIdleMode(IdleMode.kBrake);

        shooterLowerMotor.setSmartCurrentLimit((int) OVER_CURRENT);
        shooterHigherMotor.setSmartCurrentLimit((int) OVER_CURRENT);
        tiltMotor.setSmartCurrentLimit((int) 10);

        shootLowerEncoder = shooterLowerMotor.getEncoder();
        shootHigherEncoder = shooterHigherMotor.getEncoder();
        tiltEncoder = tiltMotor.getEncoder();

        tiltReverseLimit = tiltMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        tiltForwardLimit = tiltMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

        pidControllerLowerMotor = shooterLowerMotor.getPIDController();
        pid.PIDCoefficientsShoot(pidControllerLowerMotor);
        pid.PIDToMax();
        
        pidControllerHigherMotor = shooterHigherMotor.getPIDController();
        pid.PIDCoefficientsShoot(pidControllerHigherMotor);
        pid.PIDToMax();

        pidControllerTiltMotor = tiltMotor.getPIDController();
        pid = new PID_MAX();
        pid.PIDCoefficientsShooterTilt(pidControllerTiltMotor);
        pid.PIDToMax();
        
        setShooterPower(0.3);        
        // pid.putPidCoefficientToDashBoard();
    }

    public void setShooterPower(double value) {
        if (value > 0) {
            logf("Set shooter Motor power:%.3f\n", value);
        }
        if (DriverStation.isAutonomous()) {
            value = 1;
        }
//   value = 0;
        pidControllerHigherMotor.setReference(value * 6500,  CANSparkBase.ControlType.kSmartVelocity);
        // value = 0;
        pidControllerLowerMotor.setReference(value * 6500,  CANSparkBase.ControlType.kSmartVelocity);
    }

    public void setBrakeMode(boolean mode) {
        shooterLowerMotor.setIdleMode(mode ? IdleMode.kBrake : IdleMode.kCoast);
        logf("Brake mode: %s\n", shooterLowerMotor.getIdleMode());

        shooterHigherMotor.setIdleMode(mode ? IdleMode.kBrake : IdleMode.kCoast);
        logf("Brake mode: %s\n", shooterHigherMotor.getIdleMode());

    }

    public void setFlapAngle(double angle) {
        servo.set(angle);
    }

    public void setTiltAngle(double angle) {
        pidControllerTiltMotor.setReference((-angle / DEGREES_PER_REV) / GEAR_RATIO, ControlType.kSmartMotion);
    }

    public double getTiltAngle() {
        return -tiltEncoder.getPosition() * DEGREES_PER_REV * GEAR_RATIO;
    }

    public double getShootRotations() {
        return shootHigherEncoder.getPosition();
    }

    public double getShootCurrent() {
        return shooterLowerMotor.getOutputCurrent();
    }

    public double getShootVelocity() {
        return shootLowerEncoder.getVelocity();
    }

    public double getShootVelocity2() {
        return shootHigherEncoder.getVelocity();
    }

    public void shootAmp() {
        desiredSpeed = SHOOT_AMP_SPEED;
    }

    public void shootSpeaker() {
        desiredSpeed = SHOOT_SPEAKER_SPEED;
    }

    double lastSpeed = 0;

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        if (Robot.count % 10 == 0) {
            SmartDashboard.putBoolean("ShtRev", tiltReverseLimit.isPressed());
            SmartDashboard.putBoolean("ShtFWD", tiltForwardLimit.isPressed());
            SmartDashboard.putString("ShtSts", state.toString());
            SmartDashboard.putNumber("ShtPos", tiltEncoder.getPosition());
            SmartDashboard.putNumber("Tilt", getTiltAngle());               
            SmartDashboard.putNumber("Index", noteIsIndexedSensor.getVoltage());                    
        }

        if (state == State.GO_HOME) {
            tiltMotor.set(HOMING_SPEED);
            state = State.HOMING;
            setShooterPower(0.3);    
        } else if (state == State.HOMING) {
            if (tiltReverseLimit.isPressed()) {
                state = State.HOMED;
                tiltMotor.set(0);
                tiltEncoder.setPosition(0);
                servo.setAngle(FLAPPER_RETRACTED_ANGLE); // Retracked
            }
        }

        //if (RobotContainer.testMode)
         //   return;

        double current = getShootCurrent();
        if (Robot.count % 30 == 5) {
            SmartDashboard.putNumber("ShootCur", round2(current));
            SmartDashboard.putNumber("ShootPos", round2(getShootRotations()));
            SmartDashboard.putNumber("ShootVel", round2(getShootVelocity()));
        }

        

        // if (lastSpeed != desiredSpeed) {
        //     lastSpeed = desiredSpeed;
        //     //shooterLowerMotor.set(desiredSpeed);
        //     shooterHigherMotor.set(desiredSpeed);
        // }
    }
}
