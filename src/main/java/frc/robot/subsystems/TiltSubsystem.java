package frc.robot.subsystems;

import static frc.robot.utilities.Util.logd;
import static frc.robot.utilities.Util.logf;
import static frc.robot.utilities.Util.round2;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
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
public class TiltSubsystem extends SubsystemBase {
    private static int TILT_SHOOTER_CAN_CODER_ID = 9;

    private CANSparkFlex tiltMotor;
    private SparkLimitSwitch tiltReverseLimit;
    private SparkLimitSwitch tiltForwardLimit;
    private SparkPIDController pidControllerTiltMotor;
    private PID_MAX pid = new PID_MAX();
    private int TILT_SHOOTER_MOTOR_ID = 20;
    private RelativeEncoder tiltEncoder;
    private CANcoder angleEncoder;
    private double lastRotations = 0;
    private final static double DEGREES_PER_REV = 2.9;
    private int myCount = 0;
    private int overCurrentCount = 0;
    private final double maxAngle = 96.5;
    private final double minAngle = 0;
    private final double angleTolerance = 1;

    public enum State {
        IDLE, GO_HOME, HOMING, WAIT1, WAIT2, HOMED, STOP, MOVING
    };

    private final double HOMING_SPEED = -0.2;

    public State state = State.GO_HOME;
    private State lastState = State.IDLE;
    private double desiredAngle = 0;

    RunningAverage avgCurrent = new RunningAverage(5);

    public TiltSubsystem() {
        tiltMotor = new CANSparkFlex(TILT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
        tiltMotor.restoreFactoryDefaults();
        tiltMotor.setIdleMode(IdleMode.kBrake);
        tiltMotor.setSmartCurrentLimit((int) 40);
        tiltEncoder = tiltMotor.getEncoder();
        tiltEncoder.setPosition(0);
        lastRotations = 0;
        tiltReverseLimit = tiltMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        tiltForwardLimit = tiltMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        pidControllerTiltMotor = tiltMotor.getPIDController();
        pid = new PID_MAX();
        pid.PIDCoefficientsShooterTilt(pidControllerTiltMotor);
        pid.PIDToMax();
        
        angleEncoder = new CANcoder(TILT_SHOOTER_CAN_CODER_ID);
        // logf("Startup for the tilt subsystem id:%\n", TILT_SHOOTER_MOTOR_ID);
    }

    public void homeTilt() {
        logd("Home Tilt\n");
        state = State.GO_HOME;
    }

    public void setTiltAngle(double angle) {
        if (state == State.IDLE && angle <= maxAngle && angle >= minAngle) {
            state = State.MOVING;
            lastRotations = angle / DEGREES_PER_REV;
            desiredAngle = angle;
            pidControllerTiltMotor.setReference(angle / DEGREES_PER_REV, ControlType.kSmartMotion);
        }
    }

    public void setAngle(double angle) {
        pidControllerTiltMotor.setReference(angle / DEGREES_PER_REV, ControlType.kSmartMotion);
        logf("Intended Angle: %.2f\n", angle);
    }

    // rottions should be +1 or -1
    public void setRotations(double rotations) {
        if (state == State.IDLE && (lastRotations + rotations) * DEGREES_PER_REV <= maxAngle
                && (lastRotations + rotations) * DEGREES_PER_REV >= minAngle) {
            state = State.MOVING;
            lastRotations += rotations;
            desiredAngle = lastRotations * DEGREES_PER_REV;
            pidControllerTiltMotor.setReference(lastRotations, ControlType.kSmartMotion);
        }
    }

    public double getTiltAngle() {
        return tiltEncoder.getPosition() * DEGREES_PER_REV;
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        if (Robot.count % 5 == 0) {
            tiltEncoder.setPosition(angleEncoder.getAbsolutePosition().getValueAsDouble() * 360.0 / DEGREES_PER_REV);
        }

        if (lastState != state) {
            logd("New State:%s last State:%s\n", state, lastState);
            lastState = state;
        }
        double tiltCurrent = round2(tiltMotor.getOutputCurrent());
        if (tiltCurrent > 20) {
            overCurrentCount++;
        } else {
            overCurrentCount = 0;
        }
        if (overCurrentCount > 50) {
            logf("!!!!!!!! Hit Over Current on tilt motor -- will stop it\n");
            overCurrentCount = 0;
            // tiltMotor.set(0);
        }
        // int delta = 360;
        // double val = RobotContainer.driveHID.getRightY();
        // tiltMotor.set(val * .15);

        // tiltMotor.set(val);
        SmartDashboard.putNumber("TiltAngle", round2(getTiltAngle()));
        if (Robot.count % 10 == -1) {
            SmartDashboard.putBoolean("TiltRev", tiltReverseLimit.isPressed());
            SmartDashboard.putBoolean("TiltFWD", tiltForwardLimit.isPressed());
            SmartDashboard.putString("TiltState", state.toString());
            SmartDashboard.putNumber("TiltCur", tiltCurrent);
            SmartDashboard.putNumber("TiltovCnt", overCurrentCount);
            SmartDashboard.putNumber("TiltPos", round2(tiltEncoder.getPosition()));
            SmartDashboard.putNumber("TiltAngle", round2(getTiltAngle()));
            SmartDashboard.putNumber("TiltDesAngle", round2(desiredAngle));
        }

        if (state == State.GO_HOME) {
            tiltMotor.set(HOMING_SPEED);
            state = State.HOMING;
        }
        if (state == State.HOMING) {
            // if (tiltReverseLimit.isPressed()) {
            if (tiltReverseLimit.isPressed()) {
                myCount = 5;
                state = State.WAIT1;
                tiltMotor.set(0);
            }
        }
        if (state == State.WAIT1) {
            myCount--;
            if (myCount < 0) {
                // tiltEncoder.setPosition(0);
                lastRotations = 0;
                // setTiltAngle(0);
                state = State.HOMED;
                setRotations(0);
            }
        }
        if (state == State.HOMED) {
            state = State.IDLE;
        }
        if (state == State.STOP) {
            tiltMotor.set(0);
            state = State.IDLE;
        }

        if (state == State.MOVING) {
            if (Robot.count % 15 == 3)
                logf("Tilt moving desired:%.2f actual:%.2f delta:%.2f\n", desiredAngle, getTiltAngle(),
                        desiredAngle - getTiltAngle());
            if (Math.abs(desiredAngle - getTiltAngle()) <= angleTolerance) {
                state = State.IDLE;
            }
        }
    }
}
