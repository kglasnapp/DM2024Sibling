package frc.robot.subsystems;

//import static frc.robot.Util.logf;

import static frc.robot.utilities.Util.logf;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import frc.robot.RobotContainer;

public class ClimberSubsystem extends SubsystemBase {
    // Defines for Hardware
    private final static int LEFT_CLIMBER_MOTOR_ID = 30;
    private final static int RIGHT_CLIMBER_MOTOR_ID = 31;
    private final static int LEFT_SERVO_ID = 2;
    private final static int RIGHT_SERVO_ID = 1;

    // Defines for speeds
    // Speed for down left -, right +
    // Speed for up left +, right -
    private final static double SPEED = 0.8;
    private final static double releaseLatchSpeed = .1;
    private final static double homingSpeed = .5;
    private double lastLeftSpeed = 0;
    private double lastRightSpeed = 0;

    // Variables for state machine
    private enum State {
        IDLE, UNLOCK, HOMING, HOMED, UNLOCK2, OPEERATING
    }

    // private State state = State.IDLE;
    private State state = State.HOMED; // TODO state should be idle when Homing works
    private State lastState = State.IDLE;
    private int count = 0;

    // Setup the objects for the hardware
    private TalonFX leftClimber = new TalonFX(LEFT_CLIMBER_MOTOR_ID);
    private TalonFX rightClimber = new TalonFX(RIGHT_CLIMBER_MOTOR_ID);
    private Servo leftClimberServo = new Servo(LEFT_SERVO_ID);
    private Servo rightClimberServo = new Servo(RIGHT_SERVO_ID);
    private int lockCounter = -1;

    private final double CURRENT_LIMIT = 20;
    private final double unLockAngle = 0;
    private final double lockAngle = .12;
    private final double maxHeight = 380;

    // Called when robot is started
    public ClimberSubsystem() {
        logf("Climber Subsystem Initialized\n");
        setConfig(leftClimber);
        setConfig(rightClimber);
        lockServos(6); // Set to locked
        setEncoderPositions(0);
    }

    // Setup parameters for the climber motors
    private void setConfig(TalonFX talon) {
        // TODO talon.getConfigurator().apply(new TalonFXConfiguration());
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

    // Called when the robot is enabled
    public void homeClimber() {
        // Prepare to release the latch
        state = State.UNLOCK;
        unlockServos();
        // Move motors down to free latch
        setRightSpeed(releaseLatchSpeed);
        setLeftSpeed(-releaseLatchSpeed);
        setEncoderPositions(0);
        count = 5;
    }

    // Called when the robot is disabled, shut down motors and release SERVO
    public void disableRobot() {
        logf("Robot is being disabled\n");
        setRightSpeed(0);
        setLeftSpeed(0);
        lockServos(5);
        // todo: we need to remove/rethink the state we put the robot when disabled
        state = State.HOMED;
    }

    @Override
    public void periodic() {
        double rightCurrent = getCurrent(rightClimber);
        double leftCurrent = getCurrent(leftClimber);
        if (Robot.count % 10 == 0) {
            SmartDashboard.putNumber("RightCCur", rightCurrent);
            SmartDashboard.putNumber("LeftCCur", leftCurrent);
            SmartDashboard.putNumber("RightCPOS", getEncoderPosition(rightClimber));
            SmartDashboard.putNumber("leftCPOS", getEncoderPosition(leftClimber));
            SmartDashboard.putNumber("lockC", lockCounter);
            SmartDashboard.putString("CState", state.toString());
        }
        if (lockCounter == 0) {
            // setServoPositions(lockAngle);
            // SmartDashboard.putNumber("SelPow". .5);
            //unlockServos();
            lockCounter = -1;
        } else if (lockCounter > 0) {
            lockCounter--;
        }

        if (state != lastState) {
            logf("Climber new state:%s speeds r:%.2f l:%.2f currents r:%.2f l:%.2f encouders: left: %.2f right: %.2f\n",
                    state, lastRightSpeed,
                    lastLeftSpeed, rightCurrent, leftCurrent, getEncoderPosition(leftClimber),
                    getEncoderPosition(rightClimber));
            lastState = state;
        }
        count--;
        switch (state) {
            case IDLE:
                break;
            case UNLOCK:
                // After robot enabled and 100 ms passes home the climber
                if (count > 0) {
                    return;
                }
                state = State.HOMING;
                // Move climber down at homing speed
                setRightSpeed(homingSpeed);
                setLeftSpeed(-homingSpeed);
                count = 50 * 4;
                break;
            case HOMING:
                // If after 4 seconds homing not complete stop it
                if (count < -50 * 2) {
                    logf("Climber Homing timed out release SERVOs and stop motors\n");
                    state = State.HOMED;
                    lockServos(3);
                    setRightSpeed(0);
                    setLeftSpeed(0);
                    setEncoderPositions(0);
                    return;
                }
                // After 2 seconds check currents and if too high stop homing
                if (count > 190) {
                    return;
                }
                logf("Homing Current right:%.2f left:%.2f\n", rightCurrent, leftCurrent);
                if (rightCurrent > 5) {
                    setRightSpeed(0);
                }
                if (leftCurrent > 5) {
                    setLeftSpeed(0);
                }
                if (leftClimber.get() == 0 && rightClimber.get() == 0) {
                    state = State.HOMED;
                    setEncoderPositions(0);
                    lockServos(2);
                }
                break;
            case HOMED:
                // If driver wants to move either climber up
                // operated SERVO and move climbers down for 100 ms
                if (Math.abs(RobotContainer.operatorHID.getLeftY()) > .5
                        || Math.abs(RobotContainer.operatorHID.getRightY()) > .5) {

                    logf("Climb Button Hit\n");
                    unlockServos();
                    setRightSpeed(releaseLatchSpeed);
                    setLeftSpeed(-releaseLatchSpeed);
                    count = 5;
                    state = State.UNLOCK2;
                }
                break;
            case UNLOCK2:
                if (count > 0) {
                    return;
                }
                state = State.OPEERATING;
                count = 50 * 10;
                break;
            case OPEERATING:
                if (count < 0) {
                    // After 10 seconds of no movement
                    // Release SERVOs, turn off motors and move to home state
                    lockServos(1);
                    setRightSpeed(0);
                    setLeftSpeed(0);
                    state = State.HOMED;
                    return;
                }
                // If motor speed changed then reset the time delay
                if (controlMotors()) {
                    count = 50;
                }
                break;
        }
    }

    // private void setServoPositions(double angle) {
    // logf("Set Servo Position angle:%.2f\n", angle);
    // leftClimberServo.setPosition(angle);
    // rightClimberServo.setPosition(angle);
    // }

    private double getCurrent(TalonFX motor) {
        return Math.abs(motor.getTorqueCurrent().getValue());
    }

    boolean getForwardLimitSwitch(TalonFX motor) {
        return motor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }

    boolean getReverseLimitSwitch(TalonFX motor) {
        return motor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

    private boolean controlMotors() {
        double leftSpeed = 0;
        double rightSpeed = 0;
        if (RobotContainer.operatorHID.getRightY() > 0.5) {
            leftSpeed = -SPEED; // Move climber down
        }
        if (RobotContainer.operatorHID.getLeftY() > 0.5) {
            rightSpeed = SPEED; // Move climber down
        }
        if (RobotContainer.operatorHID.getRightY() < -.5) {
            leftSpeed = SPEED; // Move climber up
        }
        if (RobotContainer.operatorHID.getLeftY() < -.5) {
            rightSpeed = -SPEED; // Move climber up
        }
        if (Math.abs(getEncoderPosition(leftClimber)) > maxHeight &&
            RobotContainer.operatorHID.getLeftX() < .5) {
            leftSpeed = 0;
        }
        if (Math.abs(getEncoderPosition(rightClimber)) > maxHeight &&
            RobotContainer.operatorHID.getRightX() < .5) {
            rightSpeed = 0;
        }
        setLeftSpeed(leftSpeed);
        setRightSpeed(rightSpeed);
        boolean running = leftSpeed != 0 || rightSpeed != 0;
        if (running) {
            logf("LeftSpeed:%.2f RightSpeed:%.2f leftY:%.2f right:%.2f\n", leftSpeed, rightSpeed, RobotContainer.operatorHID.getLeftY(),
                    RobotContainer.operatorHID.getRightY());
        }
        return running;

    }

    private void lockServos(int code) {
        logf("lockServos code:%d\n", code);
        controlSERVOs(false);
    }

    private void unlockServos() {
        controlSERVOs(true);
    }

    private void controlSERVOs(boolean action) {
        if (action) {
            leftClimberServo.setPosition(unLockAngle);
            rightClimberServo.setPosition(lockAngle);
            if (lockCounter == -1) {
                lockCounter = 100;
            }
        } else {
            leftClimberServo.setPosition(lockAngle);
            rightClimberServo.setPosition(unLockAngle);
        }
        SmartDashboard.putBoolean("SERVOs", action);
        logf("Climber SERVOs:%b\n", action);
    }

    private void setRightSpeed(double speed) {
        if (lastRightSpeed != speed) {
            rightClimber.set(speed);
            lastRightSpeed = speed;
            logf("New Right Climber Speed:%.2f\n", speed);
            SmartDashboard.putNumber("RightCPOS", getEncoderPosition(rightClimber));
        }
    }

    private void setLeftSpeed(double speed) {
        if (lastLeftSpeed != speed) {
            leftClimber.set(speed);
            lastLeftSpeed = speed;
            logf("New Left Climber Speed:%.2f\n", speed);
            SmartDashboard.putNumber("LeftCPOS", getEncoderPosition(leftClimber));
        }
    }

    void setEncoderPositions(double position) {
        rightClimber.setPosition(position);
        leftClimber.setPosition(position);
    }

    double getEncoderPosition(TalonFX motor) {
        return motor.getPosition().getValue();
    }
}
