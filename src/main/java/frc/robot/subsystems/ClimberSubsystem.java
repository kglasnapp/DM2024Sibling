package frc.robot.subsystems;

import static frc.robot.Util.logf;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

//import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ClimberSubsystem extends SubsystemBase {
    // Defines for Hardware
    private final static int LEFT_CLIMBER_MOTOR_ID = 30;
    private final static int RIGHT_CLIMBER_MOTOR_ID = 31;
    //private final static int LEFT_SOLENOID_ID = 2;
    //private final static int RIGHT_SOLENOID_ID = 3;

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

    private State state = State.IDLE;
    private State lastState = State.IDLE;
    private int count = 0;

    // Setup the objects for the hardware
    private TalonFX leftClimber = new TalonFX(LEFT_CLIMBER_MOTOR_ID);
    private TalonFX rightClimber = new TalonFX(RIGHT_CLIMBER_MOTOR_ID);
    //private DigitalOutput leftClimberSolenoid = new DigitalOutput(LEFT_SOLENOID_ID);
    //private DigitalOutput rightClimberSoleniod = new DigitalOutput(RIGHT_SOLENOID_ID);

    private int lockCounter = -1;
    private PWM climberLock = new PWM(4);

    // TODO Elie and KAG I thought that we had two climber locks one for right and one for left
    
    private final double CURRENT_LIMIT = 60;

    // Called when robot is started
    public ClimberSubsystem() {
        setConfig(leftClimber);
        setConfig(rightClimber);
        // Solenoids will be set true when Robot is enabled
        controlSolenoids(false);
    }

    // Setup parameters for the climber motors
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

    // Called when the robot is enabled
    public void homeClimber() {
        // Prepare to release the latch
        state = State.UNLOCK;
        controlSolenoids(true);
        // Move motors down to free latch
        setRightSpeed(releaseLatchSpeed);
        setLeftSpeed(-releaseLatchSpeed);
        setEncoderPositions(0);
        count = 5;
    }

    // Called when the robot is disabled, shut down motors and release solenoid
    public void disableRobot() {
        setRightSpeed(0);
        setLeftSpeed(0);
        controlSolenoids(false);
    }

    @Override
    public void periodic() {
        // if (true) {
        // toggleSolenoid(); // For testing only
        // return;
        // }
        double rightCurrent = getCurrent(rightClimber);
        double leftCurrent = getCurrent(leftClimber);
        if (Robot.count % 10 == 0) {
            SmartDashboard.putNumber("RightCCur", rightCurrent);
            SmartDashboard.putNumber("LeftCCur", leftCurrent);
            SmartDashboard.putNumber("RightCPOS", getEncoderPosition(rightClimber));
            SmartDashboard.putNumber("leftCPOS", getEncoderPosition(leftClimber));
        }
        if (lockCounter == 0) {
            climberLock.setSpeed(0.4);
            SmartDashboard.putNumber("SelPow",4);
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
                    logf("Climber Homing timed out release solenoids and stop motors\n");
                    state = State.HOMED;
                    controlSolenoids(false);
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
                    controlSolenoids(false);
                }
                break;
            case HOMED:
                // If driver wants to move either climber up
                // operated solenoid and move climbers down for 100 ms
                if (RobotContainer.operatorHID.getLeftBumper()
                        || RobotContainer.operatorHID.getRightBumper()
                        || RobotContainer.operatorHID.getLeftTriggerAxis() > 0.1
                        || RobotContainer.operatorHID.getRightTriggerAxis() > 0.1) {
                    controlSolenoids(true);
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
                    // Release solenoids, turn off motors and move to home state
                    controlSolenoids(false);
                    setRightSpeed(0);
                    setLeftSpeed(0);
                    state = State.HOMED;
                    return;
                }
                // If motor speed changed then reset the time delay
                if (controlMotors()) {
                    count = 50 * 10;
                }
                break;
        }
    }

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
        if (RobotContainer.operatorController.getLeftTriggerAxis() > 0.1) {
            leftSpeed = -SPEED; // Move climber down
        }
        if (RobotContainer.operatorController.getRightTriggerAxis() > 0.1) {
            rightSpeed = SPEED; // Move climber down
        }
        if (RobotContainer.getOperatorLeftBumper()) {
            leftSpeed = SPEED; // Move climber up
        }
        if (RobotContainer.getOperatorRightBumper()) {
            rightSpeed = -SPEED; // Move climber up
        }
        if (Math.abs(getEncoderPosition(leftClimber)) > 80 && RobotContainer.getOperatorLeftBumper()) {
            leftSpeed = 0;

        }
        if (Math.abs(getEncoderPosition(rightClimber)) > 80 && RobotContainer.getOperatorRightBumper()) {
            rightSpeed = 0;
        }
        setLeftSpeed(leftSpeed);
        setRightSpeed(rightSpeed);
        return leftSpeed != 0 || rightSpeed != 0;

    }

    private void controlSolenoids(boolean action) {
        if (action) {
            climberLock.setSpeed(1);
            SmartDashboard.putNumber("SelPow",1);
            if (lockCounter == -1) {
                lockCounter = 100;
            }
        } else {
            climberLock.setSpeed(0);
            SmartDashboard.putNumber("SelPow",0);
        }
        // rightClimberSoleniod.set(action);
        // leftClimberSolenoid.set(action);
        SmartDashboard.putBoolean("Solenoids", action);
        logf("Climber Solenoids:%b\n", action);
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

    // private boolean lastStart = false;
    // private void toggleSolenoid() {
    // boolean start = RobotContainer.driveController.getHID().getStartButton();
    // if (start != lastStart) {
    // controlSolenoids(start);
    // // lastSolenoid = !lastSolenoid;
    // lastStart = start;
    // }
    // }

}
