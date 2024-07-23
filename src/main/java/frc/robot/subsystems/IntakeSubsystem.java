package frc.robot.subsystems;

import static frc.robot.Util.logf;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;


public class IntakeSubsystem extends SubsystemBase {
    private static final int GRABBER_INTAKE_MOTOR_ID = 20;
    private CANSparkMax intakeMotor;
    private RelativeEncoder intakeEncoder;

    private SparkPIDController pidController;
    private SparkLimitSwitch forwardLimit;
    public SparkLimitSwitch reverseLimit;
    private PID_MAX pid = new PID_MAX();

    public enum State {
        GO_HOME, HOMING, HOMED_COMPLETE
    }

    public final static double OUT_POSITION = 49;
    public final static double IN_POSITION = 0;
    public final static double HOMING_SPEED = -0.4;
    public State state = State.GO_HOME;

    public IntakeSubsystem() {
        // if (!RobotContainer.testMode) {
        // Setup parameters for the intake motor
        intakeMotor = new CANSparkMax(GRABBER_INTAKE_MOTOR_ID, MotorType.kBrushless);
        intakeEncoder = intakeMotor.getEncoder();
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setSmartCurrentLimit(30);
        forwardLimit = intakeMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        reverseLimit = intakeMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        setBrakeMode(true);
        

        // The following are needed if running in current mode
        pidController = intakeMotor.getPIDController();
        pid = new PID_MAX();
        pid.PIDCoefficientsTiltSmart(pidController);
        pid.PIDToMax();
        // TODO Disable SmartDash Board update
        // pid.putPidCoefficientToDashBoard();
    }

    public double getPosition() {
        return intakeEncoder.getPosition();
    }

    public void stop() {
        intakeMotor.set(0);
    }

    public void setBrakeMode(boolean mode) {
        intakeMotor.setIdleMode(mode ? IdleMode.kBrake : IdleMode.kCoast);
        logf("Intake Brake mode: %s\n", intakeMotor.getIdleMode());
    }

    public void intakeIn() {
        pidController.setReference(IN_POSITION, CANSparkBase.ControlType.kSmartMotion);
        // intakeMotor.set(HOMING_SPEED);
    }

    public void intakeOut() {
        pidController.setReference(OUT_POSITION, CANSparkBase.ControlType.kSmartMotion);
        // intakeMotor.set(-HOMING_SPEED);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        // if (RobotContainer.testMode) {
        // return;
        // }
        if (Robot.count % 20 == 0) {
            // System.out.println("state = " + state);
            // System.out.println("forward = " + forwardLimit.isPressed());
            // System.out.println("reverse = " + reverseLimit.isPressed());
            SmartDashboard.putBoolean("IntkRev", reverseLimit.isPressed());
            SmartDashboard.putBoolean("IntkFWD", forwardLimit.isPressed());
            SmartDashboard.putString("IntkSts", state.toString());
            SmartDashboard.putNumber("IntkPos", intakeEncoder.getPosition());
        }
        if (state == null) {
            state = State.GO_HOME;
        }
        if (state == State.GO_HOME) {
            intakeMotor.set(HOMING_SPEED);
            state = State.HOMING;
        }
        if (state == State.HOMING) {
            if (reverseLimit.isPressed()) {
                System.out.println("Reverse Limit press");
                state = State.HOMED_COMPLETE;
                intakeMotor.getEncoder().setPosition(0);
                intakeMotor.set(0);
            }

        }
    }

}
