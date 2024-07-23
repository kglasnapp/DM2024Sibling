package frc.robot.subsystems;

import static frc.robot.Util.logf;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class GrabberSubsystem extends SubsystemBase {
    private static final int GRABBER_INTAKE_MOTOR_ID = 14;
    private double targetGrabberPower = 0;
    private double lastGrabberPower = 0;
    private WPI_TalonSRX grabberMotor;
    public static final int GRABBER_MOTOR_ID = 50;
    public CANSparkMax grabberMotorNeo;
    private SparkPIDController pidControllerGrabberMotor;
    private PID_MAX pid = new PID_MAX();
    private AnalogInput notePartiallyInGrabberSensor;
    private AnalogInput noteFullyInGrabberSensor;
    private final double defaultGrabberPowerIn = 1; // .8;
    private final double defaultGrabberPowerOut = -1; // -.8;
    private RobotContainer robotContainer;

    public boolean ignoreSensorToStop = false;

    public boolean useNeo = true;

    public GrabberSubsystem(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        // Setup parameters for the intake motor
        if (useNeo) {
            grabberMotorNeo = new CANSparkMax(GRABBER_MOTOR_ID, MotorType.kBrushless);
            grabberMotorNeo.restoreFactoryDefaults();
            grabberMotorNeo.setIdleMode(IdleMode.kBrake);
            grabberMotorNeo.setSmartCurrentLimit((int) 30);
            pidControllerGrabberMotor = grabberMotorNeo.getPIDController();
            pid = new PID_MAX();
            pid.PIDCoefficientsGrabber(pidControllerGrabberMotor);
            pid.PIDToMax();
        } else {
            grabberMotor = new WPI_TalonSRX(GRABBER_INTAKE_MOTOR_ID);
            grabberMotor.configFactoryDefault();
            grabberMotor.setSafetyEnabled(false);
            /* enabled | Limit(amp) | Trigger Threshold(amp) | Trigger Threshold Time(s) */
            // grabberMotor.configStatorCurrentLimit(new
            // StatorCurrentLimitConfiguration(true, 20, 25, 1.0));
            grabberMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
                    10, 15, 0.5));
            setBrakeMode(true);
        }
        notePartiallyInGrabberSensor = new AnalogInput(1);
        noteFullyInGrabberSensor = new AnalogInput(2);
        grabberOff();

    }

    public double indexNoteInShooter() {
        logf("Index\n");
        double position = grabberMotorNeo.getEncoder().getPosition() - 3.25;
        pidControllerGrabberMotor.setReference(position, CANSparkBase.ControlType.kSmartMotion);
        return position;
    }

    public double indexNoteInShooterSlow() {
        logf("Index slow\n");
        double position = grabberMotorNeo.getEncoder().getPosition() - 0.75;
        pidControllerGrabberMotor.setReference(position, CANSparkBase.ControlType.kSmartMotion);
        return position;
    }

    public boolean seeNote() {

        return notePartiallyInGrabberSensor.getVoltage() > 2.4;
    }

    public boolean noteFullyIn() {
        return noteFullyInGrabberSensor.getVoltage() > 1.8;
    }

    public void setBrakeMode(boolean mode) {
        if (mode) {
            grabberMotor.setNeutralMode(NeutralMode.Brake);
        } else {
            grabberMotor.setNeutralMode(NeutralMode.Coast);
        }

        logf("Intake Brake mode: %s\n", mode);
    }

    public void grabberIn() {
        logf("Setting the grabber in\n");
        setIntakePower(defaultGrabberPowerIn);
    }

    public void grabberOut() {
        logf("Setting the grabber out\n");
        setIntakePower(defaultGrabberPowerOut);
    }

    public void grabberOut(double power) {
        logf("Setting the grabber out\n");
        setIntakePower(defaultGrabberPowerOut*power);
    }

    public void grabberOff() {
        logf("Setting the grabber off\n");
        setIntakePower(0);
    }

    private void setIntakePower(double power) {
        targetGrabberPower = power;
    }

    public double getIntakePower() {
        return targetGrabberPower;
    }

    double lastCurrent = 0;
    // This method will be called once per scheduler run

    enum State {
        IDDLE,
        GRABBER_IN,
        SENSOR_FULLY_IN,
        DONE
    };

    State state;
    int counter = 0;

    int iterationsOverCurrentLimit = 0;

    @Override
    public void periodic() {
        if (Robot.count % 10 == 0) {
            SmartDashboard.putNumber("GrbOutSr", notePartiallyInGrabberSensor.getVoltage());
            SmartDashboard.putNumber("GrbInSr", noteFullyInGrabberSensor.getVoltage());        
        }            
        
        if (useNeo) {
            // when using the neo motor, and the grabber is going in
            // as soon as we see the note, we reduce the target power.
            if (targetGrabberPower > 0) {
                if (seeNote()&&targetGrabberPower==1) {
                    targetGrabberPower /= 2; 
                    logf("Cutting grabber power to half\n");
                }
            }
        }
        if (targetGrabberPower != lastGrabberPower) {
            if (useNeo) {
                logf("changing power with NEO: %.2f\n", targetGrabberPower);
                if (targetGrabberPower > 0) {
                    grabberMotorNeo.set(targetGrabberPower * 0.35);
                } else {
                    grabberMotorNeo.set(targetGrabberPower * 1);
                }
                // pidControllerGrabberMotor.setReference(targetGrabberPower * 6500,
                //         CANSparkBase.ControlType.kSmartVelocity);
            } else {
                grabberMotor.set(targetGrabberPower);
            }
            lastGrabberPower = targetGrabberPower;
        }
        if (Robot.count % 10 == 0) {
            double current = useNeo?grabberMotorNeo.getOutputCurrent():grabberMotor.getStatorCurrent();
            if (current > lastCurrent + 1 || current < lastCurrent - 1) {
                lastCurrent = current;
                // logf("grabber current = %.2f, %.2f\n", current,
                // grabberMotor.getSupplyCurrent());
            }
        }
        if (Robot.count % 20 == 0) {
            double sensorVoltage = noteFullyInGrabberSensor.getVoltage();
           // logf("sensor voltage: %.2f\n", sensorVoltage);
            // double noteFullyInVoltage = noteFullyInGrabberSensor.getVoltage();
            // logf("note fully in sensor voltage: %.2f\n", noteFullyInVoltage);
        }
        if (Robot.count % 5 == 0) {
            double currentLimit = 9;
            if (useNeo) {
                currentLimit = 30;
            }
            if ((noteFullyIn() && 
                (targetGrabberPower == defaultGrabberPowerIn||targetGrabberPower == defaultGrabberPowerIn/2))) {
                counter++;
            } else {
                counter = 0;
            }
            double current = useNeo?grabberMotorNeo.getOutputCurrent():grabberMotor.getStatorCurrent();
            if (current > currentLimit) {
                iterationsOverCurrentLimit++;
            } else {
                iterationsOverCurrentLimit = 0;
            }
            int numberOfIterationsOverCurrentLimit = useNeo?3:1;
            if ((!ignoreSensorToStop && counter > 1) || (numberOfIterationsOverCurrentLimit <= iterationsOverCurrentLimit)) {
                float f = (float) counter;
                logf("Grabber stopped with current = %.2f and counter %.2f\n", current, f);
                if (useNeo) {
                    //logf("Executing stop by current limit with NEO\n");
                    // grabberMotorNeo.set(0);                    
                } else {
                    grabberMotor.set(0);
                }
                lastGrabberPower = 0;
                targetGrabberPower = 0;
            }
        }

    }

    public double getCurrent() {
        if (useNeo) {
            return grabberMotorNeo.getOutputCurrent();
        } else {
            return grabberMotor.getStatorCurrent();
        }
    }
}
