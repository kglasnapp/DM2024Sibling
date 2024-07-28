package frc.robot.subsystems;

import static frc.robot.Util.logf;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;


public class IntakeSubsystem extends SubsystemBase {
    
    public final static int OUT_POSITION = 0;
    private IntakeMotor motor;
  
    public State state;

    public static enum State {
        GO_HOME
    };

    public int getPosition() {
        return OUT_POSITION;
    }

    public void intakeIn() {

    }

    public SparkLimitSwitch reverseLimit;


    public void intakeOut() {

    }

    private PID_MAX pid = new PID_MAX();
    private final static int OVER_CURRENT = 30;
    private int INTAKE_MOTOR_ID = 21;


    public IntakeSubsystem() {
        this.motor = new IntakeMotor(INTAKE_MOTOR_ID, false);
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

    int lastPOV = -1;

    @Override
    public void periodic() {
        //if (robot.count % 5 == 0) {
                   SmartDashboard.putNumber("Intake Speed", motor.getVelocity());
                   
                   SmartDashboard.putNumber("Intake Pos", motor.getPosition());
        //}
       
        int pov = RobotContainer.driveController.getHID().getPOV();
    
        double value = 0;
        if ((lastPOV != pov) && (pov >= 0)) {
            if (pov == 0) {
                value = 0.0;
            }
            if (pov == 90) {

                value = 0.25;
            }
            if (pov == 180) {
                value = 0.65;
            }
            if (pov == 270) {
                value = 0.9;
            }
           motor.setSpeed(value);
          

            lastPOV = pov;
        }else if(lastPOV != pov){
            lastPOV = pov;
        }

        
    }

}
