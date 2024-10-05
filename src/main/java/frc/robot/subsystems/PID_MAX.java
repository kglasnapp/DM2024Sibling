package frc.robot.subsystems;

import static frc.robot.Util.logf;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PID_MAX {

    private SparkPIDController pidController;
    private String pidName;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, minVel, maxAcc, allowedErr;

    public void PIDCoefficientsShoot(SparkPIDController pidController) {
        kP = 7e-5;
        // kP = 5e-5;
        kI = 1e-6;
        // kI = 0.0;
        kD = 0;
        kIz = 0;
        kFF = 0.000156;
        kMaxOutput = 1;
        kMinOutput = -1;
        // Smart Motion Coefficients
        // maxRPM = 5700; // for velocity mode
        maxVel = 7000; // for velocity mode
        maxAcc = 15000;
        this.pidController =  pidController;
        pidName = "Shoot";

    }


    public void PIDCoefficientsGrabber(SparkPIDController pidController) {
        kP = 7e-5;
        kI = 1e-7;
        kD = 0;
        kIz = 0;
        kFF = 0;
        kMaxOutput = 1;
        kMinOutput = -1;
        // Smart Motion Coefficients
        // maxRPM = 5700; // for velocity mode

        maxVel = 11000; // for velocity mode
        maxAcc = 44000;
        this.pidController =  pidController;
        pidName = "Grabber";

        // kP = 7e-6;
        
        // kI = 1e-7;
        // kD = 0;
        // kIz = 0;
        // kFF = 0.000156;
        // kMaxOutput = 1;
        // kMinOutput = -1;
        // // Smart Motion Coefficients
        // // maxRPM = 5700; // for velocity mode
        // maxVel = 7000; // for velocity mode
        // maxAcc = 15000;
        // this.pidController =  pidController;
        // pidName = "Grabber";

    }



    public void PIDCoefficientsShooterTilt(SparkPIDController pidController) {
        kP = .00005; 
        kI = .2e-6 ;
        kD = .00005;
        kIz = 0;
        kFF = 0;


        kMaxOutput = 1;
        kMinOutput = -1;
        // Smart Motion Coefficients
        // maxRPM = 5700; // for velocity mode

        maxVel = 15500; // for velocity mode
        maxAcc = 15000;
        this.pidController =  pidController;
        pidName = "Tilt";

    }

    public void PIDCoefficientsTilt(SparkPIDController pidController) {
        kP = 3e-2;
        kI = 1e-6;
        kD = 0;
        kIz = 0;
        kFF = 0.000156;
        kMaxOutput = 1;
        kMinOutput = -1;
        // Smart Motion Coefficients
        // maxRPM = 5700; // for velocity mode
        maxVel = 2000; // for velocity mode
        maxAcc = 1500;
        this.pidController =  pidController;
        pidName = "Tilt";

    }

    public void PIDCoefficientsTiltSmart(SparkPIDController pidController) {
        kP = 3e-6;
        kI = 1e-6;
        kD = 0;
        kIz = 0;
        kFF = 0.000156;
        kMaxOutput = 1;
        kMinOutput = -1;
        // Smart Motion Coefficients
        // maxRPM = 5700; // for velocity mode
        maxVel = 8000; // for velocity mode
        maxAcc = 5500;
        this.pidController =  pidController;
        pidName = "Tilt";

    }


    public void PIDCoefficientsElevator(SparkPIDController pidController) {
        kP = 45e-6;
        kI = 1e-6;
        kD = 0;
        kIz = 0;
        kFF = 0.000156;
        kMaxOutput = 1;
        kMinOutput = -1;
        // Smart Motion Coefficients
        // maxRPM = 5700; // for velocity mode
        maxVel = 5700; // for velocity mode
        maxAcc = 5700 * 2;
        this.pidController =  pidController;
        pidName = "Elevator";
    }
    
    public void PIDCoefficientsIntake(SparkPIDController pidController) {
        kP = .03;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0;
        kMaxOutput = 1;
        kMinOutput = -1;
        // Smart Motion Coefficients
        // maxRPM = 5700; // for velocity mode
        maxVel = 2000; // for velocity mode
        maxAcc = 1500;
        this.pidController =  pidController;
        pidName = "Intake";

    }

    public void PIDToMax() {
        // set PID coefficients
        logf("Set PID data to the controller %s kP:%.5f\n", pidName, kP);
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);

        int smartMotionSlot = 0;
        pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
    }

    public void putPidCoefficientToDashBoard() {
        // display PID coefficients on SmartDashboard
        SmartDashboard.putString("PID", pidName);
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);

        // display Smart Motion coefficients
        SmartDashboard.putNumber("Max Velocity", maxVel);
        SmartDashboard.putNumber("Min Velocity", minVel);
        SmartDashboard.putNumber("Max Acceleration", maxAcc);
        SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
        SmartDashboard.putNumber("Set Position", 0);
        SmartDashboard.putNumber("Set Velocity", 0);
    }

    public void getPidCoefficientsFromDashBoard() {
        // read PID coefficients from SmartDashboard
        SmartDashboard.putString("PID", pidName);
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        double maxV = SmartDashboard.getNumber("Max Velocity", 0);
        double minV = SmartDashboard.getNumber("Min Velocity", 0);
        double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
        double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if ((p != kP)) {
            pidController.setP(p);
            kP = p;
        }
        if ((i != kI)) {
            pidController.setI(i);
            kI = i;
        }
        if ((d != kD)) {
            pidController.setD(d);
            kD = d;
        }
        if ((iz != kIz)) {
            pidController.setIZone(iz);
            kIz = iz;
        }
        if ((ff != kFF)) {
            pidController.setFF(ff);
            kFF = ff;
        }
        if ((max != kMaxOutput) || (min != kMinOutput)) {
            pidController.setOutputRange(min, max);
            kMinOutput = min;
            kMaxOutput = max;
        }
        if ((maxV != maxVel)) {
            pidController.setSmartMotionMaxVelocity(maxV, 0);
            maxVel = maxV;
        }
        if ((minV != minVel)) {
            pidController.setSmartMotionMinOutputVelocity(minV, 0);
            minVel = minV;
        }
        if ((maxA != maxAcc)) {
            pidController.setSmartMotionMaxAccel(maxA, 0);
            maxAcc = maxA;
        }
        if ((allE != allowedErr)) {
            pidController.setSmartMotionAllowedClosedLoopError(allE, 0);
            allowedErr = allE;
        }
    }

}
