package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.utilities.Util.logf;

public class PID_SRX extends SubsystemBase {

  // Initial (default) PID coefficients
  public double kP = 0.5;
  public double kI = 0;
  public double kD = 0;
  public double kIz = 0;
  public double kFF = 0;
  public double kMaxOutput = 1;
  public double kMinOutput = -1;
  private boolean smart = false;

  public double maxIntegralAccumulation;
  public double resetIntegralAccumulationThreshold;
  public int allowableCloseLoopError;
  public boolean PIDChanged = false;
  public String name;

  // Set PID using the default PID
  public PID_SRX(String name, boolean smart) {
    this.name = name;
    this.smart = smart;
  }

  // Set PID using the unique values for P, I, D the remaining values are default
  public PID_SRX(String name, double kP, double kI, double kD, boolean smart) {
    this.name = name;
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.name = name;
    this.smart = smart;
  }

  // Set PID using the unique values for all parameters
  public PID_SRX(String name, double kP, double kI, double kD, double kIz, double kFF, double min, double max,
      boolean smart) {
    this.name = name;
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kIz = kIz;
    this.kFF = kFF;
    this.kMaxOutput = max;
    this.kMinOutput = min;
    this.name = name;
    this.smart = smart;
  }

  public PID_SRX(String name, double kP, double kI, double kD, double kIz, double kFF, double maxIntegralAccumulation,
      double resetIntegralAccumulationThreshold, int allowableCloseLoopError, boolean smart) {
    this.name = name;
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kIz = kIz;
    this.kFF = kFF;
    this.smart = smart;
    this.name = name;
    logf("Create %s pid %s \n", name, getPidData());
  }

  @Override
  public void periodic() {
    smart = false;  // todo fix at some point
    if (smart)
      PIDChanged = updatePID();
  }

  public String getPidData() {
    return String.format("P:%f I:%f D:%f IZ:%f FF:%f Min:%f Max:%f", kP, kI, kD, kIz, kFF, kMinOutput, kMaxOutput);
  }

  public void setMinMax(double min, double max) {
    kMaxOutput = max;
    kMinOutput = min;
    if (smart) {
      SmartDashboard.putNumber("Max Output", kMaxOutput);
      SmartDashboard.putNumber("Min Output", kMinOutput);
    }
  }

  void showPID() {
    // display PID coefficients on SmartDashboard
    if (smart) {
      SmartDashboard.putString("PID Name", name);
      SmartDashboard.putNumber("P Gain", kP);
      SmartDashboard.putNumber("I Gain", kI);
      SmartDashboard.putNumber("D Gain", kD);
      SmartDashboard.putNumber("I Zone", kIz);
      SmartDashboard.putNumber("Feed Forward", kFF);
      SmartDashboard.putNumber("Max Output", kMaxOutput);
      SmartDashboard.putNumber("Min Output", kMinOutput);
      SmartDashboard.putNumber("Set Rotations", 0);
    }
  }

  private boolean updatePID() {
    boolean change = false;
    if (smart) {
      // read PID coefficients from SmartDashboard
      double p = SmartDashboard.getNumber("P Gain", 0);
      double i = SmartDashboard.getNumber("I Gain", 0);
      double d = SmartDashboard.getNumber("D Gain", 0);
      double iz = SmartDashboard.getNumber("I Zone", 0);
      double ff = SmartDashboard.getNumber("Feed Forward", 0);
      double max = SmartDashboard.getNumber("Max Output", 0);
      double min = SmartDashboard.getNumber("Min Output", 0);
      if ((p != kP)) {
        logf("PID old p:%.2f new p:%.2f\n", kP, p);
        kP = p;
        change = true;
      }
      
      if ((i != kI)) {
        kI = i;
        change = true;
      }
      if ((d != kD)) {
        kD = d;
        change = true;
      }
      if ((iz != kIz)) {
        kIz = iz;
        change = true;
      }
      if ((ff != kFF)) {
        kFF = ff;
        change = true;
      }
      if ((max != kMaxOutput) || (min != kMinOutput)) {
        kMinOutput = min;
        kMaxOutput = max;
        change = true;
      }
      if (change) {
        logf("PID %s Changed %s\n", name, getPidData());
      }
    }
    return change;
  }
}
