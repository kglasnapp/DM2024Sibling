
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Robot;

import static frc.robot.utilities.Util.logf;

public class CoralSubsystem extends SubsystemBase {
  public boolean found = false;
  public double x;
  public double y;
  public double percent;
  public double area;
  public String type;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry xMidE = table.getEntry("tx");
  NetworkTableEntry yMidE = table.getEntry("ty");
  NetworkTableEntry areaE = table.getEntry("ta");
  NetworkTableEntry typeE = table.getEntry("tclass");
  NetworkTableEntry percentE = table.getEntry("tv");
  
  // Creates a new Subsystem. 
  public CoralSubsystem() {
    logf("Start of Coral Subsystem\n");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Make sure that you declare this subsystem in RobotContainer.java
    percent = percentE.getDouble(0.0);
    type = typeE.getString("None");
    x = xMidE.getDouble(0.0);
    y = yMidE.getDouble(0.0);
    percent = percentE.getDouble(0.0);
    area = areaE.getDouble(0.0);
  }
}