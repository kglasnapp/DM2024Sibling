// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.utilities.Util.logf;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import frc.robot.subsystems.LedSubsystem;
// import frc.robot.subsystems.PDHData;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  boolean hasBeenHomed = false;

  public static int count = 0;
  // private final PDHData pdhData = new PDHData();
  public static Optional<Alliance> alliance;
  // public static Leds led = new Leds();

  Command cmd;
  RobotContainer robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    alliance = DriverStation.getAlliance();
    
    Util.logf("Start Sibling %s\n", alliance.toString());
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    count++;
    if (count % 1000 == 67) {
      // pdhData.logPDHData();
    }
    // led.periodic();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    if (cmd != null) {
      logf("Executing disabled init %s\n", cmd.getName());
      cmd.cancel();
    }
    hasBeenHomed = false;
    robotContainer.climberSubsystem.disableRobot();

  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // cmd = RobotContainer.autonomousChooser.getSelected();
    Integer firstStepWait = RobotContainer.autonomousChooserFirtWait.getSelected();
    boolean autoAim = RobotContainer.autonomousAim.getSelected();
    Integer firstStep = RobotContainer.autonomousChooserFirstStep.getSelected();
    Integer lastStep = RobotContainer.autonomousChooserLastStep.getSelected();
    if (lastStep >= firstStep) {
      lastStep++;
    } else {
      lastStep--;
    }
    Command cmd = Autonomous.getAutonomousCommand(robotContainer, firstStep, lastStep, autoAim, firstStepWait);
    if (cmd != null) {
      logf("Executing autonomous %s\n", cmd.getName());
      cmd.schedule();
    }
    homeAllSubsystems();
  }

  void homeAllSubsystems() {
    if (!hasBeenHomed) {
      robotContainer.intakeSubsystem.state = IntakeSubsystem.State.GO_HOME;
      robotContainer.shooterSubsystem.state = ShooterSubsystem.State.GO_HOME;
      robotContainer.climberSubsystem.homeClimber();
      hasBeenHomed = true;
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    Util.logf("TELEOP INIT %s\n", alliance.toString());
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    alliance = DriverStation.getAlliance();
    
    Util.logf("Enable Robot Alliance: %s\n", alliance.toString());
    
    homeAllSubsystems();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
