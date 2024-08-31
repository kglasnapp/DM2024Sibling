// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * OPERATOR:
 * left trigger: left arm down (engage left ratchet)
 * left bumper: left arm up (disengage ratchet)
 * right trigger: right arm down (engage ratchet)
 * right bumper: right arm down (disengage right ratchet)
 * 
 * a bttn: (full auto intake) press and hold: intake out, wheels in
 * a bttn: (full auto intake) release: intake in, move shooter to loading angle, wheels out
 * b bttn: speaker shot
 * x bttn: amp shot
 * y bttn: preload note (intake in, wheels out, shooter at loading angle)
 * d-pad up: speaker shot (close up, aka max tilt)
 * d-pad left: load note angle
 * d-pad down: move shooter to zero degrees
 * 
 * DRIVER:
 * back button: reset robot orientation
 * left bumper: precision drive mode (slow everything down)
 * right bumper: precision drive mode (slow everything down)
 */

package frc.robot;

import static frc.robot.Util.logf;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ChangeNormalModeCommand;
import frc.robot.commands.ChangeTurboModeCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.IntakeNoteCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TiltHomeCommand;
import frc.robot.commands.TiltManualCommand;
import frc.robot.commands.TiltSetAngleCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PoseSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TiltSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static boolean testMode = true;
  // robotIDCheck.get returns true for the sibling, false for
  final DigitalInput robotIDCheck = new DigitalInput(0);

  // private final LimeLightPose limeLightPose = new LimeLightPose();
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public final static IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  public final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  public final static CommandXboxController driveController = new CommandXboxController(2);
  public final static CommandXboxController operatorController = new CommandXboxController(3);
  public final static XboxController operatorHID = operatorController.getHID();
  public final static XboxController driveHID = driveController.getHID();

  // Second number is seconds to reach max speed
  private SlewRateLimiter sLX = new SlewRateLimiter(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / 1.0);
  private SlewRateLimiter sLY = new SlewRateLimiter(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / 1.0);
  private SlewRateLimiter sRX = new SlewRateLimiter(DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 1.0);

  public static SendableChooser<Boolean> autonomousAim = new SendableChooser<>();
  public static SendableChooser<Integer> autonomousChooserFirstWait = new SendableChooser<>();
  public static SendableChooser<Integer> autonomousChooserFirstStep = new SendableChooser<>();
  public static SendableChooser<Integer> autonomousChooserLastStep = new SendableChooser<>();
  public PoseSubsystem poseSubsystem;
  public static CoralSubsystem coralSubsystem = new CoralSubsystem();
  public TiltSubsystem tilt = new TiltSubsystem();

  // TODO: Move these to a separate file
  public final static Pose2d BLUE_SPEAKER = new Pose2d(-0.0381, 5.54, new Rotation2d());
  public final static Pose2d RED_SPEAKER = new Pose2d(16.57, 5.54, new Rotation2d(Math.toRadians(180)));
  public final static Pose2d BLUE_AMP = new Pose2d(72.5, 323, new Rotation2d(Math.toRadians(270)));
  public final static Pose2d RED_AMP = new Pose2d(578.77, 323, new Rotation2d(Math.toRadians(270)));

  public static RobotContainer instance;
  public Autonomous autonomous;
  public boolean hasBeenHomed = false;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    instance = this;
    // Set the default Robot Mode to Cube
    logf("creating RobotContainer\n");

    drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        drivetrainSubsystem,
        () -> sLY.calculate(modifyAxis(driveController.getLeftY())
            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
        () -> sLX.calculate(modifyAxis(driveController.getLeftX())
            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
        () -> sRX.calculate(-modifyAxis(driveController.getRightX())
            * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND),
        // Set precision based upon left bumper
        // Not used, currently implemented with separate commands
        driveController.y(),
        // Set robot oriented control based upon left bumper
        driveController.leftBumper()));

    poseSubsystem = new PoseSubsystem(drivetrainSubsystem, "limelight");
    configureButtonBindings();
    configureDashboard();
    autonomous = new Autonomous(this, drivetrainSubsystem, intakeSubsystem);
  }

  void homeAllSubsystems() {
    if (!hasBeenHomed) {
      tilt.homeTilt();
      if (climberSubsystem != null) {
        // TODO once climber built enable homing climberSubsystem.homeClimber();
      }
      hasBeenHomed = true;
    }
  }

  public double squareWithSign(double v) {
    return (v < 0) ? -(v * v) : (v * v);
  }

  private void configureDashboard() {
  }

  public static double getRightTrigger() {
    return driveController.getRightTriggerAxis();
  }

  public static double getLeftTrigger() {
    return driveController.getLeftTriggerAxis();
  }

  public static boolean getRightBumper() {
    return driveHID.getRawButton(6);
  }

  public static boolean getLeftBumper() {
    return driveHID.getRawButton(5);
  }

  public static boolean getOperatorRightBumper() {
    return operatorHID.getRawButton(6);
  }

  public static boolean getOperatorLeftBumper() {
    return operatorHID.getRawButton(5);
  }

  public static int getDriverPov() {
    return driveHID.getPOV();
  }

  public static boolean getBack() {
    return driveHID.getBackButton();
  }

  private void configureButtonBindings() {

    configureDriverController(driveController);
    // calibrateShooter(driveController);
    try {
      configureOperatorController(operatorController);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  // TODO this method need a lot of work
  public void calibrateShooter(CommandXboxController controller) {
    controller.pov(0).onTrue(new Command() {
      @Override
      public void initialize() {
        double currentAngle = tilt.getTiltAngle();
        tilt.setTiltAngle(currentAngle - 0.5);
        logf("setting tilt angle 1 to:%.2f \n", currentAngle - 0.5);
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    });

    controller.pov(90).onTrue(new Command() {
      @Override
      public void initialize() {
        shooterSubsystem.setAllShooterPower(1.0);
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    });

    controller.pov(180).onTrue(new Command() {
      @Override
      public void initialize() {
        double currentAngle = tilt.getTiltAngle();
        tilt.setTiltAngle(currentAngle + 0.5);
        logf("setting tilt angle 2 to:%.2f\n", (currentAngle + 0.5));
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    });

    controller.pov(270).onTrue(new Command() {
      @Override
      public void initialize() {
        // grabberSubsystem.grabberOut();
        logf("Shooting at %.2f distance with %.2f angle\n",
            // ShootToSpeakerCommand.distance(BLUE_SPEAKER, limeLightPoseSubsystem.get()),
            tilt.getTiltAngle());
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    });
  }

  public void configureDriverController(CommandXboxController controller) {
    driveController.back().whileTrue(new RunCommand(new Runnable() {
      public void run() {
        drivetrainSubsystem.zeroGyroscope();
      }
    }));

    driveController.x().onTrue(
        new IntakeNoteCommand(intakeSubsystem, indexerSubsystem));

    driveController.y().onTrue(
        new ShootCommand(shooterSubsystem, indexerSubsystem, poseSubsystem));

    driveController.rightTrigger().onTrue(
        new ChangeNormalModeCommand());

    driveController.leftTrigger().onTrue(
        new ChangeTurboModeCommand());

  }

  // --------------------- Buttons for Operator -----------------
  public void configureOperatorController(CommandXboxController opController) {
    opController.back().onTrue(new TiltHomeCommand(tilt));
    opController.x().whileTrue(new TiltSetAngleCommand(tilt, 45.0));
    opController.leftBumper().onTrue(new TiltManualCommand(tilt, false)); // Send shooter down
    opController.rightBumper().onTrue(new TiltManualCommand(tilt, true)); // Send shooter up
  }

  public void testAutonomous() {
    driveController.pov(0).whileTrue(Autonomous.getAutonomousCommand(this, 9, 11, false, 0));
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    value = deadband(value, 0.08);
    value = Math.copySign(value * value, value); // Square the axis
    return value;
  }
}
