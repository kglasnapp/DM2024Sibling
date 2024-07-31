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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.GrabberInCommand;
import frc.robot.commands.GrabberOutCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LoadNoteCommand;
import frc.robot.commands.SetShooterAngleCommand;
import frc.robot.commands.ShootAmpCommand;
import frc.robot.commands.ShootToSpeakerCommand;
import frc.robot.commands.SpeakerAlligningCommand;
import frc.robot.commands.StraightToAmpCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LimeLightPoseSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystemOld;

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
  public final ShooterSubsystemOld shooterSubsystem = (testMode) ? null : new ShooterSubsystemOld(this);
  public final ShooterSubsystem shooterSubsystem2 = new ShooterSubsystem();
  public final IndexerSubsystem shooterSubsystemEly= new IndexerSubsystem();
  public final GrabberSubsystem grabberSubsystem =  (testMode) ? null :new GrabberSubsystem(this);
  public final ClimberSubsystem climberSubsystem =  (testMode) ? null :new ClimberSubsystem();
  public final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  public final LedSubsystem leds = (testMode)?null: new LedSubsystem(grabberSubsystem, shooterSubsystem);
  public final static CommandXboxController driveController = new CommandXboxController(2);
  public final static CommandXboxController operatorController = new CommandXboxController(3);
  public final static XboxController operatorHID = operatorController.getHID();
  public final static XboxController driveHID = driveController.getHID();


  private SlewRateLimiter sLX = new SlewRateLimiter(15);
  private SlewRateLimiter sLY = new SlewRateLimiter(15);
  private SlewRateLimiter sRX = new SlewRateLimiter(15);
  
  public static SendableChooser<Boolean> autonomousAim = new SendableChooser<>();
  public static SendableChooser<Integer> autonomousChooserFirtWait = new SendableChooser<>();
  public static SendableChooser<Integer> autonomousChooserFirstStep = new SendableChooser<>();
  public static SendableChooser<Integer> autonomousChooserLastStep = new SendableChooser<>();
  public LimeLightPoseSubsystem limeLightPoseSubsystem;
  public static CoralSubsystem coralSubsystem = new CoralSubsystem();


  public final static Pose2d BLUE_SPEAKER = new Pose2d(-0.0381, 5.54, new Rotation2d());
  public final static Pose2d RED_SPEAKER = new Pose2d(16.57, 5.54, new Rotation2d(Math.toRadians(180)));

  public final static Pose2d BLUE_AMP = new Pose2d(72.5, 323, new Rotation2d(Math.toRadians(270)));
  public final static Pose2d RED_AMP = new Pose2d(578.77, 323, new Rotation2d(Math.toRadians(270)));

  public static RobotContainer instance;
  public Autonomous autotonomous;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    instance = this;
    // Set the default Robot Mode to Cube
    logf("creating RobotContainer\n");
   
    drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        drivetrainSubsystem,
        () -> (-modifyAxis((sLY.calculate(driveController.getLeftY())))          
                * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
        () -> -modifyAxis((sLX.calculate(driveController.getLeftX())))
            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> (-modifyAxis((sRX.calculate(driveController.getRightX())))
                * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
        driveController.y()));// Set precision based upon left bumper    

    limeLightPoseSubsystem = new LimeLightPoseSubsystem(drivetrainSubsystem, "limelight-front");
    configureButtonBindings();
    configureDashboard();
    autotonomous = new Autonomous(this, drivetrainSubsystem, intakeSubsystem);
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
      // configureOperatorController(operatorController);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  public void testAmpShoot(CommandXboxController controller) {
    controller.pov(0).onTrue(new Command() {
      @Override
      public void initialize() {
        // double currentAngle = shooterSubsystem.servo.getAngle();
        shooterSubsystem.servo.setAngle(ShooterSubsystemOld.FLAPPER_EXPANDED_ANGLE);
        shooterSubsystem.setTiltAngle(-4.5);
        shooterSubsystem.setShooterPower(0.155);
      }
      @Override
      public boolean isFinished() {
        return true;
      }
    });

    controller.pov(180).onTrue(new Command() {
      @Override
      public void initialize() {
        grabberSubsystem.grabberOut();
      }
      @Override
      public boolean isFinished() {
        return true;
      }
    });
  }

  public void testServo(CommandXboxController controller) {
    controller.pov(0).onTrue(new Command() {
      @Override
      public void initialize() {
        // double currentAngle = shooterSubsystem.servo.getAngle();
        shooterSubsystem.servo.setAngle(ShooterSubsystemOld.FLAPPER_EXPANDED_ANGLE);
        logf("servo setting angle to = keith 118\n");
      }
      @Override
      public boolean isFinished() {
        return true;
      }
    });

    controller.pov(180).onTrue(new Command() {
      @Override
      public void initialize() {
        shooterSubsystem.servo.setAngle(ShooterSubsystemOld.FLAPPER_RETRACTED_ANGLE); 
        logf("Servo setting angle to = 100\n");
      }
      @Override
      public boolean isFinished() {
        return true;
      }
    });

  }
  
  public void calibrateShooter(CommandXboxController controller) {
    
    controller.pov(0).onTrue(new Command() {

      @Override
      public void initialize() {
        double currentAngle = shooterSubsystem.getTiltAngle();
        shooterSubsystem.setTiltAngle(currentAngle - 0.5);
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
        shooterSubsystem.setShooterPower(1);
      }
      @Override
      public boolean isFinished() {
        return true;
      }
    });

    controller.pov(180).onTrue(new Command() {
      @Override
      public void initialize() {
        double currentAngle = shooterSubsystem.getTiltAngle();
        shooterSubsystem.setTiltAngle(currentAngle + 0.5);
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
        grabberSubsystem.grabberOut();
        logf("Shooting at %.2f distance with %.2f angle\n",ShootToSpeakerCommand.distance(BLUE_SPEAKER, limeLightPoseSubsystem.get()), shooterSubsystem.getTiltAngle());
      }
      @Override
      public boolean isFinished() {
        return true;
      }
    });
  }

  public void testShootToAmpWithTrajectory(CommandXboxController controller) {

    // id #6 x = 1.8415 y = 8.2042   1.78  6.21
    // id #5 x = 14.70  y = 8.2042 

    controller.x().whileTrue(
      new ShootAmpCommand(shooterSubsystem, intakeSubsystem, grabberSubsystem, limeLightPoseSubsystem)
      .andThen(new StraightToAmpCommand(drivetrainSubsystem, limeLightPoseSubsystem))
      .andThen(new DriveCommand(drivetrainSubsystem, 0.01,0, 0))
      .andThen(new WaitCommand(0.5))
      .andThen(new DriveCommand(drivetrainSubsystem, 0,0, 0))
      .andThen(new GrabberOutCommand(grabberSubsystem)));
  }



  public void configureOperatorController(CommandXboxController controller) {
    //pov(0) set shooter angle to 0
    // controller.pov(0).onTrue(new SetShooterAngleCommand(shooterSubsystem, false, 0)); // Retrack
    //pov(90) set shooter to minimum angle
    // controller.pov(180).onTrue(new SetShooterAngleCommand(shooterSubsystem, false, -30));
    //pov(270) set shooter to amp shot
    // controller.pov(270).onTrue(new SetShooterAngleCommand(shooterSubsystem, true, -4)); // Up extended
    // controller.pov(90).onTrue(Autonomous.grabNoteCommand(this));
    // controller.pov(90).onTrue(new StraightPathCommand(drivetrainSubsystem, limeLightPoseSubsystem,
    //                                 new Pose2d(2.37,5.58, new Rotation2d())));
    controller.y().onTrue(new ShootToSpeakerCommand(shooterSubsystem,
        intakeSubsystem,
        grabberSubsystem,
        limeLightPoseSubsystem,
        drivetrainSubsystem).alongWith(new SpeakerAlligningCommand(limeLightPoseSubsystem, drivetrainSubsystem)));
    controller.b().onTrue(
        new IntakeCommand(intakeSubsystem, IntakeCommand.State.OUT, -1)
            .alongWith(new GrabberInCommand(grabberSubsystem)));
    controller.pov(90).onTrue(
      new GrabberInCommand(grabberSubsystem));
    controller.a().onTrue(
      new LoadNoteCommand(intakeSubsystem, grabberSubsystem, shooterSubsystem));    
    controller.x().whileTrue(new Command() {
      @Override
      public void initialize() {
        //intakeSubsystem.state = IntakeSubsystem.State.GO_HOME;
      }
      public boolean isFinished() {
        return true;
      }
    });    
    //only one of the following three v
    //calibrateShooter(controller);
  //  testShootToAmpWithTrajectory(controller);
    //testAmpShoot(controller);
    //testAutonomous(); //case 1
  }

  public void configureDriverController(CommandXboxController controller) {
    driveController.back().whileTrue(new RunCommand(new Runnable() {
      public void run() {
        drivetrainSubsystem.zeroGyroscope();
      }
    }));
   
    // controller.pov(180).whileTrue(new Command() {
    //   @Override
    //   public void initialize() {
    //     double currentAngle = shooterSubsystem.getTiltAngle();
    //     shooterSubsystem.setTiltAngle(currentAngle - 0.5);
    //    logf("setting tilt angle 1 to:%.2f \n", currentAngle - 0.5);
    //   }
    //   @Override
    //   public boolean isFinished() {
    //     return true;
    //   }
    // });

    // controller.x().whileTrue(new Command() {
    //   @Override
    //   public void initialize() {
    //     shooterSubsystem.setTiltAngle(-25);       
    //   }
    //   @Override
    //   public boolean isFinished() {
    //     return true;
    //   }
    // });

    // controller.pov(90).onTrue(new StraightPathCommand(drivetrainSubsystem, limeLightPoseSubsystem,
    //                                 new Pose2d(2.37,5.58, new Rotation2d())));
    // controller.b().onTrue(new Command() {
    //   boolean finished = false;

    //   enum State {
    //     START, READY, DONE
    //   }

    //   State state;

    //   @Override
    //   public void initialize() {
    //     finished = false;
    //     state = State.START;
    //   }

    //   double timer;

    //   @Override
    //   public void execute() {
    //     switch (state) {
    //       case START:
    //         shooterSubsystem.setShooterPower(1);
    //         state = State.READY;
    //         break;
    //       case READY:
    //         if (shooterSubsystem.getShootVelocity() > 6000 &&
    //             shooterSubsystem.getShootVelocity2() > 6000) {
    //           grabberSubsystem.grabberOut();
    //           timer = RobotController.getFPGATime() / 1000;
    //           state = State.DONE;
    //         }
    //         break;
    //       case DONE:
    //         if (timer + 500 < RobotController.getFPGATime() / 1000) {
    //           shooterSubsystem.setShooterPower(0.3);
    //           finished = true;
    //         }
    //         break;
    //     }
    //   }

    //   @Override
    //   public boolean isFinished() {
    //     return finished;
    //   }

    // });                    
    
    
  }

  public void testAutonomous() {
    driveController.pov(0).whileTrue(Autonomous.getAutonomousCommand(this, 9, 11, false,0 ));
  } 

  public final void testPickNote(RobotContainer robotContainer) {
    driveController.leftBumper().onTrue(new Command() {
      @Override
      public void initialize() {
        logf("Sending the shooter to %.2f angle\n", -driveController.getLeftTriggerAxis() * 30);
        robotContainer.shooterSubsystem.setTiltAngle(-driveController.getLeftTriggerAxis() * 30);
      }

      @Override
      public boolean isFinished() {
        return true;
      }

    });
    driveController.b().onTrue(Autonomous.grabNoteCommand(this));
    driveController.a().onTrue(new ShootToSpeakerCommand(shooterSubsystem,
        intakeSubsystem,
        grabberSubsystem,
        limeLightPoseSubsystem,
        drivetrainSubsystem).alongWith(new SpeakerAlligningCommand(limeLightPoseSubsystem, drivetrainSubsystem)));
  }

  public static final void testGrabber(RobotContainer robotContainer) {
    GrabberSubsystem grabberSubsystem = new GrabberSubsystem(robotContainer);

    driveController.b().onTrue(
        // new ResetOdometryWithCameraCommand(limeLightPoseSubsystem));
        new Command() {
          @Override
          public void initialize() {
            logf("grabber in\n");
            grabberSubsystem.grabberIn();
          }

          @Override
          public boolean isFinished() {
            return true;
          }
        });

    driveController.a().onTrue(
        // new ResetOdometryWithCameraCommand(limeLightPoseSubsystem));
        new Command() {
          @Override
          public void initialize() {
            grabberSubsystem.grabberOut();
            logf("grabber out\n");
          }

          @Override
          public boolean isFinished() {
            return true;
          }
        });

    driveController.x().onTrue(
        // new ResetOdometryWithCameraCommand(limeLightPoseSubsystem));
        new Command() {
          @Override
          public void initialize() {
            grabberSubsystem.grabberOff();
          }

          @Override
          public boolean isFinished() {
            return true;
          }
        });
  }

  public final void testShooterTilt() {
    // IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    int povs[] = { 0, 90, 180, 270 };
    for (int pov : povs) {
      driveController.pov(pov).onTrue(
          // new ResetOdometryWithCameraCommand(limeLightPoseSubsystem));
          new Command() {
            @Override
            public void initialize() {
              logf("intake in\n");
              shooterSubsystem.setTiltAngle(-5 * pov / 90);
            }

            @Override
            public boolean isFinished() {
              return true;
            }
          });
    }

  }

  public final void testIntake() {
    // IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    driveController.b().onTrue(
        // new ResetOdometryWithCameraCommand(limeLightPoseSubsystem));
        new Command() {
          @Override
          public void initialize() {
            logf("intake in\n");
            intakeSubsystem.intakeIn();
          }

          @Override
          public boolean isFinished() {
            return true;
          }
        });

    driveController.a().onTrue(
        // new ResetOdometryWithCameraCommand(limeLightPoseSubsystem));
        new Command() {
          @Override
          public void initialize() {
            logf("intake out\n");
            intakeSubsystem.intakeOut();
          }

          @Override
          public boolean isFinished() {
            return true;
          }
        });

  }

  public final void testShooterFlap() {

    driveController.b().onTrue(
        // new ResetOdometryWithCameraCommand(limeLightPoseSubsystem));
        new Command() {
          @Override
          public void initialize() {
            shooterSubsystem.setFlapAngle(75);
          }

          @Override
          public boolean isFinished() {
            return true;
          }
        });

    driveController.a().onTrue(
        // new ResetOdometryWithCameraCommand(limeLightPoseSubsystem));
        new Command() {
          @Override
          public void initialize() {
            shooterSubsystem.setFlapAngle(25);  // TODO KAG was 10
          }

          @Override
          public boolean isFinished() {
            return true;
          }
        });
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
