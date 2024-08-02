
package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.GrabNoteCommandAutonomous;
import frc.robot.commands.GrabberInCommand;
import frc.robot.commands.GrabberOutCommand;
import frc.robot.commands.LoadNoteCommand;
import frc.robot.commands.ShootToSpeakerCommand;
import frc.robot.commands.SpeakerAlligningCommand;
import frc.robot.commands.StraightPathCommand;
import frc.robot.commands.WaitUntilShooterReadyCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class Autonomous {
        DrivetrainSubsystem drivetrainSubsystem;
        RobotContainer robotContainer;
        // private final BalanceCommand balanceCommand;

        public static Command test;

        public Autonomous(RobotContainer robotContainer, DrivetrainSubsystem drivetrain, IntakeSubsystem intake) {
                this.robotContainer = robotContainer;
                this.drivetrainSubsystem = drivetrain;
                // RobotContainer.autonomousChooserFirstStep.getSelected();
                RobotContainer.autonomousAim.setDefaultOption("False", false);
                RobotContainer.autonomousChooserFirtWait.setDefaultOption("0", Integer.valueOf(0));
                RobotContainer.autonomousChooserFirstStep.setDefaultOption("0", Integer.valueOf(-1));
                RobotContainer.autonomousChooserLastStep.setDefaultOption("0", Integer.valueOf(-1));

                RobotContainer.autonomousAim.addOption("True", true);
                for (int i = 0; i < 6; ++i) {
                        RobotContainer.autonomousChooserFirtWait.addOption("" + (i), Integer.valueOf(i));                 
                }

                for (int i = 0; i < 16; ++i) {
                        RobotContainer.autonomousChooserFirstStep.addOption("" + (i + 1), Integer.valueOf(i));
                        RobotContainer.autonomousChooserLastStep.addOption("" + (i + 1), Integer.valueOf(i));
                }
                SmartDashboard.putData("Auto Start Wait", RobotContainer.autonomousChooserFirtWait);
                SmartDashboard.putData("Auto Aim", RobotContainer.autonomousAim);
                SmartDashboard.putData("Autonomous Start", RobotContainer.autonomousChooserFirstStep);
                SmartDashboard.putData("Autonomous End", RobotContainer.autonomousChooserLastStep);

                // balanceCommand = new BalanceCommand(drivetrain);
                // RobotContainer.autonomousChooser.setDefaultOption("Balance ",
                // getOverAndBalanceCommand(drivetrain));

                // RobotContainer.autonomousChooser.addOption("Coral Tag 6",
                // Autonomous.getPieceWithCoral(robotContainer,
                // robotContainer.limeLightPoseSubsystem,
                // "/home/lvuser/deploy/Blue6Short.wpilib.json",
                // "/home/lvuser/deploy/Blue6ReturnShort.wpilib.json",
                // new Pose2d(3.8, 4.8, new Rotation2d(Math.toRadians(180)))));

                // // test = testMovements(drivetrain, intake, robotContainer);
                // // Put the chooser on the dashboard
                // SmartDashboard.putData("Autonomous Mode", RobotContainer.autonomousChooser);
                // SmartDashboard.putData("Autonomous Mode", autonomousChooser);

        }

        // ShooterSubsystem shooterSubsystem;
        // IntakeSubsystem intakeSubsystem;
        // GrabberSubsystem grabberSubsystem;
        // LimeLightPoseSubsystem poseEstimatorSubsystem;
        /**
        ------------------------------------------------------------------
 Blue   |								 |   Red
        |			        4,12		  		 |
        |	3       					11	 |
        |								 |
        |				5,13				 |
        |()	2       					10     ()|
        |								 |
        |		/		6,14	       \		 |
        |	1 ------      				-----	9	 |
        |		\			       /		 |
        |				7,15				 |
        |								 |
        |								 |
        |				8,16				 |
        |								 |
        ------------------------------------------------------------------
         */

        public static final double noteInitialPosition[][] = new double[][] {
                        { 1.30, 4.16 }, // 1
                        { 1.30, 5.54 }, // 2 -- good
                        { 1.30, 7.04 }, // 3
                        { 7.71, 7.49 }, // 4
                        { 7.71, 5.82 }, // 5
                        { 7.71, 4.14 }, // 6
                        { 7.71, 2.46 }, // 7
                        { 7.71, 0.80 }, // 8
                        { 15.02, 4.16 }, // 9 --- good
                        { 15.02, 5.54 }, // 10
                        { 15.02, 7.04 }, // 11
                        { 8.71, 7.49 }, // 12
                        { 8.71, 5.82 }, // 13
                        { 8.71, 4.14 }, // 14
                        { 8.71, 2.46 }, // 15
                        { 8.71, 0.80 } // 16
        };


        public static final double accendingNoteInitialPosition[][] = new double[][] {
                        { 1.30, 4.16, 180 }, // 1
                        { 1.30, 5.54, 180 }, // 2 -- good
                        { 2.15, 6.04, 230 }, // 3
                        { 7.71, 7.49, 180 }, // 4
                        { 7.71, 5.82, 180 }, // 5
                        { 7.71, 4.14, 180 }, // 6
                        { 7.71, 2.46, 180 }, // 7
                        { 7.71, 0.80, 180 }, // 8
                        { 15.02, 4.16, 0 }, // 9 --- good
                        { 14.53, 5.54, 0 }, // 10
                        { 14.27, 6.04, -50 }, // 11
                        { 8.71, 7.49, 0 }, // 12
                        { 8.71, 5.82, 0 }, // 13
                        { 8.71, 4.14, 0 }, // 14
                        { 8.71, 2.46, 0 }, // 15
                        { 8.71, 0.80, 0 } // 16
        };


        public static final double descendingNoteInitialPosition[][] = new double[][] {
                        { 2.15, 3.16, 130 }, // 1
                        { 1.30, 5.54, 180 }, // 2 -- good                        
                        { 1.30, 7.04, 180 }, // 3
                        { 7.71, 7.49, 180 }, // 4
                        { 7.71, 5.82, 180 }, // 5
                        { 7.71, 4.14, 180 }, // 6
                        { 7.71, 2.46, 180 }, // 7
                        { 7.71, 0.80, 180 }, // 8
                        { 14.17, 3.16, 50 }, // 9 --- good
                        { 15.02, 5.54, 0 }, // 10
                        { 15.02, 7.04, 0 }, // 11
                        { 8.71, 7.49, 0 }, // 12
                        { 8.71, 5.82, 0 }, // 13
                        { 8.71, 4.14, 0 }, // 14
                        { 8.71, 2.46, 0 }, // 15
                        { 8.71, 0.80, 0 } // 16
        };


        public static final double shootAccendingPosition[][] = new double[][] {
                        { 1.30, 4.16 }, // 1
                        { 1.30, 5.54 }, // 2 -- good
                        { 1.30, 7.04 }, // 3
                        { 1.30, 7.04 }, // 4
                        { 1.30, 5.82 }, // 5
                        { 7.71, 4.14 }, // 6
                        { 1.30, 2.46 }, // 7
                        { 1.30, 0.80 }, // 8
                        { 15.02, 5.54 }, // 9 --- good
                        { 15.02, 5.54 }, // 10
                        { 15.02, 7.04 }, // 11
                        { 15.02, 7.04 }, // 12
                        { 15.02, 5.82 }, // 13
                        { 8.71, 4.14 }, // 14
                        { 15.02, 2.46 }, // 15
                        { 15.02, 0.80 } // 16
        };

        public static final double shootDescendingPosition[][] = new double[][] {
                       { 1.30, 4.16 }, // 1
                        { 1.30, 5.54 }, // 2 -- good
                        { 1.30, 7.04 }, // 3
                        { 1.30, 7.49 }, // 4
                        { 1.30, 5.82 }, // 5
                        { 7.71, 4.14 }, // 6
                        { 3.30, 2.46 }, // 7
                        { 3.30, 2.46 }, // 8
                        { 15.02, 4.16 }, // 9 --- good
                        { 15.02, 5.54 }, // 10
                        { 15.02, 5.54 }, // 11
                        { 15.02, 5.82 }, // 12
                        { 15.02, 5.82 }, // 13
                        { 8.71, 4.14 }, // 14
                        { 14.02, 2.46 }, // 15
                        { 14.02, 0.80 } // 16
        };

        public static final double chaosPosition[][] = new double[][] {                        
                        { 7.71, 7.49, 45 }, // 4
                        { 7.71, 5.82, 45 }, // 5
                        { 7.71, 4.14, 45 }, // 6
                        { 7.71, 2.46, 45 }, // 7
                        { 7.71, 0.80, 45 }, // 8
                        { 8.71, 7.49, 45 }, // 12
                        { 8.71, 5.82, 45 }, // 13
                        { 8.71, 4.14, 45 }, // 14
                        { 8.71, 2.46, 45 }, // 15
                        { 8.71, 0.80, 45 } // 16
        };

        public static final double allowedShootXBlue = 5.25;
        public static final double allowedShootXRed = 11.25;
        public static final double allowedShootYBlueUp = 7.10;
        public static final double allowedShootYRedUp = 7.10;

        public static final double allowedShootYBlueDown = 1.58;
        public static final double allowedShootYRedDown = 1.58;

        public static Command getAutonomousCommand(RobotContainer robotContainer, int from, int until, 
                                                  boolean aimFirst, int delay) {
                double robotAngle = (from >= 8) ? 0 : 180;
                
                double shootFromPosition[][] = (from <= until) ? shootAccendingPosition : shootDescendingPosition;

                Command command;

                if (aimFirst) {
                        command = new ShootToSpeakerCommand(robotContainer.shooterSubsystem,
                                                robotContainer.intakeSubsystem,
                                                robotContainer.grabberSubsystem,
                                                robotContainer.limeLightPoseSubsystem,
                                                robotContainer.drivetrainSubsystem)
                        .alongWith(new SpeakerAlligningCommand(
                                                        robotContainer.limeLightPoseSubsystem, 
                                                        robotContainer.drivetrainSubsystem));
                } else {
                        command =
                                new Command() {
                                        @Override
                                        public void initialize() {
                                                robotContainer.shooterSubsystem.setTiltAngle(-4);
                                        }
                                        @Override
                                        public boolean isFinished() {
                                                return true;
                                        }
                                };
                        command = command.andThen(
                                        new WaitUntilShooterReadyCommand(robotContainer.shooterSubsystem)                                                
                                                        .andThen(new GrabberOutCommand(robotContainer.grabberSubsystem))
                                                        .andThen(new WaitCommand(0.3)));
                }
                if (delay > 0) {
                        command = new WaitCommand(delay).andThen(command);
                }
                if (from == -1) {
                        return command;
                }
                /**
                 * from can be higher than until.
                 * In that case, we go in reverse order
                 * look at the end of the loop to see how
                 * i gets incremented, or decremented.
                 */
                for (int i = from; i != until;) {
                        //command = command
                                       // .andThen(new IntakeNoteCommand(robotContainer.intakeSubsystem));
                                                      //  IntakeCommand.State.OUT, 4000));
                        if (shootFromPosition[i][0] == noteInitialPosition[i][0] &&
                                        shootFromPosition[i][1] == noteInitialPosition[i][1]) {
                        //command = command
                        //               .andThen(new WaitUntilIntakeOutCommand(robotContainer.intakeSubsystem));
                        }
                        double pickupX = noteInitialPosition[i][0];                                                                                        
                        double pickupY = noteInitialPosition[i][1];
                        double pickupAlpha = robotAngle;

                        if (from < until) {
                                pickupX = accendingNoteInitialPosition[i][0];
                                pickupY = accendingNoteInitialPosition[i][1];
                                pickupAlpha = accendingNoteInitialPosition[i][2];
                        } else if (until < from) {
                                pickupX = descendingNoteInitialPosition[i][0];
                                pickupY = descendingNoteInitialPosition[i][1];
                                pickupAlpha = descendingNoteInitialPosition[i][2];                                
                        }
                        
                        command = command
                                        .andThen(
                                                        new StraightPathCommand(robotContainer.drivetrainSubsystem,
                                                                        robotContainer.limeLightPoseSubsystem,
                                                                        new Pose2d(pickupX,
                                                                                        pickupY,
                                                                                        Rotation2d.fromDegrees(
                                                                                                        pickupAlpha)))
                                                                        .alongWith(new GrabberInCommand(
                                                                                        robotContainer.grabberSubsystem)))
                                        .andThen(grabNoteCommand(robotContainer));
                                        //.andThen(new WaitCommand(0.1));

                        if (shootFromPosition[i][0] != noteInitialPosition[i][0] ||
                                        shootFromPosition[i][1] != noteInitialPosition[i][1]) {
                                command = command
                                                .andThen(new LoadNoteCommand(robotContainer.intakeSubsystem,
                                                                robotContainer.grabberSubsystem,
                                                                robotContainer.shooterSubsystem)
                                                .alongWith(
                                                                new StraightPathCommand(
                                                                                robotContainer.drivetrainSubsystem,
                                                                                robotContainer.limeLightPoseSubsystem,
                                                                                new Pose2d(shootFromPosition[i][0],
                                                                                                shootFromPosition[i][1],
                                                                                                Rotation2d.fromDegrees(
                                                                                                                robotAngle)))));
                        } else {
                                command = command
                                                .andThen(new LoadNoteCommand(robotContainer.intakeSubsystem,
                                                                robotContainer.grabberSubsystem,
                                                                robotContainer.shooterSubsystem)
                                                                .alongWith(new SpeakerAlligningCommand(
                                                                                robotContainer.limeLightPoseSubsystem,
                                                                                robotContainer.drivetrainSubsystem)));
                        }
                        command = command
                                        .andThen(new ShootToSpeakerCommand(robotContainer.shooterSubsystem,
                                                        robotContainer.intakeSubsystem,
                                                        robotContainer.grabberSubsystem,
                                                        robotContainer.limeLightPoseSubsystem,
                                                        robotContainer.drivetrainSubsystem));
                        if (from < until) {
                                ++i;
                        } else {
                                --i;
                        }
                }
                return command;
        }

        public static Command getAutonomousCommand(RobotContainer robotContainer, int from, int until, boolean accending) {
                double robotAngle = (from >= 8) ? 0 : 180;
                
                double shootFromPosition[][] = (accending) ? shootAccendingPosition : shootDescendingPosition;

                Command command =
                                // new ResetOdometryWithCameraCommand(robotContainer.limeLightPoseSubsystem)
                                // .andThen(
                                new WaitUntilShooterReadyCommand(robotContainer.shooterSubsystem)
                                                .andThen(new GrabberOutCommand(robotContainer.grabberSubsystem))
                                                .andThen(new WaitCommand(0.3));
                if (from == -1) {
                        return command;
                }
                /**
                 * from can be higher than until.
                 * In that case, we go in reverse order
                 * look at the end of the loop to see how
                 * i gets incremented, or decremented.
                 */
                for (int i = from; i != until;) {
                       // command = command
                        //                .andThen(new IntakeNoteCommand(robotContainer.intakeSubsystem));
                                                        //IntakeCommand.State.OUT, 4000));
                        if (shootFromPosition[i][0] == noteInitialPosition[i][0] &&
                                        shootFromPosition[i][1] == noteInitialPosition[i][1]) {
                        //command = command
                        //              .andThen(new WaitUntilIntakeOutCommand(robotContainer.intakeSubsystem));
                        }
                        double pickupX = noteInitialPosition[i][0];                                                                                        
                        double pickupY = noteInitialPosition[i][1];
                        double pickupAlpha = robotAngle;

                        if (from != until) {
                                if (accending) {
                                        pickupX = accendingNoteInitialPosition[i][0];
                                        pickupY = accendingNoteInitialPosition[i][1];
                                        pickupAlpha = accendingNoteInitialPosition[i][2];
                                } else {
                                        pickupX = descendingNoteInitialPosition[i][0];
                                        pickupY = descendingNoteInitialPosition[i][1];
                                        pickupAlpha = descendingNoteInitialPosition[i][2];                                
                                }
                        }
                        
                        command = command
                                        .andThen(
                                                        new StraightPathCommand(robotContainer.drivetrainSubsystem,
                                                                        robotContainer.limeLightPoseSubsystem,
                                                                        new Pose2d(pickupX,
                                                                                        pickupY,
                                                                                        Rotation2d.fromDegrees(
                                                                                                        pickupAlpha)))
                                                                        .alongWith(new GrabberInCommand(
                                                                                        robotContainer.grabberSubsystem)))
                                        .andThen(grabNoteCommand(robotContainer));

                        if (shootFromPosition[i][0] != noteInitialPosition[i][0] ||
                                        shootFromPosition[i][1] != noteInitialPosition[i][1]) {
                                command = command
                                                .andThen(new LoadNoteCommand(robotContainer.intakeSubsystem,
                                                                robotContainer.grabberSubsystem,
                                                                robotContainer.shooterSubsystem)
                                                .alongWith(
                                                                new StraightPathCommand(
                                                                                robotContainer.drivetrainSubsystem,
                                                                                robotContainer.limeLightPoseSubsystem,
                                                                                new Pose2d(shootFromPosition[i][0],
                                                                                                shootFromPosition[i][1],
                                                                                                Rotation2d.fromDegrees(
                                                                                                                robotAngle)))));
                        } else {
                                command = command
                                                .andThen(new LoadNoteCommand(robotContainer.intakeSubsystem,
                                                                robotContainer.grabberSubsystem,
                                                                robotContainer.shooterSubsystem)
                                                                .alongWith(new SpeakerAlligningCommand(
                                                                                robotContainer.limeLightPoseSubsystem,
                                                                                robotContainer.drivetrainSubsystem)));
                        }
                        command = command
                                        .andThen(new ShootToSpeakerCommand(robotContainer.shooterSubsystem,
                                                        robotContainer.intakeSubsystem,
                                                        robotContainer.grabberSubsystem,
                                                        robotContainer.limeLightPoseSubsystem,
                                                        robotContainer.drivetrainSubsystem));
                        if (accending) {
                                ++i;
                        } else {
                                --i;
                        }
                        if (from < 8) {
                                if (i < 0) {
                                        i = 7;
                                } else if (i > 7) {
                                        i = 0;
                                }                                
                        } else {
                                if (i < 8) {
                                        i = 15;
                                } else if (i > 15) {
                                        i = 8;
                                }
                        }
                }
                return command;
        }

        public static Command grabNoteCommand(RobotContainer robotContainer) {
                return new GrabNoteCommandAutonomous(robotContainer, true);
                                // .andThen(new GrabberInCommand(robotContainer.grabberSubsystem))
                                // .andThen(new WaitCommand(0.1))
                                // .andThen(new GrabberInCommand(robotContainer.grabberSubsystem))
                                // .andThen(new WaitCommand(0.1))
                                // .andThen(new GrabberInCommand(robotContainer.grabberSubsystem))
                                // .andThen(new WaitCommand(0.1))
                                // .andThen(new GrabberInCommand(robotContainer.grabberSubsystem))
                                // .andThen(new WaitCommand(0.1))
                                // .andThen(new GrabberInCommand(robotContainer.grabberSubsystem))
                                // .andThen(new WaitCommand(0.1));
        }

        // public Command getAutonomousCommandCase2() {
        // Command command = new DisplayLogCommand("Case 2")
        // .andThen(new WaitCommand(0.5));
        // return command;
        // }

        // public Command caseCommand(String name, IntakeSubsystem intakeSubsystem,
        // double deltaY) {
        // return new ZeroGyroCommand(drivetrainSubsystem, balanceCommand, (180))
        // // .andThen(new SetModeConeCube(RobotMode.Cube))
        // // .andThen(new PositionCommand(this, OperatorButtons.LOW))
        // .andThen(new IntakeCommand(intakeSubsystem, IntakeCommand.State.OUT, 300))
        // // .andThen(new PositionCommand(this, OperatorButtons.HOME))
        // .andThen(
        // new StraightPathCommand(drivetrainSubsystem,
        // robotContainer.limeLightPoseSubsystem,
        // new Pose2d(4.49, 5.08,
        // new Rotation2d(Math.toRadians(180)))))
        // .andThen(new RotateCommand(drivetrainSubsystem))
        // .andThen(new StraightPathCommand(drivetrainSubsystem,
        // robotContainer.limeLightPoseSubsystem,
        // new Pose2d(4.79, 5.08, new Rotation2d(Math.toRadians(0)))))
        // // .andThen(new PositionCommand(robotContainer, OperatorButtons.GROUND))
        // .andThen(new IntakeCommand(intakeSubsystem, IntakeCommand.State.IN, 1000))
        // // .andThen(new PositionCommand(robotContainer, OperatorButtons.HOME))
        // .andThen(new RotateCommand(drivetrainSubsystem))
        // .andThen(new StraightPathCommand(drivetrainSubsystem,
        // robotContainer.limeLightPoseSubsystem,
        // new Pose2d(1.89, 4.88, new Rotation2d(Math.toRadians(180)))))
        // .andThen(new IntakeCommand(intakeSubsystem, IntakeCommand.State.OUT, 300));
        // }

        // public Command testMovements(DrivetrainSubsystem drivetrainSubsystem,
        // IntakeSubsystem intake,
        // RobotContainer robotContainer) {
        // return new DisplayLogCommand("Test Case")
        // // .andThen(new RobotOrientedDriveCommand(drivetrainSubsystem, .4, 0, 0,
        // 500))
        // // .andThen(new SetModeConeCube(RobotMode.Cube))
        // // .andThen(new RobotOrientedDriveCommand(drivetrainSubsystem, 0, 0, 0, 50))
        // // .andThen(new PositionCommand(robotContainer, OperatorButtons.LOW))
        // .andThen(new IntakeCommand(intake, State.OUT, 1000))
        // .andThen(new WaitCommand(.5))
        // // .andThen(new PositionCommand(robotContainer, OperatorButtons.HOME))
        // .andThen(new RobotOrientedDriveCommand(drivetrainSubsystem, -.3, 0, 0, 1000))
        // .andThen(new WaitCommand(.25))
        // .andThen(new RotateCommand(drivetrainSubsystem))
        // // .andThen(new PositionCommand(robotContainer, OperatorButtons.GROUND))
        // .andThen(new IntakeCommand(intake, State.IN, 1000))
        // .andThen(new RobotOrientedDriveDeacceleratedCommand(drivetrainSubsystem, -.3,
        // 0, 0,
        // 300));
        // // .andThen(new RobotOrientedDriveCommand(drivetrainSubsystem, 0, 0, 0, 50));
        // }

        // private Supplier<Pose2d> poseEstimator;

        // public Command caseCommandOld(String name, IntakeSubsystem intakeSubsystem,
        // double speedY, int durationY) {
        // Command command = new ZeroGyroCommand(drivetrainSubsystem, balanceCommand,
        // (180))
        // // .andThen(new SetModeConeCube(RobotMode.Cube))
        // // .andThen(new PositionCommand(robotContainer, OperatorButtons.HIGH))
        // .andThen(new IntakeCommand(intakeSubsystem, IntakeCommand.State.OUT, 1000))
        // // .andThen(new PositionCommand(robotContainer, OperatorButtons.HOME))
        // .andThen(new RobotOrientedDriveDeacceleratedCommand(drivetrainSubsystem, 0,
        // speedY, 0,
        // durationY))
        // .andThen(new WaitCommand(0.1))
        // .andThen(new RobotOrientedDriveDeacceleratedCommand(drivetrainSubsystem, -.7,
        // 0, 0,
        // 1500))
        // .andThen(new RotateCommand(drivetrainSubsystem))
        // // .andThen(new PositionCommand(robotContainer, OperatorButtons.GROUND))
        // .andThen(new IntakeCommand(intakeSubsystem, IntakeCommand.State.IN, 1000));
        // command.setName(name);
        // return command;
        // }

        // public Command test2() {
        // Command command = new ZeroGyroCommand(drivetrainSubsystem, balanceCommand,
        // (180))
        // .andThen(new StraightPathCommand(drivetrainSubsystem, poseEstimator,
        // new Pose2d(new Translation2d(2.1,
        // KeyPadPositionSupplier.FIELD_WIDTH - 5.24),
        // new Rotation2d(Math.toRadians(180)))))
        // .andThen(new DriveCommand(drivetrainSubsystem, -2, 0, 100))
        // .andThen(new WaitCommand(0.5))
        // .andThen(new DriveCommand(drivetrainSubsystem, -2, 0, 0))
        // .andThen(new WaitCommand(2.5))
        // .andThen(new DriveCommand(drivetrainSubsystem, -0.05, 0, 0))
        // .andThen(new WaitCommand(2))
        // .andThen(new DriveCommand(drivetrainSubsystem, 0, 0, 0))
        // .andThen(new BalanceCommand(drivetrainSubsystem));
        // command.setName("Test 2");
        // return command;
        // }

        // public Command getOverAndBalanceCommand(DrivetrainSubsystem
        // m_drivetrainSubsystem) {
        // BalanceCommand balanceCommand = new BalanceCommand(m_drivetrainSubsystem);

        // return new Command() {
        // @Override
        // public void initialize() {
        // DefaultDriveCommand.autonomous = true;
        // SwerveModule.setPowerRatio(1.5);
        // balanceCommand.zeroGyroscope();
        // }

        // @Override
        // public boolean isFinished() {
        // return true;
        // }

        // }
        // // .andThen(new SetModeConeCube(RobotMode.Cube))
        // // .andThen(new PositionCommand(robotContainer, OperatorButtons.HIGH))
        // .andThen(new IntakeCommand(robotContainer.intakeSubsystem,
        // IntakeCommand.State.OUT,
        // 300))
        // // .andThen(new PositionCommand(robotContainer, OperatorButtons.HOME))
        // .andThen(new RobotOrientedDriveCommand(m_drivetrainSubsystem, -0.03, 0, 0,
        // 3500))
        // .andThen(new RobotOrientedDriveCommand(m_drivetrainSubsystem, 0.02, 0, 0,
        // 800))
        // .andThen(balanceCommand);
        // }

        // public static Command getPieceWithCoral(RobotContainer robotContainer,
        // LimeLightPoseSubsystem limeLightPoseSubsystem,
        // String splineGetCube, String splineDropCube, Pose2d cubePose) {
        // DrivetrainSubsystem drivetrainSubsystem = robotContainer.drivetrainSubsystem;
        // IntakeSubsystem intakeSubsystem = robotContainer.intakeSubsystem;
        // return
        // // new SetModeConeCube(RobotMode.Cube)
        // // .andThen(
        // new IntakeCommand(intakeSubsystem, IntakeCommand.State.OUT, 300)// )
        // .andThen(new TrajectoryUntilSeeingCube(splineGetCube,
        // drivetrainSubsystem, limeLightPoseSubsystem))
        // // .andThen(new PositionCommand(robotContainer, OperatorButtons.GROUND))
        // .andThen(new IntakeCommand(intakeSubsystem, IntakeCommand.State.IN, 2500)
        // .alongWith(new DriveToObjectCommand(drivetrainSubsystem, "cube")))//
        // .andThen(new
        // // PositionCommand(robotContainer,
        // // OperatorButtons.HOME))
        // .andThen(new TrajectoryCommand(splineDropCube,
        // drivetrainSubsystem, limeLightPoseSubsystem))
        // .andThen(new IntakeCommand(intakeSubsystem, IntakeCommand.State.OUT, 300));
        // }

        // static Command grabCube(
        // RobotContainer robotContainer,
        // DrivetrainSubsystem drivetrainSubsystem,
        // LimeLightPoseSubsystem limeLightPoseSubsystem, IntakeSubsystem
        // intakeSubsystem,
        // Pose2d cubePose) {
        // // return new PositionCommand(robotContainer, OperatorButtons.GROUND)
        // // .andThen(
        // return new IntakeCommand(intakeSubsystem, IntakeCommand.State.IN, 2500)
        // .alongWith(new StraightPathCommand(drivetrainSubsystem,
        // limeLightPoseSubsystem, cubePose));// );
        // }

        // autonomousChooser.setDefaultOption("Over and Balance",

        // AutonomousCommandFactory.getAutonomousSimpleLowCommand(m_drivetrainSubsystem,
        // m_armSubsystem, grabberSubsystem)
        // .andThen(AutonomousCommandFactory.getOverAndBalanceCommand(m_drivetrainSubsystem,
        // poseEstimator)));
        // // A chooser for autonomous commands
        // autonomousChooser.setDefaultOption("Middle Balance",
        // AutonomousCommandFactory.getAutonomousSimpleCommand(m_drivetrainSubsystem,
        // m_armSubsystem, grabberSubsystem)
        // .andThen(AutonomousCommandFactory.getSetPositionAndBalanceCommand(m_drivetrainSubsystem,
        // poseEstimator)));
        // autonomousChooser.addOption("Simple Case and Left out",
        // AutonomousCommandFactory.getAutonomousAcceleratedAndLeftOutCommand(m_drivetrainSubsystem,
        // m_armSubsystem,
        // grabberSubsystem));
        // autonomousChooser.addOption("Simple Case and Right out",
        // AutonomousCommandFactory.getAutonomousSimpleAndRightDeacceleratedOutCommand(m_drivetrainSubsystem,
        // m_armSubsystem,
        // grabberSubsystem));

        // // Add commands to the autonomous command chooser
        // autonomousChooser.addOption("Case 1 left",
        // getAutonomousCommandCase1(0).andThen(new
        // StraightPathCommand(m_drivetrainSubsystem,
        // getPoseEstimatorForTarget(poseEstimator, 2),
        // getFinalPoseForCase1(0))));

        // autonomousChooser.addOption("Case 1 middle", getAutonomousCommandCase1(1));
        // autonomousChooser.addOption("CaSe 1 right", getAutonomousCommandCase1(2)
        // .andThen(new StraightPathCommand(m_drivetrainSubsystem,
        // getPoseEstimatorForTarget(poseEstimator, 0),
        // getFinalPoseForCase1(2))));

        // autonomousChooser.addOption("Case 2 left", getAutonomousCommandCase2(0));
        // autonomousChooser.addOption("Case 1 middle", getAutonomousCommandCase1(1));
        // autonomousChooser.addOption("Case 2 red", getAutonomousCommandCase2Red());
        // autonomousChooser.addOption("Case 2 blue", getAutonomousCommandCase2Blue());
        // autonomousChooser.addOption("Case 2 right", getAutonomousCommandCase2(2));
        // autonomousChooser.addOption("Case 3", getAutonomousCommandCase3());

        // public Command getAutonomousCommandCase2Red() {
        // Command command = new ZeroGyroCommand(drivetrainSubsystem, balanceCommand,
        // (180))

        // // .andThen(new GrabberCommand(grabberSubsystem, false))
        // // .andThen(new KeyPadStateCommand(1))
        // // .andThen(getCommandFor(1))
        // // .andThen(new GrabberCommand(grabberSubsystem, true))
        // // .andThen(new WaitCommand(0.5))
        // // .andThen(new ZeroExtenderCommand(m_armSubsystem))
        // // .andThen(new StraightPathCommand(m_drivetrainSubsystem, poseEstimator,
        // // new Pose2d(new Translation2d(2.1, 5.24), new
        // // Rotation2d(Math.toRadians(180)))))
        // // .andThen(new ZeroShoulderCommand(m_armSubsystem))
        // // .andThen(new ChangeTurboModeCommand())
        // // .andThen(new DriveCommand(m_drivetrainSubsystem, -1, 0, 0))
        // .andThen(new WaitCommand(0.5));
        // // .andThen(new ChangeNormalModeCommand())
        // // .andThen(new DriveCommand(m_drivetrainSubsystem, -1, 0, 0))
        // // .andThen(new WaitCommand(2.5))
        // // // .andThen(new DriveCommand(m_drivetrainSubsystem, -0.05,0,0))
        // // // .andThen(new WaitCommand(2))
        // // .andThen(new DriveCommand(m_drivetrainSubsystem, 0, 0, 0));
        // // // .andThen(new BalanceCommand(m_drivetrainSubsystem))
        // command.setName("Case 2 red");
        // return command;
        // }

        // public Command getAutonomousCommandCase2Blue() {
        // Command command = new ZeroGyroCommand(m_drivetrainSubsystem,
        // balanceCommand, (180))
        // .andThen(new GrabberCommand(grabberSubsystem, false))
        // .andThen(new KeyPadStateCommand(1))
        // .andThen(getCommandFor(1))
        // .andThen(new GrabberCommand(grabberSubsystem, true))
        // .andThen(new WaitCommand(0.5))
        // .andThen(new ZeroExtenderCommand(m_armSubsystem))
        // .andThen(new StraightPathCommand(m_drivetrainSubsystem, poseEstimator,
        // new Pose2d(new Translation2d(2.1, KeyPadPositionSupplier.FIELD_WIDTH - 5.24),
        // new Rotation2d(Math.toRadians(180)))))
        // .andThen(new ZeroShoulderCommand(m_armSubsystem))
        // .andThen(new ChangeTurboModeCommand())
        // .andThen(new DriveCommand(m_drivetrainSubsystem, -1, 0, 0))
        // .andThen(new WaitCommand(0.5))
        // .andThen(new ChangeNormalModeCommand())
        // .andThen(new DriveCommand(m_drivetrainSubsystem, -1, 0, 0))
        // .andThen(new WaitCommand(2.5))
        // // .andThen(new DriveCommand(m_drivetrainSubsystem, -0.05,0,0))
        // // .andThen(new WaitCommand(2))
        // .andThen(new DriveCommand(m_drivetrainSubsystem, 0, 0, 0));
        // // .andThen(new BalanceCommand(m_drivetrainSubsystem))
        // command.setName("Case 2 blue");
        // return command;
        // }

        // public Command getAutonomousCommandCase3() {
        // KeyPadPositionSupplier.state = 0;
        // Command command = new Command() {
        // @Override
        // public void initialize() {
        // KeyPadPositionSupplier.state = 0;
        // }

        // @Override
        // public boolean isFinished() {
        // return true;
        // }

        // }.andThen(getCommandFor(0)
        // .andThen(new GrabberCommand(grabberSubsystem, true))
        // // .andThen(new WaitCommand(1))
        // .andThen(new ZeroExtenderCommand(m_armSubsystem))
        // // .andThen(
        // // new StraightPathCommand(m_drivetrainSubsystem, poseEstimator,
        // // new Pose2d(5.75,
        // // 7.4,
        // // new Rotation2d(Math.toRadians(180)))))
        // .andThen(
        // new StraightPathCommand(m_drivetrainSubsystem,
        // getPoseEstimatorForTarget(poseEstimator, 0),
        // new Pose2d(4.4,
        // 5.3,
        // new Rotation2d(Math.toRadians(0)))))
        // .andThen(
        // new StraightPathCommand(m_drivetrainSubsystem,
        // getPoseEstimatorForTarget(poseEstimator, 0),
        // new Pose2d(6.4,
        // 5.3,
        // new Rotation2d(Math.toRadians(0)))))
        // .andThen(new ShoulderCommand(m_armSubsystem, 40352))
        // .andThen(new ExtenderCommand(m_armSubsystem, 183023 * 16 / 36))
        // .andThen(new GrabberCommand(grabberSubsystem, false))
        // .andThen(new GrabberCommand(grabberSubsystem, false))
        // .andThen(new WaitCommand(2))
        // .andThen(new ZeroExtenderCommand(m_armSubsystem))
        // .andThen(Commands.parallel(
        // new ShoulderCommand(m_armSubsystem, 190432),
        // new StraightPathCommand(m_drivetrainSubsystem,
        // getPoseEstimatorForTarget(poseEstimator, 0),
        // new Pose2d(3.2, 5.3,
        // new Rotation2d(Math.toRadians(0))))))
        // .andThen(getCommandFor(4))
        // .andThen(new GrabberCommand(grabberSubsystem, true)));
        // command.setName("case 3");
        // return command;
        // }

}