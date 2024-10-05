package frc.robot;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AimTiltToSpeaker;
import frc.robot.commands.AutoIntakeNoteCommand;
import frc.robot.commands.AutoShootWithAngleCommand;
import frc.robot.commands.IntakeNoteCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SpeakerAlligningCommand;
import frc.robot.commands.TiltHomeCommand;
import frc.robot.commands.TiltSetAngleCommand;

public class Autonomous {
    private SendableChooser<Command> autoChooser;

    public Autonomous(RobotContainer robotContainer) {
        NamedCommands.registerCommand("intake",
                new AutoIntakeNoteCommand(robotContainer.intakeSubsystem, robotContainer.indexerSubsystem));
        NamedCommands.registerCommand("firstShoot", new AutoShootWithAngleCommand(robotContainer.shooterSubsystem,
                robotContainer.indexerSubsystem, robotContainer.tiltSubsystem, .55, 45));
        NamedCommands.registerCommand("farSideFirstShoot", new AutoShootWithAngleCommand(robotContainer.shooterSubsystem,
                robotContainer.indexerSubsystem, robotContainer.tiltSubsystem, .80, 62));
        NamedCommands.registerCommand("homeShooter", new TiltSetAngleCommand(robotContainer.tiltSubsystem, 10.0));
        NamedCommands.registerCommand("shoot", new AutoShootWithAngleCommand(robotContainer.shooterSubsystem,
                robotContainer.indexerSubsystem, robotContainer.tiltSubsystem, .55, 10));
        NamedCommands.registerCommand("fancyShoot", Commands
                .parallel(new AimTiltToSpeaker(robotContainer.tiltSubsystem, robotContainer.poseSubsystem, false),
                        new SpeakerAlligningCommand(robotContainer.poseSubsystem, robotContainer.drivetrainSubsystem))
                .andThen(new ShootCommand(robotContainer.shooterSubsystem, robotContainer.indexerSubsystem,
                        robotContainer.poseSubsystem, 1.0)));

        AutoBuilder.configureHolonomic(
                robotContainer.poseSubsystem::get,
                robotContainer.poseSubsystem::setCurrentPose,
                robotContainer.drivetrainSubsystem::getChassisSpeeds,
                robotContainer.drivetrainSubsystem::drive,
                Constants.PATH_FOLLOWER_CONFIG,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                // Reference to this subsystem to set requirements
                robotContainer.drivetrainSubsystem);

        FollowPathCommand.warmupCommand().schedule();

        autoChooser = AutoBuilder.buildAutoChooser("Basic 4 Note");
        // SmartDashboard.putData(autoChooser);
        
        ShuffleboardTab tab = Shuffleboard.getTab("Odometry");
        tab.add("chooser", autoChooser).withPosition(0, 1).withSize(2, 0);
    }

    public Command getAutonomousCommand() {
        if (autoChooser == null) {
            return null;
        }
        return autoChooser.getSelected();
    }

    public Map<String, Command> getAllAutosByName() {
        Map<String, Command> map = new HashMap<String, Command>();

        for (String auto : AutoBuilder.getAllAutoNames()) {
            map.put(auto, new PathPlannerAuto(auto));
        }

        return map;
    }

    public String getSelectedAutoName() {
        // Auto chooser stores the name of the selected auto but doesnt make it
        // publiclly accessiable. Here, we access it anyways with reflection
        try {
            Field f = autoChooser.getClass().getField("m_selected");
            f.setAccessible(true);
            return (String) f.get(autoChooser);
        } catch (Exception e) {
            e.printStackTrace();
        }

        // Just return nothing incase our hack didnt work
        return "";
    }
}