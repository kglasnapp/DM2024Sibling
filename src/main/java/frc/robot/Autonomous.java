package frc.robot;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoIntakeNoteCommand;
import frc.robot.commands.AutoShootWithAngleCommand;
import frc.robot.commands.IntakeNoteCommand;
import frc.robot.commands.TiltHomeCommand;

public class Autonomous {
    private SendableChooser<Command> autoChooser;

    public Autonomous(RobotContainer robotContainer) {
        NamedCommands.registerCommand("intake",
                new AutoIntakeNoteCommand(robotContainer.intakeSubsystem, robotContainer.indexerSubsystem));
        NamedCommands.registerCommand("firstShoot", new AutoShootWithAngleCommand(robotContainer.shooterSubsystem,
                robotContainer.indexerSubsystem, robotContainer.tiltSubsystem, .55, 55));
        NamedCommands.registerCommand("homeShooter", new TiltHomeCommand(robotContainer.tiltSubsystem));
        NamedCommands.registerCommand("shoot", new AutoShootWithAngleCommand(robotContainer.shooterSubsystem,
                robotContainer.indexerSubsystem, robotContainer.tiltSubsystem, .55, 10));

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

        autoChooser = AutoBuilder.buildAutoChooser("New Auto");
        SmartDashboard.putData(autoChooser);
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