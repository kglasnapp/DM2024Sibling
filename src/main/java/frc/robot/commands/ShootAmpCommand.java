package frc.robot.commands;

    import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightPoseSubsystem;
import frc.robot.subsystems.ShooterSubsystemOld;
import static frc.robot.utilities.Util.logf;

    
public class ShootAmpCommand extends Command {
    
        ShooterSubsystemOld shooterSubsystem;
        IntakeSubsystem intakeSubsystem;
        GrabberSubsystem grabberSubsystem;
        LimeLightPoseSubsystem poseEstimatorSubsystem;
        double angle;
        double power;
        double startTime;
        boolean finished = false;
        State state = State.STARTING_SHOOTER_MOTORS;
        /**
         * Time in milliseconds needed to wait to ensure the shooter reached the
         * desired speed
         */
        public final static double TIME_THRESHOLD_SHOOTER = 500;
        /**
         * Time in milliseconds needed for the shooter to push the note out after
         * the grabber sent it, before we shut down the shooter.
         */
        public final static double TIME_THRESHOLD_AFTER_SHOOT = 500;
    
        /**
         * Number of ticks we allow as a threshold before we accept that the shooter
         * reach
         * the desired angle for the intake
         */
        public final static double INTAKE_ANGLE_TICKS_THRESHOLD = 500;
    
        /**
         * Number of DEGRESS we allow as a threshold before we accept that the shooter
         * reach
         * the desired angle for the shooter
         */
        public final static double SHOOTER_ANGLE_DEGREES_THRESHOLD = 1;
        
        public static enum State {
            STARTING_SHOOTER_MOTORS, CHECK_INTAKE_ANGLE, CHECK_SHOOTER_ANGLE, GRABBER_OUT, END
        };
    
        public ShootAmpCommand(ShooterSubsystemOld shooterSubsystem,
                IntakeSubsystem intakeSubsystem,
                GrabberSubsystem grabberSubsystem,
                LimeLightPoseSubsystem poseEstimatorSubsystem) {
            this.shooterSubsystem = shooterSubsystem;
            this.intakeSubsystem = intakeSubsystem;
            this.grabberSubsystem = grabberSubsystem;
        }
    
        @Override
        public void initialize() {        
            shooterSubsystem.servo.setAngle(ShooterSubsystemOld.FLAPPER_EXPANDED_ANGLE);  // Extended

            logf("####################### set servo 130\n");
            shooterSubsystem.setTiltAngle(-4);
            shooterSubsystem.setShooterPower(0.13*0.85); //0.155);
            startTime = RobotController.getFPGATime() / 1000;
        }
    
        @Override
        public void execute() {
        }
    
        @Override
        public boolean isFinished() {
            return true;
        }
    }
    
