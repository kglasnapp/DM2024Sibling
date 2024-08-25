package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightPoseSubsystem;
import frc.robot.subsystems.ShooterSubsystemOld;
import edu.wpi.first.math.geometry.Pose2d;
import static frc.robot.utilities.Util.logf;

public class ShootToSpeakerCommand extends Command {
    ShooterSubsystemOld shooterSubsystem;
    IntakeSubsystem intakeSubsystem;
    GrabberSubsystem grabberSubsystem;
    LimeLightPoseSubsystem poseEstimatorSubsystem;
    //SpeakerAlligningCommand shootSpeakerAlligningCommand;
    DrivetrainSubsystem drivetrainSubsystem;
    double power = .9;
    double startTime;
    boolean finished = false;


    State state = State.CHECK_INTAKE_ANGLE_TO_FEED;
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
    public final static double INTAKE_ANGLE_POS_THRESHOLD = 1;

    /**
     * 
     * Number of DEGRESS we allow as a threshold before we accept that the shooter
     * reach
     * the desired angle for the shooter
     */
    public final static double SHOOTER_ANGLE_DEGREES_THRESHOLD = 3;

    /**
     * Multiplier of the distance to calculate the shooter angle
     */
    public final static double ANGLE_RATIO = 1;

    /**
     * Multiplier of the distance to calculate the shooter power
     */
    public final static double POWER_RATIO = 1;

    public final static double MIN_ANGLE_TO_FEED = -10;

    public static enum State {
        CHECK_INTAKE_ANGLE_TO_FEED,
        //FEED_NOTE,
        NOTE_WENT_OUT,
        CHECK_SHOOTER_ANGLE_AND_ROBOT_POSE,
        TRIGGER,
        //TRIGGER_OUT,
        SEE_NOTE_GOING_OUT
    };

    public ShootToSpeakerCommand(ShooterSubsystemOld shooterSubsystem,
            IntakeSubsystem intakeSubsystem,
            GrabberSubsystem grabberSubsystem,
            LimeLightPoseSubsystem poseEstimatorSubsystem,
            DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.grabberSubsystem = grabberSubsystem;
        this.poseEstimatorSubsystem = poseEstimatorSubsystem;
        // this.shootSpeakerAlligningCommand = new SpeakerAlligningCommand(poseEstimatorSubsystem, drivetrainSubsystem);
        addRequirements(shooterSubsystem, intakeSubsystem, grabberSubsystem);
    }

    double shooterAngleToFeed = MIN_ANGLE_TO_FEED;
    double angle;

    @Override
    public void initialize() {
        finished = false;
        notSeeTheNoteCounter = 0;
        // shootSpeakerAlligningCommandFinished = false;
        state = State.CHECK_SHOOTER_ANGLE_AND_ROBOT_POSE;
        angle = calculateTiltAngle();
        shooterSubsystem.setTiltAngle(angle);
        shooterSubsystem.setShooterPower(1);
        startTime = RobotController.getFPGATime() / 1000;
        // shootSpeakerAlligningCommand.initialize();
        shooterSubsystem.servo.setAngle(ShooterSubsystemOld.FLAPPER_RETRACTED_ANGLE);
        logf("Start Shoot speaker command\n");
    }


    double angleTable[][] = {
        { 1.31, -8.05 , .6},
        { 1.71, -13.39, .6 },
        { 2.06, -15.50, .7 },
        { 2.37, -21.0, .7 },
        { 2.67, -23. , .8 },
        { 2.99, -24, .8},
        { 3.25, -25.1 , 9},
        { 3.70, -26, .9 }
    };


    public double calculateTiltAngle() {
        Pose2d pose = poseEstimatorSubsystem.get();
        Alliance alliance = DriverStation.getAlliance().get();
        Pose2d speakerPose = alliance == Alliance.Blue ? RobotContainer.BLUE_SPEAKER : RobotContainer.RED_SPEAKER;
        double distance = distance(speakerPose, pose);
        // TODO should set angle to a hightr number -- If distance to close set angle 
        if (distance <= 1.31) {
            return 0;
        }
        for (int i=1;i<angleTable.length;++i) {
            if (distance <= angleTable[i][0]) {
                double m = (angleTable[i][1] - angleTable[i-1][1])/(angleTable[i][0] - angleTable[i-1][0]);
                double b = angleTable[i][1] - (m * angleTable[i][0]);
                return m*distance + b;
            }
        }
        double m = (angleTable[angleTable.length-1][1] - angleTable[angleTable.length-2][1])/(angleTable[angleTable.length-1][0] - angleTable[angleTable.length-2][0]);
        double b = angleTable[angleTable.length-1][1] - (m * angleTable[angleTable.length-1][0]);
        angle = m*distance + b;
        // double dsqr = distance * distance;
        // double angle = 4.4263 * dsqr * distance - 23.6759 * dsqr + 30.1972 * distance - 12.9287;
        // // the robot physically cannot move more than 30 degress. We are adding a
        // // software limit in here.
        // if (distance > 3.488) {
        //     angle = -3.284 * distance - 10.116;
        // }
        // // angle = -8 * distance + 7.63;

        // // angle = -(60 - angle);
        if (angle < -30) {
            angle = -30;
        }
        if (angle > 0) {
            angle = 0;
        }
logf("Get tilt paraameters for target dist:%.2f angle:%.2f power:%.2f\n", distance, angle, power);

        return angle;
    }

    public static double distance(Pose2d pose1, Pose2d pose2) {
        double dx = pose1.getX() - pose2.getX();
        double dy = pose1.getY() - pose2.getY();

        return Math.sqrt(dx * dx + dy * dy);
    }

    State lastState = null;

    // boolean shootSpeakerAlligningCommandFinished = false;
    int notSeeTheNoteCounter = 0;
    @Override
    public void execute() {
        if (Robot.count % 10 == 0) {
            SmartDashboard.putString("strSt", state.toString());
            if (state != lastState) {
                lastState = state;
                logf("Shooting Command state: %s\n", state.toString());
            }
        }
        // if (!shootSpeakerAlligningCommand.isFinished() && !shootSpeakerAlligningCommandFinished) {
        //     shootSpeakerAlligningCommand.execute();
        // } else if (!shootSpeakerAlligningCommandFinished) {
        //     shootSpeakerAlligningCommand.end(true);
        //     shootSpeakerAlligningCommandFinished = true;
        // }

        switch (state) {
            case CHECK_SHOOTER_ANGLE_AND_ROBOT_POSE:
                // if (Robot.count % 10 == 0) {
                //     logf("Tilt Angle = %.2f target angle = %.2f\n", shooterSubsystem.getTiltAngle(), angle);
                // }
                if (Math.abs(shooterSubsystem.getTiltAngle() - angle) < SHOOTER_ANGLE_DEGREES_THRESHOLD
                        // && shootSpeakerAlligningCommandFinished
                        ) {
                    logf("Tilt Angle = %.2f target angle = %.2f\n", shooterSubsystem.getTiltAngle(), angle);
                    state = State.TRIGGER;
                }
                break;
            case TRIGGER:
                logf("Shooter speeds: " + shooterSubsystem.getShootVelocity() + " "
                        + shooterSubsystem.getShootVelocity2() + " grabber current %.2f\n", grabberSubsystem.getCurrent());
                //double avgVelocity = (shooterSubsystem.getShootVelocity() + shooterSubsystem.getShootVelocity2()) / 2;
                if (shooterSubsystem.getShootVelocity() > 6300
                        && shooterSubsystem.getShootVelocity2() > 6300) {
                    // (Math.abs(RobotController.getFPGATime() / 1000 - startTime) >
                    // TIME_THRESHOLD_SHOOTER) {
                    logf("Shoot\n");
                    grabberSubsystem.grabberOut();
                    state = State.SEE_NOTE_GOING_OUT;
                    startTime = RobotController.getFPGATime() / 1000;
                }
                break;
            case SEE_NOTE_GOING_OUT:
                logf("END STATUS start time: "+startTime+" currentTime:"+(RobotController.getFPGATime() / 1000)+" grabber current: %.2f\n",
                grabberSubsystem.getCurrent());

                if (!grabberSubsystem.seeNote()) {
                    if (notSeeTheNoteCounter > 3) {
                        startTime = RobotController.getFPGATime() / 1000;
                        state = State.NOTE_WENT_OUT;
                    }
                    notSeeTheNoteCounter++;
                } else {
                    notSeeTheNoteCounter = 0;
                }

                // if (((RobotController.getFPGATime() / 1000) - startTime > 100 &&
                //         (Math.abs(grabberSubsystem.getCurrent()) < 0.35) && (Math.abs(grabberSubsystem.getCurrent())!=0))||
                //     (RobotController.getFPGATime() / 1000) - startTime > TIME_THRESHOLD_AFTER_SHOOT) {
                //     logf("Shooter speeds after shoot: " + shooterSubsystem.getShootVelocity() + " "
                //             + shooterSubsystem.getShootVelocity2() + "\n");
                //     shooterSubsystem.setShooterPower(0.3);
                //     grabberSubsystem.grabberOff();
                //     finished = true;
                // }
                break;
            case NOTE_WENT_OUT:
                if (startTime + 300 < RobotController.getFPGATime() / 1000) {
                    shooterSubsystem.setShooterPower(0.3);
                    grabberSubsystem.grabberOff();
                    finished = true;
                }
                break;
            case CHECK_INTAKE_ANGLE_TO_FEED:
                break;
        }

    }

    @Override
    public void end(boolean interrupted) {
        // shooterSubsystem.setShooterPower(0.3);
        // if (!shootSpeakerAlligningCommandFinished) {
            //shootSpeakerAlligningCommand.end(interrupted);
        // }
        drivetrainSubsystem.stop();
        grabberSubsystem.grabberOff();
        logf("*************************Shoot to speaker finished\n");
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
