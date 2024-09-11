package frc.robot.commands;

import static frc.robot.utilities.Util.logf;
//import static frc.robot.Constants.isMini;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveToObjectCommand extends Command {
    private DrivetrainSubsystem drivetrainSubsystem;
    private CoralSubsystem coralSubsystem;
    private double x;
    private double area;
    private double finishArea = 10;
    private double finishX = 0.002;
    // private String type;
    private double startTime;
    private double timeout;

    /** Creates a new ReplaceMeCommand. */
    public DriveToObjectCommand(DrivetrainSubsystem drivetrainSubsystem, CoralSubsystem coralSubsystem, String type) {
        this(drivetrainSubsystem, coralSubsystem, type, 1000);
    }

    public DriveToObjectCommand(DrivetrainSubsystem drivetrainSubsystem, CoralSubsystem coralSubsystem, String type,
            double timeout) {
        this.timeout = timeout;
        // Use addRequirements() here to declare subsystem dependencies.
        this.drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);
        // this.type = type;
        this.coralSubsystem = coralSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logf("Starting Drive To Object Command\n");
        startTime = RobotController.getFPGATime() / 1000;
        state = State.IDLE;
        counter = 0;
        finished = false;
    }

    enum State {
        IDLE,
        START,
        TIMER_START,
        FINISHED
    };

    State state;
    int counter = 0;
    double startTimeToGetTheNote = 0;
    // Called every time the scheduler runs while the command is scheduled

    @Override
    public void execute() {
        double omegaSpeed = 0;
        double xSpeed = 0;
        switch (state) {
            case IDLE:
                if (coralSubsystem.percent == 1) {
                    state = State.START;
                } else {
                    drivetrainSubsystem.drive(new ChassisSpeeds(-0.0048, 0, Math.toRadians(0)));
                }
                break;
            case START:
                area = coralSubsystem.area;
                if (// (area > finishArea)||
                (coralSubsystem.percent != 1)) {
                    if (counter++ > 3) {
                        state = State.TIMER_START;
                        startTimeToGetTheNote = RobotController.getFPGATime() / 1000;
                    }
                    drivetrainSubsystem.drive(new ChassisSpeeds(-0.012, 0, 0));
                    // drivetrainSubsystem.drive(new ChassisSpeeds(-0.3, 0, 0));
                    break;
                } else {
                    counter = 0;
                }
                x = -coralSubsystem.x;
                x += 2.5;
                if (Math.abs(x) > finishX) {
                    omegaSpeed = x * x * 0.003;
                    if (x < 0) {
                        omegaSpeed = -omegaSpeed;
                    }
                }
                xSpeed = -0.022;
                drivetrainSubsystem.drive(new ChassisSpeeds(xSpeed, 0, Math.toRadians(omegaSpeed)));
                break;
            case TIMER_START:
                if (startTimeToGetTheNote + 1500 < RobotController.getFPGATime() / 1000) {
                    state = State.FINISHED;
                } else {
                    drivetrainSubsystem.drive(new ChassisSpeeds(-0.022, 0, Math.toRadians(0)));
                }
                break;
            case FINISHED:
                drivetrainSubsystem.stop();
                break;
        }

        logf("Coral state:%s x:%.4f y:%.4f area:%.3f PerCent:%.3f xSpeed:%.5f omegaSpeed:%.5f\n",
                state, coralSubsystem.x, coralSubsystem.y, coralSubsystem.area, coralSubsystem.percent, xSpeed,
                omegaSpeed);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean value = (startTime + timeout < RobotController.getFPGATime() / 1000) || state == State.FINISHED;
        if (value) {
            logf("Drive to object command area = " + area + " finish area = " + finishArea + "\n");
        }
        return value || finished;
        // if (!coral.type.equals(type)) {
        // logf("Coral invalid type: coral:%s requested:%s\n", coral.type, type);
        // return true;
        // }
        // return (Math.abs(x) < finishX && coral.area > finishArea);
    }

    boolean finished = false;

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        finished = true;
        drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
        logf("Drive to object end\n");
    }
}